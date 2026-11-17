#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
BLE scan for advertising Spheros, serialized with the device controllers.

The scan takes the same per-host flock the device controllers hold around
their scan+connect phase (see ble_connect_lock in
sphero_instance_controller/sphero_instance_device_controller_node.py): same
lockfile (env SPHERO_BLE_CONNECT_LOCK, default below) and the same
LOCK_EX|LOCK_NB poll. Unlike the controller, a scan that can't get the lock
gives up (busy) instead of proceeding unlocked.
"""

import asyncio
import fcntl
import os
import time
from contextlib import contextmanager

# Must match DEFAULT_BLE_CONNECT_LOCK in sphero_instance_device_controller_node.py.
DEFAULT_BLE_CONNECT_LOCK = '/tmp/sphero_ble_connect.lock'
LOCK_POLL_S = 0.25
LOCK_SLACK_S = 5.0  # wait up to duration + this for the lock, then report busy

MIN_DURATION_S = 1.0
MAX_DURATION_S = 10.0
DEFAULT_DURATION_S = 4.0

# Fallback if spherov2 can't be imported; mirrors its toy_type prefixes.
FALLBACK_PREFIXES = ('SB-', 'BB-', 'GB-', 'SM-', '2B-', 'D2-', 'Q5-', 'RV-',
                     'SK-', 'Sphero')


def clamp_duration(raw) -> float:
    """Parse ?duration=, clamped to [1, 10]; missing/invalid -> 4."""
    try:
        value = float(raw)
    except (TypeError, ValueError):
        return DEFAULT_DURATION_S
    if value != value:  # NaN
        return DEFAULT_DURATION_S
    return max(MIN_DURATION_S, min(MAX_DURATION_S, value))


@contextmanager
def ble_lock(timeout):
    """Hold the per-host BLE flock; yields True if acquired within timeout."""
    lock_path = os.environ.get('SPHERO_BLE_CONNECT_LOCK', DEFAULT_BLE_CONNECT_LOCK)
    fd = open(lock_path, 'w')
    acquired = False
    deadline = time.monotonic() + timeout
    try:
        while True:
            try:
                fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
                acquired = True
                break
            except BlockingIOError:
                if time.monotonic() >= deadline:
                    break
                time.sleep(LOCK_POLL_S)
        yield acquired
    finally:
        if acquired:
            try:
                fcntl.flock(fd, fcntl.LOCK_UN)
            except OSError:
                pass
        fd.close()


def sphero_prefixes():
    """Advertised-name prefixes spherov2 recognizes as Sphero toys."""
    try:
        from spherov2 import scanner
        prefixes = {cls.toy_type.prefix or cls.toy_type.filter_prefix
                    for cls in scanner.all_toys()
                    if getattr(cls, 'toy_type', None)}
        return tuple(sorted(p for p in prefixes if p)) or FALLBACK_PREFIXES
    except Exception:  # noqa: BLE001 - any import/attr problem -> fallback
        return FALLBACK_PREFIXES


def discover(duration):
    """Raw bleak discovery: list of (name, address, rssi)."""
    from bleak import BleakScanner
    found = asyncio.run(BleakScanner.discover(timeout=duration, return_adv=True))
    return [(adv.local_name or dev.name, dev.address, adv.rssi)
            for dev, adv in found.values()]


def filter_and_dedupe(raw, prefixes):
    """Keep Sphero names, strongest RSSI per name, sorted strongest first."""
    best = {}
    for name, address, rssi in raw:
        if not name or not name.startswith(tuple(prefixes)):
            continue
        if name not in best or rssi > best[name]['rssi']:
            best[name] = {'name': name, 'address': address, 'rssi': int(rssi)}
    return sorted(best.values(), key=lambda r: r['rssi'], reverse=True)


def scan(duration, discover_fn=discover):
    """Scan under the BLE lock. Returns ('busy', None) or ('ok', robots)."""
    with ble_lock(duration + LOCK_SLACK_S) as acquired:
        if not acquired:
            return 'busy', None
        raw = discover_fn(duration)
    return 'ok', filter_and_dedupe(raw, sphero_prefixes())
