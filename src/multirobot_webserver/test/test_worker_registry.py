#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Unit tests for the Phase 2 worker registry + least-loaded selection."""

import os

import pytest

from multirobot_webserver.worker_registry import (
    WorkerRegistry,
    DEFAULT_CAPACITY,
)

EXAMPLE = os.path.join(os.path.dirname(__file__), 'example_workers.yaml')


def make_registry():
    """All-online 4-worker registry, capacity 4 each."""
    return WorkerRegistry.from_config({
        'workers': [
            {'name': 'a', 'host': 'h-a'},
            {'name': 'b', 'host': 'h-b'},
            {'name': 'c', 'host': 'h-c'},
            {'name': 'd', 'host': 'h-d'},
        ]
    })


def test_default_capacity_is_four():
    reg = make_registry()
    assert all(w.capacity == 4 for w in reg.all())
    assert DEFAULT_CAPACITY == 4


def test_distributes_least_loaded():
    """16 assignments spread evenly: 4 per worker, least-loaded each step."""
    reg = make_registry()
    picks = []
    for _ in range(16):
        w = reg.select_worker()
        reg.assign(w)
        picks.append(w.name)
    # First 4 picks hit each worker once (deterministic config order on ties).
    assert picks[:4] == ['a', 'b', 'c', 'd']
    # Every worker ends at its cap of 4.
    assert {w.name: w.count for w in reg.all()} == {
        'a': 4, 'b': 4, 'c': 4, 'd': 4,
    }


def test_deterministic_tiebreak_by_config_order():
    reg = make_registry()
    # All free and equal -> first by config order.
    assert reg.select_worker().name == 'a'


def test_respects_per_worker_cap():
    reg = WorkerRegistry.from_config({
        'workers': [{'name': 'solo', 'host': 'h', 'capacity': 2}]
    })
    for _ in range(2):
        reg.assign(reg.select_worker())
    assert reg.get('solo').count == 2
    with pytest.raises(RuntimeError):
        reg.select_worker()


def test_rejects_when_all_full():
    reg = make_registry()
    for _ in range(16):
        reg.assign(reg.select_worker())
    with pytest.raises(RuntimeError) as exc:
        reg.select_worker()
    assert 'capacity' in str(exc.value)
    assert '16/16' in str(exc.value)


def test_skips_offline_workers():
    reg = make_registry()
    reg.set_online('a', False)
    # 'a' is excluded; remaining 3 workers cap the fleet at 12.
    for _ in range(12):
        reg.assign(reg.select_worker())
    assert reg.get('a').count == 0
    with pytest.raises(RuntimeError):
        reg.select_worker()


def test_release_frees_capacity():
    reg = WorkerRegistry.from_config({
        'workers': [{'name': 'x', 'host': 'h', 'capacity': 1}]
    })
    reg.assign(reg.select_worker())
    with pytest.raises(RuntimeError):
        reg.select_worker()
    reg.release('x')
    assert reg.select_worker().name == 'x'


def test_least_loaded_prefers_most_free_after_churn():
    """After uneven removal, the emptiest worker is chosen next."""
    reg = make_registry()
    # Fill a and b fully (4 each), leave c and d empty.
    for name in ('a', 'b'):
        for _ in range(4):
            reg.assign(reg.get(name))
    # Next pick should be the most-free worker, c (config order before d).
    assert reg.select_worker().name == 'c'


def test_from_yaml_example_file():
    reg = WorkerRegistry.from_yaml(EXAMPLE)
    names = [w.name for w in reg.all()]
    assert names == ['w1', 'w2', 'w3']
    # w3 is online: false -> excluded from selection.
    assert reg.get('w3').online is False
    for _ in range(8):  # w1 + w2 only, 4 each
        reg.assign(reg.select_worker())
    assert reg.get('w3').count == 0
    with pytest.raises(RuntimeError):
        reg.select_worker()
