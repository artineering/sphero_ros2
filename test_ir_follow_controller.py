#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Same circle-follow test as test_ir_follow_proc.py, but driven through the
Sphero INSTANCE CONTROLLER wrapper (core/sphero/sphero.py) instead of raw
Sensor calls -- this exercises the IR methods we added to the controller.

  SB-3660  -> leader.start_ir_broadcast(0, 1) + drives a circle.  LED = RED.
  SB-3AAC  -> follower.start_ir_follow(0, 1).                     LED = BLUE.
  SB-33E3  -> follower.start_ir_follow(0, 1).                     LED = GREEN.

The controller's IR methods send commands on proc=Processors.SECONDARY
internally and take (near, far) order (swapping to the Sensor (far, near)).

Run from the repo root:  python3 test_ir_follow_controller.py
"""

import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                'src', 'sphero_instance_controller'))

from spherov2 import scanner
from spherov2.sphero_edu import SpheroEduAPI

from sphero_instance_controller.core.sphero.sphero import Sphero
from sphero_instance_controller.spherov2_collision_patch import apply_collision_patch

apply_collision_patch()

# --- Config ---
BROADCASTER_NAME = 'SB-3660'
# Each follower: (name, (r, g, b) LED color)
FOLLOWERS = [
    ('SB-3AAC', (0, 0, 255)),   # BLUE
    ('SB-33E3', (0, 255, 0)),   # GREEN
]

NEAR_CODE = 0
FAR_CODE = 1
RUN_SECONDS = 120

# Leader circle drive (tune for arena size; smaller speed / bigger heading step = tighter circle).
LEADER_SPEED = 80
HEADING_STEP_DEG = 15
STEP_INTERVAL = 0.2


def connect(name):
    """Scan for, connect to, and wrap a Sphero. Returns (Sphero, context_manager)."""
    print(f"[{name}] scanning...", flush=True)
    robot = scanner.find_toy(toy_name=name)
    print(f"[{name}] found {robot}, connecting...", flush=True)
    cm = SpheroEduAPI(toy=robot)
    api = cm.__enter__()
    print(f"[{name}] connected", flush=True)
    return Sphero(robot=robot, api=api, name=name), cm


def main():
    # Connect sequentially (avoids concurrent-connect BLE contention).
    leader, leader_cm = connect(BROADCASTER_NAME)

    followers = []  # list of (name, Sphero, cm)
    for name, _color in FOLLOWERS:
        sph, cm = connect(name)
        followers.append((name, sph, cm))

    try:
        leader.set_led(255, 0, 0)   # RED = leader
        for (name, sph, _cm), (_n, (r, g, b)) in zip(followers, FOLLOWERS):
            sph.set_led(r, g, b)

        print(f"[{BROADCASTER_NAME}] start_ir_broadcast({NEAR_CODE}, {FAR_CODE}):",
              leader.start_ir_broadcast(NEAR_CODE, FAR_CODE), flush=True)

        for name, sph, _cm in followers:
            print(f"[{name}] start_ir_follow({NEAR_CODE}, {FAR_CODE}):",
                  sph.start_ir_follow(NEAR_CODE, FAR_CODE), flush=True)

        print(f"\n>>> RED (leader) drives a circle while broadcasting. "
              f"Watch the followers track it for {RUN_SECONDS}s. <<<\n", flush=True)

        # Leader drives a circle: set speed once, then sweep heading continuously.
        leader.set_speed(LEADER_SPEED)
        heading = 0
        end = time.time() + RUN_SECONDS
        while time.time() < end:
            heading = (heading + HEADING_STEP_DEG) % 360
            leader.set_heading(heading)
            time.sleep(STEP_INTERVAL)

    finally:
        for name, sph, _cm in followers:
            sph.stop_ir_follow()
            sph.stop()
            sph.set_led(0, 0, 0)
        leader.stop()
        leader.stop_ir_broadcast()
        leader.set_led(0, 0, 0)
        for _name, _sph, cm in followers:
            cm.__exit__(None, None, None)
        leader_cm.__exit__(None, None, None)
        print("disconnected", flush=True)


if __name__ == '__main__':
    main()
