#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Verify IR FOLLOW on BOLT, using the SECONDARY-processor fix.

  SB-3660  -> broadcasts on (near=0, far=1), stays stationary. LED = RED.
  SB-3AAC  -> follows on (near=0, far=1).                      LED = BLUE.
  SB-33E3  -> follows on (near=0, far=1).                      LED = GREEN.

All IR commands are sent with proc=Processors.SECONDARY (the fix proven by
test_ir_proc_probe.py: get_bot_to_bot_infrared_readings returned 0xf there).

The leader (red) drives itself in a circle while broadcasting; the followers
chase it via IR. Tune LEADER_SPEED / HEADING_STEP_DEG / STEP_INTERVAL for the
arena size (smaller speed / bigger heading step = tighter circle).

How to run the test physically:
  1. Put the followers (blue + green) on the floor, leader (red) in the middle,
     with clear space around them.
  2. The leader drives a circle on its own; watch the followers for the run:
       - track red around the circle  -> FOLLOW WORKS
       - only spin in place           -> following but not seeing red
       - scatter / do nothing         -> follow not engaging

Run from the repo root:  python3 test_ir_follow_proc.py
"""

import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                'src', 'sphero_instance_controller'))

from spherov2 import scanner
from spherov2.sphero_edu import SpheroEduAPI
from spherov2.types import Color
from spherov2.commands.sensor import Sensor
from spherov2.controls.v2 import Processors

from sphero_instance_controller.spherov2_collision_patch import apply_collision_patch

apply_collision_patch()

# --- Config ---
BROADCASTER_NAME = 'SB-3660'
# Each follower: (name, LED color)
FOLLOWERS = [
    ('SB-3AAC', Color(r=0, g=0, b=255)),   # BLUE
    ('SB-33E3', Color(r=0, g=255, b=0)),   # GREEN
]

NEAR_CODE = 0
FAR_CODE = 1
PROC = Processors.SECONDARY
RUN_SECONDS = 120

# Leader circle drive (tune for arena size; smaller speed / bigger heading step = tighter circle).
LEADER_SPEED = 40         # 0-255
HEADING_STEP_DEG = 15     # heading increment per step
STEP_INTERVAL = 0.2       # seconds per step  -> angular rate = HEADING_STEP_DEG / STEP_INTERVAL


def connect(name):
    print(f"[{name}] scanning...", flush=True)
    robot = scanner.find_toy(toy_name=name)
    print(f"[{name}] found {robot}, connecting...", flush=True)
    cm = SpheroEduAPI(toy=robot)
    api = cm.__enter__()
    print(f"[{name}] connected", flush=True)
    return robot, api, cm


def main():
    # Connect sequentially (avoids concurrent-connect BLE contention).
    b_toy, b_api, b_cm = connect(BROADCASTER_NAME)

    followers = []  # list of (name, toy, api, cm)
    for name, _color in FOLLOWERS:
        toy, api, cm = connect(name)
        followers.append((name, toy, api, cm))

    try:
        b_api.set_main_led(Color(r=255, g=0, b=0))   # RED = broadcaster (move by hand)
        for (name, _toy, api, _cm), (_n, color) in zip(followers, FOLLOWERS):
            api.set_main_led(color)

        # far_code, near_code per Sensor signature; all on SECONDARY processor.
        Sensor.start_robot_to_robot_infrared_broadcasting(b_toy, FAR_CODE, NEAR_CODE, proc=PROC)
        print(f"[{BROADCASTER_NAME}] broadcasting (far={FAR_CODE}, near={NEAR_CODE}) "
              f"on proc={PROC.name}", flush=True)

        for name, toy, _api, _cm in followers:
            Sensor.start_robot_to_robot_infrared_following(toy, FAR_CODE, NEAR_CODE, proc=PROC)
            print(f"[{name}] following (far={FAR_CODE}, near={NEAR_CODE}) "
                  f"on proc={PROC.name}", flush=True)

        print(f"\n>>> RED (leader) will drive itself in a circle while broadcasting. "
              f"Watch the followers track it for {RUN_SECONDS}s. <<<\n", flush=True)

        # Leader drives a circle: set speed once, then sweep heading continuously.
        b_api.set_speed(LEADER_SPEED)
        heading = 0
        end = time.time() + RUN_SECONDS
        while time.time() < end:
            heading = (heading + HEADING_STEP_DEG) % 360
            b_api.set_heading(heading)
            time.sleep(STEP_INTERVAL)

    finally:
        for name, toy, api, _cm in followers:
            try:
                Sensor.stop_robot_to_robot_infrared_following(toy, proc=PROC)
            except Exception:
                pass
            api.stop_roll()
            api.set_main_led(Color(r=0, g=0, b=0))
        b_api.stop_roll()
        try:
            Sensor.stop_robot_to_robot_infrared_broadcasting(b_toy, proc=PROC)
        except Exception:
            pass
        b_api.set_main_led(Color(r=0, g=0, b=0))
        for _name, _toy, _api, cm in followers:
            cm.__exit__(None, None, None)
        b_cm.__exit__(None, None, None)
        print("disconnected", flush=True)


if __name__ == '__main__':
    main()
