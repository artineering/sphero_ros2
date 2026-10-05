#!/usr/bin/env python3
"""Record a drive for offline Kalman-filter tuning: camera + commands + LEDs.

Each robot gets front LED green / back LED red (heading.py's pair) with the
main panel off, re-sent every 2 s. Each robot then gets a solo nudge so the
offline step can tell which blob is which (by motion -- LED writes land
seconds late). Then all robots drive: a slow square, a circle (heading steps
at 2 Hz), a faster square.

  <out>/raw.mjpeg    every camera frame as received
  <out>/frames.csv   frame, t        (wall clock at frame arrival)
  <out>/cmds.csv     t, robot, speed, heading   from /sphero/<ns>/motion_cmd
  <out>/leds.csv     t, robot, event

Usage (ROS sourced):
  python3 scripts/kf_record.py --robots SB-3660 SB-418F SB-5B47
"""
import argparse
import json
import os
import threading
import time

import rclpy
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import String

from blob_detect_timing import open_camera


def schedule(robots, roll, led, log, stop):
    def all_roll(speed, h):
        for r in robots:
            roll(r, speed, h)

    def leds_on():
        for r in robots:
            led(r, 'main', 0, 0, 0)
            led(r, 'front', 0, 255, 0)
            led(r, 'back', 255, 0, 0)

    # LED writes over BLE are slow and sometimes dropped: re-assert every 2 s.
    def keep_lit():
        while not stop.is_set():
            leds_on()
            stop.wait(2.0)
    threading.Thread(target=keep_lit, daemon=True).start()
    time.sleep(4.0)                                        # LEDs land, still frames

    for r in robots:                                       # solo nudge = identity
        log(r, 'nudge')
        roll(r, 50, 0)
        time.sleep(0.6)
        roll(r, 0, 0)
        time.sleep(2.5)

    for speed, leg in ((70, 1.2), (120, 0.8)):
        for h in (0, 90, 180, 270):                        # square
            all_roll(speed, h)
            time.sleep(leg)
            all_roll(0, h)
            time.sleep(1.0)
        if speed == 70:                                    # circle between squares
            h = 0                                          # 2 Hz: what the device
            for _ in range(24):                            # controller sustains
                all_roll(60, h)
                h = (h + 30) % 360
                time.sleep(0.5)
            all_roll(0, h)
            time.sleep(1.5)
    time.sleep(1.5)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--robots', nargs='+', required=True)
    ap.add_argument('--device', default='/dev/video0')
    ap.add_argument('--out', default=os.path.expanduser(
        f'~/sphero_ros2/recordings/kf_{time.strftime("%Y%m%d_%H%M%S")}'))
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)
    ns = {r: r.replace('-', '_') for r in args.robots}

    rclpy.init()
    node = rclpy.create_node('kf_record')
    cmds = open(os.path.join(args.out, 'cmds.csv'), 'w')
    cmds.write('t,robot,speed,heading\n')
    leds = open(os.path.join(args.out, 'leds.csv'), 'w')
    leds.write('t,robot,event\n')

    def on_cmd(msg, r):
        d = json.loads(msg.data)
        cmds.write(f'{time.time():.4f},{r},{d.get("speed", 0)},{d.get("heading", 0)}\n')

    roll_pub, led_pub = {}, {}
    for r in args.robots:
        node.create_subscription(String, f'/sphero/{ns[r]}/motion_cmd',
                                 lambda m, r=r: on_cmd(m, r), 50)
        roll_pub[r] = node.create_publisher(String, f'/sphero/{ns[r]}/roll', 10)
        led_pub[r] = node.create_publisher(String, f'/sphero/{ns[r]}/led', 10)
    ex = SingleThreadedExecutor()
    ex.add_node(node)
    threading.Thread(target=ex.spin, daemon=True).start()
    time.sleep(1.0)                                        # let DDS match

    def roll(r, speed, h):
        roll_pub[r].publish(String(data=json.dumps({'heading': h % 360, 'speed': speed})))

    def led(r, kind, red, green, blue):
        led_pub[r].publish(String(data=json.dumps(
            {'type': kind, 'red': red, 'green': green, 'blue': blue})))

    def log(r, event):
        leds.write(f'{time.time():.4f},{r},{event}\n')

    cap = open_camera(args.device)
    for _ in range(30):
        cap.read()
    done = threading.Event()

    def drive():
        try:
            schedule(args.robots, roll, led, log, done)
        finally:
            done.set()
    threading.Thread(target=drive, daemon=True).start()

    raw = open(os.path.join(args.out, 'raw.mjpeg'), 'wb')
    frames = open(os.path.join(args.out, 'frames.csv'), 'w')
    frames.write('frame,t\n')
    i = 0
    try:
        while not done.is_set():
            ok, buf = cap.read()
            if not ok:
                continue
            frames.write(f'{i},{time.time():.4f}\n')
            raw.write(buf.tobytes())
            i += 1
    finally:
        for r in args.robots:
            roll(r, 0, 0)
        time.sleep(0.3)
        raw.close()
        frames.close()
        cap.release()
        cmds.close()
        leds.close()
        ex.shutdown()
        node.destroy_node()
        rclpy.shutdown()
    print(f'{i} frames -> {args.out}')


if __name__ == '__main__':
    main()
