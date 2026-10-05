#!/usr/bin/env python3
"""Blob detection on moving robots: drive a speed sweep while detecting live.

Each connected robot does forward/back bursts (heading h, then h+180, so it
ends near where it started) at increasing speeds, stopping in between. Legs
are long enough to reach speed: commands take ~1 s to show up as motion.
The camera loop runs blob_detect_timing.detect() on every frame and records:

  <out>/raw.mjpeg     every frame as received (replay / convert to AVI)
  <out>/blobs.csv     frame, t, phase, x, y, area, w, h per blob
  <out>/fail_*.png    annotated frames whose blob count != the baseline

The baseline is the blob count on the still frames before driving starts.
Results are binned by measured blob speed (px/frame), not by commanded phase.
Drives via the fleet's /sphero/<NAME>/roll (indefinite roll + speed 0 stop, so
the single-threaded device controller is never blocked by a duration roll).

Usage (ROS sourced):
  python3 scripts/blob_motion_test.py --robots SB-3660 SB-418F SB-5B47
"""
import argparse
import json
import os
import threading
import time

import cv2
import numpy as np
import rclpy
from std_msgs.msg import String

from blob_detect_timing import detect, make_roi, open_camera, threshold_map

# (speed, seconds per leg). Legs shrink as speed rises to keep travel short.
SWEEP = [(100, 1.0), (150, 1.0), (200, 0.8), (255, 0.7)]
PAUSE_S = 2.0


def drive(node, pubs, heading, phase, stop_evt):
    def roll(speed, h):
        for p in pubs:
            p.publish(String(data=json.dumps({'heading': h % 360, 'speed': speed})))

    time.sleep(2.0)                                        # still frames for baseline
    for speed, leg in SWEEP:
        for h in (heading, heading + 180):
            phase[0] = f's{speed}'
            roll(speed, h)
            time.sleep(leg)
            roll(0, h)
            phase[0] = 'stop'
            time.sleep(PAUSE_S)
    roll(0, heading)
    time.sleep(1.0)
    stop_evt.set()


def annotate(g, blobs, text):
    img = cv2.cvtColor(g, cv2.COLOR_GRAY2BGR)
    for b in blobs:
        x, y, w, h = b['BoundingBox']
        cv2.rectangle(img, (x, y), (x + w, y + h), (0, 255, 255), 2)
        cv2.putText(img, str(b['Area']), (x + w + 4, y + 12), 0, 0.6, (0, 255, 255), 1)
    cv2.putText(img, text, (20, 40), 0, 1.2, (0, 0, 255), 2)
    return img


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--robots', nargs='+', required=True)
    ap.add_argument('--heading', type=int, default=0)
    ap.add_argument('--device', default='/dev/video0')
    ap.add_argument('--ratio', type=float, default=3.2)
    ap.add_argument('--min-area', type=int, default=200)
    ap.add_argument('--margin', type=int, default=25)
    ap.add_argument('--out', default=os.path.expanduser(
        f'~/sphero_ros2/recordings/motion_{time.strftime("%Y%m%d_%H%M%S")}'))
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)

    rclpy.init()
    node = rclpy.create_node('blob_motion_test')
    pubs = [node.create_publisher(String, f'/sphero/{r.replace("-", "_")}/roll', 10)
            for r in args.robots]
    time.sleep(1.0)                                        # let DDS match subscribers

    cap = open_camera(args.device)
    for _ in range(30):
        cap.read()
    roi = make_roi(args.margin)
    tmap = threshold_map([cv2.imdecode(cap.read()[1], cv2.IMREAD_GRAYSCALE)
                          for _ in range(5)], roi, args.ratio)

    phase, stop_evt = ['still'], threading.Event()
    threading.Thread(target=drive, args=(node, pubs, args.heading, phase, stop_evt),
                     daemon=True).start()

    raw = open(os.path.join(args.out, 'raw.mjpeg'), 'wb')
    csv = open(os.path.join(args.out, 'blobs.csv'), 'w')
    csv.write('frame,t,phase,x,y,area,w,h\n')
    rows, times, baseline, fails, i, t0 = [], [], None, 0, 0, time.perf_counter()
    try:
        while not stop_evt.is_set():
            ok, buf = cap.read()
            if not ok:
                continue
            t = time.perf_counter() - t0
            raw.write(buf.tobytes())
            a = time.perf_counter()
            g = cv2.imdecode(buf, cv2.IMREAD_GRAYSCALE)
            blobs = detect(g, roi, tmap, args.min_area)
            times.append(time.perf_counter() - a)
            ph = phase[0]
            if baseline is None and ph == 'still' and i == 30:
                baseline = len(blobs)
            for b in blobs:
                (x, y), (_, _, w, h) = b['Centroid'], b['BoundingBox']
                csv.write(f'{i},{t:.4f},{ph},{x:.2f},{y:.2f},{b["Area"]},{w},{h}\n')
            rows.append((i, ph, len(blobs), [b['Centroid'] for b in blobs],
                         [b['Area'] for b in blobs]))
            if baseline is not None and len(blobs) != baseline and fails < 40:
                cv2.imwrite(os.path.join(args.out, f'fail_{i:05d}.png'),
                            annotate(g, blobs, f'frame {i} {ph}: {len(blobs)} blobs '
                                               f'(expected {baseline})'))
                fails += 1
            i += 1
    finally:
        for p in pubs:
            p.publish(String(data=json.dumps({'heading': args.heading, 'speed': 0})))
        raw.close()
        csv.close()
        cap.release()
        node.destroy_node()
        rclpy.shutdown()

    report(rows, times, baseline, args.out)


def report(rows, times, baseline, out):
    ms = np.array(times) * 1e3
    print(f'output:          {out}')
    print(f'frames:          {len(rows)}   baseline blobs: {baseline}')
    print(f'decode+detect:   mean {ms.mean():.2f} ms  p95 {np.percentile(ms, 95):.2f}  '
          f'max {ms.max():.2f}')
    good = sum(n == baseline for _, _, n, _, _ in rows)
    print(f'correct count:   {good}/{len(rows)} ({100 * good / len(rows):.1f}%)')

    # Speed of the fastest blob per frame: nearest-neighbour step from the last
    # frame that had the full count (so a missed robot cannot fake a jump).
    bins = [(0, 2), (2, 5), (5, 10), (10, 20), (20, 1e9)]
    stats = {b: [0, 0, []] for b in bins}
    prev = None
    for _, _, n, cents, areas in rows:
        c = np.array(cents) if cents else np.zeros((0, 2))
        if prev is None or not len(c):
            prev = c if n == baseline else prev
            continue
        d = np.linalg.norm(c[:, None] - prev[None], axis=2).min(axis=1)
        j = int(d.argmax())
        b = next(b for b in bins if b[0] <= d[j] < b[1])
        stats[b][0] += 1
        stats[b][1] += n == baseline
        stats[b][2].append(areas[j])
        if n == baseline:
            prev = c

    print(f'\n{"fastest blob":>16s} {"frames":>6s} {"correct":>8s}  its area min/median')
    for (lo, hi), (n, ok, a) in stats.items():
        if n:
            label = f'{lo:g}-{hi:g} px/fr' if hi < 1e9 else f'>{lo:g} px/fr'
            print(f'{label:>16s} {n:6d} {100 * ok / n:7.1f}%  {min(a)}/{int(np.median(a))}')


if __name__ == '__main__':
    main()
