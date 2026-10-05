#!/usr/bin/env python3
"""Time the MATLAB blob pipeline, ported to OpenCV, on the live overhead camera.

MATLAB                                  OpenCV
  f = read(v,i); g = rgb2gray(f)          MJPEG decoded straight to gray (luma only)
  b = imbinarize(g)                       per-pixel threshold = --ratio x local floor
  cb = bwconncomp(b, 8)                   cv2.findContours (outer, 8-connected)
  stats = regionprops(Area,Centroid,BB)     pixel moments of each blob's patch
  in = inpolygon(cx, cy, px, py)          cv2.pointPolygonTest, --margin px slack
  filtered = stats(in)                    + fragments joined, Area >= --min-area,
                                          touching pairs split

Threshold: the arena is lit unevenly -- the floor reads ~19 in the middle and
~8 in the corners, and robots dim with it (gray p90 ~160 mid, ~55 corner), so
no single gray level works. Measured against the LOCAL floor, though, every
robot is 3.4-5.6x (blob median), the blue tape <= 2.75x and floor noise
<= 2.5x. The floor map is a morphological opening (61 px > a ~40 px robot, so
robots and tape vanish) of a still frame, taken once at startup; per frame it
costs one compare against the precomputed map.

Edges: thresholding runs inside the arena polygon grown by --margin px, and the
centroid -- not the pixel mask -- decides membership (as inpolygon does), so a
robot straddling the boundary keeps its whole blob. The centroid may sit up to
--margin px outside: the calibrated edge runs ~a robot radius inside the wall.

A robot rolling with its dark side up fragments into pieces under --min-area;
each piece under it joins its nearest piece within 25 px (less than a robot).

Touching robots merge into one blob of ~2x robot area. A blob of at least
1.6x the frame's median blob area is split into round(area / median) parts by
k-means on its pixel coordinates.

The camera has no gray format (YUYV is 5 fps at 1920x1200), but JPEG stores
luma as its own plane: taking the raw MJPEG bytes and decoding with
IMREAD_GRAYSCALE skips chroma decode and the RGB->gray conversion entirely.

The camera is opened with the overhead_tracking settings (1920x1200 MJPG @ 60,
manual exposure, gain 120, AWB off). Controls are applied after streaming
starts because UVC drops them on stream-on.

Usage: python3 scripts/blob_detect_timing.py [--frames 600] [--device /dev/video0]
       python3 scripts/blob_detect_timing.py --replay recordings/<run>/raw.mjpeg
"""
import argparse
import subprocess
import time

import cv2
import numpy as np

# Arena corners (px), TL TR BR BL, from ~/overhead_field/overhead_arena.yaml
ARENA_PX = np.array([[318.5, 60.0], [1660.0, 83.5], [1680.5, 1056.0], [300.0, 1065.0]])
CONTROLS = 'auto_exposure=1,gain=120,white_balance_automatic=0,exposure_dynamic_framerate=0'


def make_roi(margin=25):
    """Crop around the arena grown by `margin` px; polygon kept crop-relative."""
    x0, y0, w, h = cv2.boundingRect(ARENA_PX.astype(np.float32))
    x0, y0 = max(0, x0 - margin), max(0, y0 - margin)
    w, h = min(1920 - x0, w + 2 * margin), min(1200 - y0, h + 2 * margin)
    poly = (ARENA_PX - (x0, y0)).astype(np.float32)
    mask = np.zeros((h, w), np.uint8)
    cv2.fillPoly(mask, [np.round(poly).astype(np.int32)], 255)
    k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * margin + 1, 2 * margin + 1))
    return {'sl': (slice(y0, y0 + h), slice(x0, x0 + w)), 'mask': cv2.dilate(mask, k),
            'poly': poly, 'offset': np.array([x0, y0]), 'margin': margin}


def threshold_map(still_grays, roi, ratio):
    """Per-pixel threshold = ratio x local floor brightness, from still frames."""
    g = np.median(np.stack([f[roi['sl']] for f in still_grays]), axis=0).astype(np.uint8)
    k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (61, 61))
    floor = cv2.morphologyEx(cv2.medianBlur(g, 5), cv2.MORPH_OPEN, k)
    floor = cv2.GaussianBlur(floor.astype(np.float32), (0, 0), 15)
    return np.clip(np.round(ratio * np.maximum(floor, 1)), 0, 255).astype(np.uint8)


def _blob(area, cx, cy, x, y, w, h, off, split=False):
    return {'Area': int(area), 'Centroid': (float(cx + off[0]), float(cy + off[1])),
            'BoundingBox': (int(x + off[0]), int(y + off[1]), int(w), int(h)),
            'Split': split}


def detect(g, roi, tmap, min_area, link_px=25):
    crop = g[roi['sl']]
    b = cv2.compare(crop, tmap, cv2.CMP_GT) & roi['mask']
    # Outer contours (8-connected, like bwconncomp(b,8)) instead of labelling the
    # whole 1.5 MP crop: 0.9 ms vs 8.5 ms. Area/centroid are then exact pixel
    # moments of each blob's own small patch, so they match regionprops.
    contours, _ = cv2.findContours(b, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    parts = []
    for c in contours:
        x, y, w, h = cv2.boundingRect(c)
        if w * h < 10:                                    # single-pixel noise
            continue
        mo = cv2.moments(c)
        if mo['m00'] > 0:
            parts.append((c, mo['m10'] / mo['m00'], mo['m01'] / mo['m00'], x, y, w, h))
        else:
            parts.append((c, x + w / 2, y + h / 2, x, y, w, h))
    # A robot rolling with its dark side up breaks into fragments (e.g. 100+82+41
    # px) that each fall under min_area. Each part smaller than min_area joins
    # its single nearest part within link_px (< one ~35 px robot); full-size
    # parts never initiate a join, so two robots cannot be chained together.
    group = list(range(len(parts)))

    def root(i):
        while group[i] != i:
            group[i] = group[group[i]]
            i = group[i]
        return i
    if parts:
        xy = np.array([(p[1], p[2]) for p in parts])
        d2 = ((xy[:, None] - xy[None]) ** 2).sum(axis=2)
        np.fill_diagonal(d2, np.inf)
        for i, p in enumerate(parts):
            j = int(d2[i].argmin())
            if cv2.contourArea(p[0]) < min_area and d2[i, j] < link_px ** 2:
                group[root(i)] = root(j)
    groups = {}
    for i in range(len(parts)):
        groups.setdefault(root(i), []).append(parts[i])

    found = []
    for members in groups.values():
        x = min(p[3] for p in members)
        y = min(p[4] for p in members)
        w = max(p[3] + p[5] for p in members) - x
        h = max(p[4] + p[6] for p in members) - y
        if w * h < min_area:
            continue
        m = np.zeros((h, w), np.uint8)
        cv2.drawContours(m, [p[0] - (x, y) for p in members], -1, 255, -1)
        m &= b[y:y + h, x:x + w]
        mo = cv2.moments(m, binaryImage=True)
        if mo['m00'] < min_area:
            continue
        cx, cy = x + mo['m10'] / mo['m00'], y + mo['m01'] / mo['m00']
        # centroid may sit up to `margin` outside: the calibrated edge runs about
        # a robot radius inside the wall, and a robot cannot be past the wall
        if cv2.pointPolygonTest(roi['poly'], (cx, cy), True) < -roi['margin']:
            continue
        found.append((int(mo['m00']), cx, cy, x, y, w, h, m))
    if not found:
        return []
    ref = float(np.median([f[0] for f in found]))
    off, out = roi['offset'], []
    for area, cx, cy, x, y, w, h, m in found:
        k = int(round(area / ref)) if len(found) >= 3 and area >= 1.6 * ref else 1
        if k < 2:
            out.append(_blob(area, cx, cy, x, y, w, h, off))
            continue
        ys, xs = np.nonzero(m)
        pts = np.column_stack([xs + x, ys + y]).astype(np.float32)
        crit = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_MAX_ITER, 20, 0.5)
        _, lab, centers = cv2.kmeans(pts, k, None, crit, 3, cv2.KMEANS_PP_CENTERS)
        for j in range(k):
            p = pts[lab.ravel() == j]
            px0, py0 = p.min(axis=0)
            px1, py1 = p.max(axis=0)
            out.append(_blob(len(p), *centers[j], px0, py0, px1 - px0 + 1, py1 - py0 + 1,
                             off, split=True))
    return out


def open_camera(device):
    cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1200)
    cap.set(cv2.CAP_PROP_FPS, 60)
    cap.set(cv2.CAP_PROP_CONVERT_RGB, 0)                  # hand back raw MJPEG bytes
    if not cap.isOpened() or not cap.read()[0]:
        raise SystemExit(f'cannot read {device} (is the tracker node holding it?)')
    time.sleep(1.5)
    subprocess.run(['v4l2-ctl', '-d', device, '-c', CONTROLS], check=False)
    return cap


def camera_frames(cap):
    while True:
        ok, buf = cap.read()                              # raw JPEG, no decode
        if not ok:
            return
        yield buf


def replay_frames(path):
    data = open(path, 'rb').read()
    s = data.find(b'\xff\xd8\xff')
    while s >= 0:
        e = data.find(b'\xff\xd8\xff', s + 3)
        yield np.frombuffer(data[s:e if e >= 0 else len(data)], np.uint8)
        s = e


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--device', default='/dev/video0')
    ap.add_argument('--replay', help='raw .mjpeg file instead of the live camera')
    ap.add_argument('--frames', type=int, default=600)
    ap.add_argument('--warmup', type=int, default=30)
    ap.add_argument('--min-area', type=int, default=200,
                    help='px; a robot is ~400-850 px, LED/glint specks are 1-35')
    ap.add_argument('--ratio', type=float, default=3.2,
                    help='x local floor; robots 3.4-5.6, tape <= 2.75, floor <= 2.5')
    ap.add_argument('--margin', type=int, default=25, help='px grown around the arena')
    args = ap.parse_args()

    cap = None if args.replay else open_camera(args.device)
    frames = replay_frames(args.replay) if args.replay else camera_frames(cap)
    for _ in range(args.warmup):                          # let the controls settle
        next(frames)
    roi = make_roi(args.margin)
    tmap = threshold_map([cv2.imdecode(next(frames), cv2.IMREAD_GRAYSCALE)
                          for _ in range(5)], roi, args.ratio)

    t_read, t_decode, t_detect, counts, splits = [], [], [], [], 0
    t_start = time.perf_counter()
    for _ in range(args.frames):
        t0 = time.perf_counter()
        buf = next(frames, None)
        t1 = time.perf_counter()
        if buf is None:
            break
        g = cv2.imdecode(buf, cv2.IMREAD_GRAYSCALE)
        t2 = time.perf_counter()
        blobs = detect(g, roi, tmap, args.min_area)
        t3 = time.perf_counter()
        t_read.append(t1 - t0)
        t_decode.append(t2 - t1)
        t_detect.append(t3 - t2)
        counts.append(len(blobs))
        splits += any(b['Split'] for b in blobs)
    wall = time.perf_counter() - t_start
    if cap is not None:
        cap.release()

    d = np.array(t_detect) * 1e3
    r = np.array(t_read) * 1e3
    j = np.array(t_decode) * 1e3
    print(f'frames:            {len(d)}  ({len(d) / wall:.1f} fps end-to-end)')
    print(f'detect (ms):       mean {d.mean():.2f}  median {np.median(d):.2f}  '
          f'p95 {np.percentile(d, 95):.2f}  max {d.max():.2f}')
    print(f'gray decode (ms):  mean {j.mean():.2f}  median {np.median(j):.2f}  '
          f'p95 {np.percentile(j, 95):.2f}')
    print(f'decode+detect (ms): mean {(j + d).mean():.2f}')
    print(f'read wait (ms):    mean {r.mean():.2f}  (frame-arrival wait, not compute)')
    hist = dict(zip(*np.unique(counts, return_counts=True)))
    print(f'blobs in arena:    {{count: frames}} {hist}   frames with a split: {splits}')
    if blobs:
        print('last frame blobs:', blobs[:10])


if __name__ == '__main__':
    main()
