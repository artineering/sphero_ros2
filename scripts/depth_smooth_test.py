#!/usr/bin/env python3
"""Experiment (read-only): does blurring baseline + live depth before
differencing kill the depth_diff noise? Does NOT change detect_blobs.

Uses a MASKED (normalized) Gaussian blur so invalid (0) depth pixels don't bleed
zeros into valid regions: out = blur(depth*valid) / blur(valid).

Captures the live depth ONCE and caches it to ~/kinect_field/depth_live_raw.npy,
so you can sweep the kernel size offline without the camera:
    python3 scripts/depth_smooth_test.py --ksize 5
    python3 scripts/depth_smooth_test.py --ksize 9
    python3 scripts/depth_smooth_test.py --recapture     # grab a fresh live frame

Writes to ~/kinect_field/:
    depth_diff_raw.png      baseline-live, NO smoothing (current behaviour)
    depth_diff_smooth.png   baseline-live, both masked-blurred
    mask_closer_raw.png     closer-than-floor mask, raw
    mask_closer_smooth.png  closer-than-floor mask, smoothed
and prints the closer-pixel counts (raw vs smoothed) so the noise drop is numeric.

PREREQUISITE for capture: STOP field_tracker_node (it owns the Kinect);
source install/setup.bash.
"""
import argparse
import os

import cv2
import numpy as np

OUT = os.path.expanduser("~/kinect_field")
HEIGHTMAP = os.path.join(OUT, "kinect_heightmap.npy")
LIVE_NPY = os.path.join(OUT, "depth_live_raw.npy")
FG_THRESHOLD_MM = 25.0
FG_HEIGHT_MAX_MM = 300.0
N_FRAMES = 15


def masked_blur(depth, ksize, sigma):
    """Gaussian blur that ignores invalid (0) pixels (normalized by validity)."""
    valid = (depth > 0).astype(np.float32)
    num = cv2.GaussianBlur(depth.astype(np.float32) * valid, (ksize, ksize), sigma)
    den = cv2.GaussianBlur(valid, (ksize, ksize), sigma)
    out = np.zeros_like(depth, dtype=np.float32)
    nz = den > 1e-6
    out[nz] = num[nz] / den[nz]
    out[depth <= 0] = 0.0          # keep originally-invalid pixels invalid
    return out


def diff_heat(base, live):
    diff = base - live
    both = (base > 0) & (live > 0)
    dh = np.zeros_like(diff)
    dh[both] = np.clip(diff[both], -100, 200)
    return cv2.applyColorMap(((dh + 100) / 300 * 255).astype(np.uint8), cv2.COLORMAP_JET)


def closer_mask(base, live):
    d = base - live
    m = (base > 0) & (live > 0) & (d > FG_THRESHOLD_MM) & (d < FG_HEIGHT_MAX_MM)
    return m


def get_live(recapture):
    if os.path.exists(LIVE_NPY) and not recapture:
        print(f"loading cached live depth {LIVE_NPY}")
        return np.load(LIVE_NPY).astype(np.float32)
    from kinect_field_tracking.detection import FreenectSource, capture_heightmap
    print("capturing live depth from Kinect (field_tracker_node must be stopped)...")
    src = FreenectSource()
    src.stop()
    for _ in range(3):
        src.get_depth()
    live = capture_heightmap(src, N_FRAMES)
    src.close()
    np.save(LIVE_NPY, live)
    print(f"cached live depth -> {LIVE_NPY}")
    return live


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--ksize", type=int, default=5, help="Gaussian kernel (odd)")
    ap.add_argument("--sigma", type=float, default=0.0, help="0 = derived from ksize")
    ap.add_argument("--recapture", action="store_true", help="grab a fresh live frame")
    args = ap.parse_args()
    k = args.ksize | 1   # force odd

    base = np.load(HEIGHTMAP).astype(np.float32)
    live = get_live(args.recapture)

    base_s = masked_blur(base, k, args.sigma)
    live_s = masked_blur(live, k, args.sigma)

    cv2.imwrite(os.path.join(OUT, "depth_diff_raw.png"), diff_heat(base, live))
    cv2.imwrite(os.path.join(OUT, "depth_diff_smooth.png"), diff_heat(base_s, live_s))
    m_raw = closer_mask(base, live)
    m_smooth = closer_mask(base_s, live_s)
    cv2.imwrite(os.path.join(OUT, "mask_closer_raw.png"), m_raw.astype(np.uint8) * 255)
    cv2.imwrite(os.path.join(OUT, "mask_closer_smooth.png"), m_smooth.astype(np.uint8) * 255)

    print(f"\nksize={k} sigma={args.sigma}")
    print(f"closer-than-floor px  raw   : {int(m_raw.sum())}")
    print(f"closer-than-floor px  smooth: {int(m_smooth.sum())}")
    print(f"wrote depth_diff_raw/smooth + mask_closer_raw/smooth to {OUT}")


if __name__ == "__main__":
    main()
