#!/usr/bin/env python3
"""Diagnostic: capture a live Kinect depth frame and visualize the foreground
segmentation so we can see WHY registration finds ~40 candidates.

Saves to ~/kinect_field/:
  depth_live.png      colorized live depth; NO-RETURN (0) pixels in MAGENTA
  depth_baseline.png  colorized empty-arena heightmap
  depth_diff.png      (baseline - live) heat: positive = closer than floor
  mask_closer.png     foreground rule #1: baseline-live > fg_threshold (subtraction)
  mask_holes.png      foreground rule #2: valid floor in baseline, EMPTY in live
  depth_overlay.png   live depth with the field polygon (from calibration) drawn

The two masks answer A vs B: if the candidate BAND shows up in mask_holes it's
IR drop-out (holes); if it shows up in mask_closer it's a baseline<->live
subtraction mismatch (camera moved / far-edge noise / stale baseline).

PREREQUISITES (rpi5): STOP field_tracker_node first (it owns the Kinect);
source install/setup.bash. Usage:  python3 scripts/depth_debug.py
"""
import os

import cv2
import numpy as np
import yaml

from kinect_field_tracking.detection import FreenectSource, capture_heightmap

OUT = os.path.expanduser("~/kinect_field")
HEIGHTMAP = os.path.join(OUT, "kinect_heightmap.npy")
FIELD_YAML = os.path.join(OUT, "kinect_field.yaml")
FG_THRESHOLD_MM = 25.0
FG_HEIGHT_MAX_MM = 300.0
N_FRAMES = 15            # average the live capture to match how detection sees it


def colorize(depth, lo=600.0, hi=2600.0):
    """Colorize a depth (mm) image; 0/invalid -> magenta so holes are visible."""
    d = depth.astype(np.float32)
    norm = np.clip((d - lo) / (hi - lo), 0, 1)
    img = cv2.applyColorMap((norm * 255).astype(np.uint8), cv2.COLORMAP_JET)
    img[depth <= 0] = (255, 0, 255)   # BGR magenta = no return
    return img


def main():
    os.makedirs(OUT, exist_ok=True)
    base = np.load(HEIGHTMAP).astype(np.float32)
    print(f"baseline {HEIGHTMAP}: valid px={int((base>0).sum())}/{base.size}")

    src = FreenectSource()
    src.stop()
    for _ in range(3):
        src.get_depth()
    live = capture_heightmap(src, N_FRAMES)   # per-pixel mean of valid returns
    src.close()
    print(f"live: valid px={int((live>0).sum())}/{live.size}")

    valid_bg = base > 0
    diff = base - live
    closer = valid_bg & (live > 0) & (diff > FG_THRESHOLD_MM) & (diff < FG_HEIGHT_MAX_MM)
    holes = valid_bg & (live <= 0)
    print(f"closer-than-floor px (rule #1, subtraction): {int(closer.sum())}")
    print(f"hole px           (rule #2, no return)     : {int(holes.sum())}")

    cv2.imwrite(os.path.join(OUT, "depth_live.png"), colorize(live))
    cv2.imwrite(os.path.join(OUT, "depth_baseline.png"), colorize(base))
    # diff heat: only where both valid
    dh = np.zeros_like(diff)
    both = valid_bg & (live > 0)
    dh[both] = np.clip(diff[both], -100, 200)
    dh = ((dh + 100) / 300 * 255).astype(np.uint8)
    cv2.imwrite(os.path.join(OUT, "depth_diff.png"), cv2.applyColorMap(dh, cv2.COLORMAP_JET))
    cv2.imwrite(os.path.join(OUT, "mask_closer.png"), (closer.astype(np.uint8) * 255))
    cv2.imwrite(os.path.join(OUT, "mask_holes.png"), (holes.astype(np.uint8) * 255))

    # overlay field polygon (depth-frame pixels) if calibration present
    overlay = colorize(live)
    try:
        with open(FIELD_YAML) as f:
            fc = yaml.safe_load(f)["field_calibration"]
        pts = np.array(fc["corners_px"], dtype=np.int32).reshape(-1, 1, 2)
        cv2.polylines(overlay, [pts], True, (255, 255, 255), 2)
    except Exception as e:  # noqa: BLE001
        print(f"(no field polygon overlay: {e})")
    cv2.imwrite(os.path.join(OUT, "depth_overlay.png"), overlay)

    print(f"\nwrote depth_live/baseline/diff + mask_closer/mask_holes + depth_overlay to {OUT}")


if __name__ == "__main__":
    main()
