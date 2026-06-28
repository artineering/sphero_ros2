"""detect_blobs over the synthetic SimSource (no Kinect hardware)."""

import cv2
import numpy as np

from kinect_field_tracking.detection import (
    DetectConfig,
    Intrinsics,
    SimSource,
    detect_blobs,
    masked_blur,
)


def test_sim_detects_expected_blob_count():
    intr = Intrinsics()
    cfg = DetectConfig()
    n = 3
    src = SimSource(intr, cfg, n_spheres=n, seed=2)
    baseline = src.baseline()
    # warm up a few frames
    for _ in range(3):
        src.get_depth()
    depth = src.get_depth()
    meas, mask = detect_blobs(depth, baseline, intr, cfg)
    # robust: detect close to n blobs (balls may occasionally touch/merge)
    assert 1 <= len(meas) <= n + 1
    # measurement tuple is (X_mm, Y_mm, Z_mm, u, v)
    for m in meas:
        assert len(m) == 5


def _floor_with_blob_and_speckle():
    """Flat floor (2000mm) + one solid raised blob (skewed depth) + speckle blocks."""
    H, W = 480, 640
    floor = 2000.0
    baseline = np.full((H, W), floor, dtype=np.float32)
    live = baseline.copy()

    # real blob: solid disc, mostly 1900 (100mm closer) with a skewed tail at 1750
    # so its RAW median (1900) differs from its SMOOTHED (mean-pulled) value.
    canvas = np.zeros((H, W), dtype=np.uint8)
    cv2.circle(canvas, (320, 240), 20, 255, -1)
    disc_mask = canvas > 0
    live[disc_mask] = 1900.0
    # make ~40% of the disc a closer tail (skews mean below the median)
    ys, xs = np.nonzero(disc_mask)
    tail = xs > 320  # right half of the disc
    live[ys[tail], xs[tail]] = 1750.0

    # speckle: scattered 8x8 blocks 40mm closer (survive OPEN+min_area WITHOUT blur)
    rng = np.random.default_rng(0)
    for _ in range(60):
        x = int(rng.integers(20, W - 30))
        y = int(rng.integers(20, H - 30))
        if abs(x - 320) < 40 and abs(y - 240) < 40:
            continue  # keep speckle off the real blob
        live[y:y + 8, x:x + 8] = 1960.0
    return live, baseline, disc_mask


def test_smoothing_collapses_speckle_to_single_blob():
    intr = Intrinsics()
    live, baseline, _ = _floor_with_blob_and_speckle()

    raw = detect_blobs(live, baseline, intr, DetectConfig(smooth_ksize=1))[0]
    smooth = detect_blobs(live, baseline, intr, DetectConfig(smooth_ksize=25))[0]

    assert len(raw) >= 10            # speckle produces many spurious blobs
    assert len(smooth) == 1          # smoothing leaves only the real blob
    X, Y, Z, u, v = smooth[0]
    assert abs(u - 320) < 8 and abs(v - 240) < 8  # centroid ~ the real blob


def test_blob_z_comes_from_raw_not_smoothed_depth():
    intr = Intrinsics()
    live, baseline, disc_mask = _floor_with_blob_and_speckle()

    raw_median = float(np.median(live[disc_mask]))
    smoothed = masked_blur(live, 25)
    smooth_median = float(np.median(smoothed[disc_mask]))
    # precondition: raw vs smoothed genuinely differ over the blob
    assert abs(raw_median - smooth_median) > 20

    meas = detect_blobs(live, baseline, intr, DetectConfig(smooth_ksize=25))[0]
    assert len(meas) == 1
    z = meas[0][2]
    assert abs(z - raw_median) < 12          # z is the RAW blob depth
    assert abs(z - smooth_median) > 12        # z is NOT the blurred depth


def test_sim_video_lit_blob_is_brighter_than_unlit():
    intr = Intrinsics()
    cfg = DetectConfig()
    src = SimSource(intr, cfg, n_spheres=2, seed=4, probe_rgb=(0, 255, 0))
    dark = src.get_video()       # all discs dim
    assert dark.max() <= 70      # nothing blown out
    src.set_lit(0)
    lit = src.get_video()        # one disc blown-out white
    assert lit.max() >= 200      # lit disc much brighter
    assert int(lit.sum()) > int(dark.sum())
