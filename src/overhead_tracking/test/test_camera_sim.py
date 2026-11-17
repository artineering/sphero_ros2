#!/usr/bin/env python3
"""Frame sources. The sim must feed the REAL detector, or node tests prove nothing."""

import os

import numpy as np
import pytest

from overhead_tracking import blobs as B
from overhead_tracking import camera as C

# the node sim tests' arena quad (1280x720), as (u, v) corners
QUAD = [(80.0, 650.0), (1200.0, 660.0), (1210.0, 60.0), (70.0, 50.0)]


def test_sim_produces_a_frame_of_the_requested_size():
    s = C.build_source('sim', width=640, height=480, n=3, seed=1)
    s.start()
    g, ts = s.latest_gray()
    assert g.shape == (480, 640) and g.dtype == np.uint8 and ts > 0


def test_sim_frames_are_detected_by_the_real_detector():
    """The coupling that makes the node sim tests meaningful. The map is built
    from the same frame: the 61 px opening removes the robots from it."""
    pos = [(300, 300), (600, 350), (900, 400), (500, 500), (1100, 200)]
    s = C.build_source('sim', width=1280, height=720, positions=pos, seed=3)
    s.start()
    g, _ = s.latest_gray()
    roi = B.make_roi(QUAD, g.shape)
    blobs = B.detect(g, roi, B.threshold_map([g], roi))
    assert len(blobs) == 5
    for cx, cy in s.positions:
        assert any(abs(b['Centroid'][0] - cx) <= 1 and abs(b['Centroid'][1] - cy) <= 1
                   for b in blobs)


def test_sim_floor_is_uneven_and_realistic():
    """Real arena: ~19 mid, ~8 in the corners, in-arena median ~16."""
    s = C.build_source('sim', width=1280, height=720, positions=[])
    s.start()
    g, _ = s.latest_gray()
    roi = B.make_roi(QUAD, g.shape, margin=0)
    assert 13 <= np.median(g[roi['sl']][roi['mask'] > 0]) <= 19
    centre = g[340:380, 620:660].mean()
    corner = g[:40, :40].mean()
    assert centre - corner > 8


def test_fresh_array_every_tick():
    """The immutability invariant that lets workers take zero-copy views."""
    s = C.build_source('sim', width=320, height=240, n=1)
    s.start()
    a, _ = s.latest_gray()
    s.on_tick()
    b, _ = s.latest_gray()
    assert a is not b


def test_scripted_positions_and_motion():
    s = C.build_source('sim', width=640, height=480,
                       positions=[(100.0, 100.0), (300.0, 200.0)])
    s.set_velocity(0, 60.0, 0.0)
    s.step(1.0)
    assert s.positions[0][0] == pytest.approx(160.0)
    assert s.positions[1][0] == pytest.approx(300.0)


def test_set_lit_brightens_one_robot():
    """The hook the serial-probe linking tests need."""
    s = C.build_source('sim', width=640, height=480,
                       positions=[(150.0, 150.0), (400.0, 300.0)], noise=0)
    s.on_tick()
    dark, _ = s.latest_gray()
    s.set_lit(0)
    s.on_tick()
    lit, _ = s.latest_gray()
    win = (slice(120, 180), slice(120, 180))
    assert lit[win].mean() > dark[win].mean()
    other = (slice(270, 330), slice(370, 430))
    assert lit[other].mean() == pytest.approx(dark[other].mean(), abs=1.0)
    s.clear_lit()
    s.on_tick()
    assert s.latest_gray()[0][win].mean() == pytest.approx(dark[win].mean(), abs=1.0)


def test_unknown_source_kind_is_a_clean_error():
    with pytest.raises(SystemExit):
        C.build_source('lidar')


@pytest.mark.skipif(not os.path.exists('/dev/video0'),
                    reason='no /dev/video0 on this machine')
def test_v4l2_source_constructs_when_the_device_exists():
    s = C.build_source('v4l2', device='/dev/video0', width=640, height=480)
    assert hasattr(s, 'start') and hasattr(s, 'latest_gray')
    assert hasattr(s, 'apply_controls') and hasattr(s, 'close')


def test_v4l2_source_object_builds_without_opening_the_device():
    """Construction must not touch hardware -- only start() does."""
    s = C.V4L2Source(device='/dev/does-not-exist')
    assert s.latest_gray() == (None, 0.0)
    assert s.stats()['device'] == '/dev/does-not-exist'
