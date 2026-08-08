#!/usr/bin/env python3
"""Roster selection, status merging, and serial LED-probe blob matching."""

import cv2
import numpy as np

from overhead_tracking import identify as I


def dark(h=200, w=300):
    return np.zeros((h, w), np.uint8)


def lit_at(base, centres, radius=8, value=200):
    img = base.copy()
    for u, v in centres:
        cv2.circle(img, (int(u), int(v)), radius, value, -1)
    return img


# --------------------------------------------------------------- roster logic
def test_select_targets_prefers_explicit_request():
    last_seen = {'SB-AAAA': 100.0, 'SB-BBBB': 100.0}
    assert I.select_targets(['SB-BBBB'], last_seen, 101.0, 15.0) == ['SB-BBBB']


def test_select_targets_falls_back_to_fresh_heartbeats():
    last_seen = {'SB-AAAA': 100.0, 'SB-STALE': 10.0, 'SB-NONE': None}
    assert I.select_targets([], last_seen, 105.0, 15.0) == ['SB-AAAA']


def test_merge_preserves_untargeted_callsigns():
    """Re-probing one robot must not silently mark everyone else failed."""
    reg, fail = I.merge_link_status(
        prev_registered=['SB-AAAA', 'SB-BBBB'], prev_failed=['SB-CCCC'],
        targets=['SB-BBBB'], new_registered=[], new_failed=['SB-BBBB'])
    assert reg == ['SB-AAAA']
    assert fail == ['SB-BBBB', 'SB-CCCC']


def test_merge_clears_a_name_from_failed_once_it_links():
    reg, fail = I.merge_link_status(['SB-AAAA'], ['SB-BBBB'], ['SB-BBBB'],
                                    ['SB-BBBB'], [])
    assert reg == ['SB-AAAA', 'SB-BBBB'] and fail == []


def test_status_payload_shape():
    p = I.link_status_payload(['SB-B', 'SB-A'], ['SB-C'], {'SB-A': 0})
    assert p == {'complete': True, 'registered': ['SB-A', 'SB-B'],
                 'failed': ['SB-C'], 'links': {'SB-A': 0}}


# ------------------------------------------------------------- lit detection
def test_lit_blob_found_against_dark_pre_frame():
    pre = dark()
    post = lit_at(pre, [(150, 100)])
    blobs, _ = I.lit_blobs_gray(post, pre, brightness_delta=40, min_area=20)
    assert len(blobs) == 1
    u, v = blobs[0]['centroid']
    assert abs(u - 150) < 3 and abs(v - 100) < 3


def test_identical_frames_yield_no_lit_blob():
    pre = lit_at(dark(), [(150, 100)])
    blobs, _ = I.lit_blobs_gray(pre, pre, brightness_delta=40)
    assert blobs == []


def test_already_lit_robot_is_cancelled_by_the_diff():
    """A previously-linked robot showing its status colour must not confuse the
    match -- the pre/post diff is ~0 wherever both frames are bright."""
    pre = lit_at(dark(), [(50, 50)])                 # already on
    post = lit_at(pre, [(220, 150)])                 # newly lit
    blobs, _ = I.lit_blobs_gray(post, pre, brightness_delta=40)
    assert len(blobs) == 1
    u, v = blobs[0]['centroid']
    assert abs(u - 220) < 3 and abs(v - 150) < 3


# ------------------------------------------------------------------ matching
def test_match_picks_the_nearest_blob():
    centres = [(50, 50), (150, 100), (250, 160)]
    lit = [{'centroid': (152.0, 101.0), 'bbox': (0, 0, 1, 1), 'area': 50}]
    r = I.match_lit_blob(centres, lit, tol_px=25.0)
    assert r['matched'] and r['blob_index'] == 1


def test_match_reports_too_far_rather_than_guessing():
    centres = [(50, 50)]
    lit = [{'centroid': (250.0, 160.0), 'bbox': (0, 0, 1, 1), 'area': 50}]
    r = I.match_lit_blob(centres, lit, tol_px=25.0)
    assert not r['matched'] and r['reason'] == 'too_far'


def test_exclusion_keeps_the_assignment_bijective():
    """Two callsigns must never bind to the same blob."""
    centres = [(50, 50), (150, 100)]
    lit = [{'centroid': (51.0, 51.0), 'bbox': (0, 0, 1, 1), 'area': 50}]
    first = I.match_lit_blob(centres, lit, 25.0)
    assert first['blob_index'] == 0
    second = I.match_lit_blob(centres, lit, 25.0, exclude_indices={0})
    assert second['blob_index'] != 0


def test_empty_inputs_report_a_reason():
    assert I.match_lit_blob([(1, 1)], [], 25.0)['reason'] == 'no_lit'
    lit = [{'centroid': (1.0, 1.0), 'bbox': (0, 0, 1, 1), 'area': 50}]
    assert I.match_lit_blob([], lit, 25.0)['reason'] == 'no_blobs'


def test_full_three_robot_serial_probe_is_a_bijection():
    centres = [(60, 60), (160, 90), (250, 150)]
    pre = dark()
    bound, used = {}, set()
    for name, idx in [('SB-A', 2), ('SB-B', 0), ('SB-C', 1)]:
        post = lit_at(pre, [centres[idx]])
        lit, _ = I.lit_blobs_gray(post, pre, 40, 20)
        r = I.match_lit_blob(centres, lit, 25.0, exclude_indices=used)
        assert r['matched']
        bound[name] = r['blob_index']
        used.add(r['blob_index'])
    assert bound == {'SB-A': 2, 'SB-B': 0, 'SB-C': 1}
    assert len(set(bound.values())) == 3
