"""Unit tests for pure registration helpers."""

from kinect_field_tracking.registration import (
    fresh_deployed,
    select_targets,
    merge_registration_status,
    registration_status_payload,
)


def test_fresh_deployed_filters_stale_heartbeats():
    now = 1000.0
    last_seen = {'A': now - 5, 'B': now - 20, 'C': now - 1, 'D': None}
    assert fresh_deployed(last_seen, now, 15.0) == ['A', 'C']


def test_select_targets_explicit_wins():
    now = 1000.0
    last_seen = {'A': now - 1, 'B': now - 1}
    assert select_targets(['B'], last_seen, now, 15.0) == ['B']


def test_select_targets_falls_back_to_deployed():
    now = 1000.0
    last_seen = {'A': now - 1, 'B': now - 100}
    assert select_targets([], last_seen, now, 15.0) == ['A']


def test_merge_preserves_untouched_callsigns():
    # re-register only C; A,B keep their prior status
    reg, fail = merge_registration_status(
        prev_registered={'A', 'B'}, prev_failed={'C'},
        targets={'C'}, new_registered={'C'}, new_failed=set())
    assert reg == ['A', 'B', 'C']
    assert fail == []


def test_merge_moves_target_to_failed():
    reg, fail = merge_registration_status(
        prev_registered={'A', 'B'}, prev_failed=set(),
        targets={'B'}, new_registered=set(), new_failed={'B'})
    assert reg == ['A']
    assert fail == ['B']


def test_merge_no_name_in_both():
    reg, fail = merge_registration_status(
        prev_registered=set(), prev_failed={'A'},
        targets={'A'}, new_registered={'A'}, new_failed=set())
    assert reg == ['A']
    assert fail == []


def test_status_payload_shape():
    p = registration_status_payload({'B', 'A'}, {'C'})
    assert p == {'complete': True, 'registered': ['A', 'B'], 'failed': ['C']}
