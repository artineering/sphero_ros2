#!/usr/bin/env python3
"""nudge.py: solo-window detection, moved-blob pick, free-space heading."""

from overhead_tracking import nudge as N

POLY = [(0, 0), (200, 0), (200, 100), (0, 100)]


def run(watch, events):
    """events: ('cmd', t, name, speed) | ('frame', stamp). Returns the actions
    taken as (stamp, action, name); 'before' / 'after' handled like the node."""
    out = []
    for ev in events:
        if ev[0] == 'cmd':
            watch.command(*ev[1:])
            continue
        act = watch.frame(ev[1])
        if act == 'before':
            out.append((ev[1], act, watch.name))
            watch.set_before(['snap'])
        elif act == 'after':
            name, before = watch.take()
            assert before == ['snap']
            out.append((ev[1], act, name))
    return out


def frames(t0, t1, dt=0.1):
    return [('frame', round(t0 + k * dt, 3)) for k in range(int(round((t1 - t0) / dt)) + 1)]


def test_solo_window_before_then_after_settle():
    ev = frames(0, 0.9) + [('cmd', 1.0, 'A', 50)] + frames(1.0, 1.4) + \
        [('cmd', 1.5, 'A', 0)] + frames(1.5, 3.5)
    acts = run(N.NudgeWatch(settle=1.5), ev)
    assert acts == [(1.0, 'before', 'A'), (3.0, 'after', 'A')]


def test_other_robot_moving_spoils_the_window():
    ev = [('cmd', 1.0, 'A', 50)] + frames(1.0, 1.2) + [('cmd', 1.3, 'B', 50),
                                                       ('cmd', 1.5, 'A', 0),
                                                       ('cmd', 1.6, 'B', 0)] + frames(1.3, 5.0)
    assert run(N.NudgeWatch(settle=1.0), ev) == [(1.0, 'before', 'A')]


def test_no_window_while_another_robot_is_moving():
    ev = [('cmd', 0.0, 'B', 60), ('cmd', 1.0, 'A', 50), ('cmd', 1.5, 'A', 0)] + frames(0, 5)
    assert run(N.NudgeWatch(settle=1.0), ev) == []


def test_recorded_sequence_gives_two_windows_and_none_for_the_group_drive():
    # the kf_20261005_171405 schedule: two solo nudges, then everyone drives
    ev = [('cmd', 4.59, 'SB-3660', 50)] + frames(4.6, 5.0) + [('cmd', 5.11, 'SB-3660', 0)] + \
        frames(5.2, 7.5) + [('cmd', 7.56, 'SB-418F', 50)] + frames(7.6, 8.3) + \
        [('cmd', 8.42, 'SB-418F', 0)] + frames(8.5, 10.6) + \
        [('cmd', 10.74, 'SB-3660', 70), ('cmd', 10.95, 'SB-418F', 70)] + frames(10.8, 13)
    acts = run(N.NudgeWatch(settle=1.5), ev)
    assert [(a, n) for _s, a, n in acts] == [('before', 'SB-3660'), ('after', 'SB-3660'),
                                            ('before', 'SB-418F'), ('after', 'SB-418F')]
    assert acts[1][0] >= 5.11 + 1.5 and acts[3][0] >= 8.42 + 1.5


def test_moved_blob_picks_the_one_that_left_and_where_it_went():
    before = [(100, 100), (300, 100), (500, 500)]
    after = [(101, 99), (360, 140), (500, 501)]
    old, new, reason = N.moved_blob(before, after, 5.0)
    assert reason == '' and old == (300, 100) and new == (360, 140)


def test_moved_blob_refuses_ambiguity():
    assert N.moved_blob([(0, 0), (100, 0)], [(50, 50), (150, 50)], 5.0)[2] == 'moved=2'
    assert N.moved_blob([(0, 0), (100, 0)], [(0, 0), (100, 0)], 5.0)[2] == 'moved=0'
    # moved, but merged into a neighbour: no new blob appeared
    assert N.moved_blob([(0, 0), (100, 0)], [(100, 0)], 5.0)[2] == 'new=0'
    assert N.moved_blob([], [(1, 1)], 5.0)[2] == 'moved=0'


def test_free_heading_away_from_the_wall_and_the_neighbour():
    # hard against the west wall: go east
    deg, room = N.free_heading([(5, 50)], [(5, 50)], POLY, 15)
    assert deg == 0.0 and room > 0
    # north-east corner: go south-west-ish
    deg, _ = N.free_heading([(195, 95)], [(195, 95)], POLY, 15)
    assert 180 <= deg <= 270
    # a neighbour 20 cm east in open field: not east
    deg, _ = N.free_heading([(100, 50)], [(100, 50), (120, 50)], POLY, 15)
    assert deg != 0.0
    # either of two candidates may be the robot: both must have room
    deg, room = N.free_heading([(5, 50), (195, 50)], [(5, 50), (195, 50)], POLY, 15)
    assert deg in (90.0, 270.0) and room > 0
