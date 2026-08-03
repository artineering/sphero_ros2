"""Offline unit tests for the fleet-policy Proximity cue step handler.

Exercises execute_proximity_step's math with synthetic group snapshots (no
hardware, no ROS): anchor (FIELD_POINT / CENTROID), no-fix exclusion, min-sep
repulsion direction, field-bounds clamp, and the deadband stop.
"""
import math

from sphero_instance_controller.core.common.task import TaskDescriptor
from sphero_instance_controller.core.sphero.sphero_task_executor import (
    SpheroTaskExecutorBase,
)
from sphero_instance_controller.core.sphero import sphero_task_handlers as h


class ProxExec(SpheroTaskExecutorBase):
    """Records roll/stop; feeds a fixed own position + member snapshot."""

    def __init__(self, own, members):
        self.sends = []
        self._own = own
        self._members = members
        super().__init__(
            position_callback=lambda: self._own,
            member_positions_callback=lambda: self._members,
        )

    def _send_roll_command(self, heading, speed, duration=0):
        self.sends.append(('roll', heading, speed))

    def _send_stop_command(self):
        self.sends.append(('stop',))


def _step(own, snapshot, **params):
    ex = ProxExec(own, snapshot)
    task = TaskDescriptor(task_id='p', task_type='proximity_step', parameters=params)
    done = h.execute_proximity_step(ex, task)
    return ex, done


def test_field_point_attraction_heading_and_speed_cap():
    # Own at origin, anchor 100cm east, no neighbors -> roll due east, capped.
    ex, done = _step(
        {'x': 0.0, 'y': 0.0}, {},
        me='SB_ME', members=[],
        anchor=h.PROXIMITY_ANCHOR_FIELD_POINT, anchor_x=100.0, anchor_y=0.0,
        max_speed=60, target_spacing_cm=30.0, min_separation_cm=15.0,
    )
    assert done is True
    assert ex.sends == [('roll', 0, 60)]  # heading 0, speed min(60,100)=60


def test_centroid_is_mean_of_fixes():
    # Two members with fixes -> centroid (50,50); own at origin -> heading 45.
    members = {'SB_A': {'x': 100.0, 'y': 0.0}, 'SB_B': {'x': 0.0, 'y': 100.0}}
    ex, done = _step(
        {'x': 0.0, 'y': 0.0}, members,
        me='SB_ME', members=['SB_A', 'SB_B'],
        anchor=h.PROXIMITY_ANCHOR_CENTROID,
        max_speed=60, target_spacing_cm=30.0, min_separation_cm=15.0,
    )
    assert done is True
    kind, heading, speed = ex.sends[0]
    assert kind == 'roll'
    assert heading == 45           # atan2(50,50)
    assert speed == 60             # min(60, hypot(50,50)=70)


def test_no_fix_member_excluded_from_centroid():
    # Only SB_A has a fix; centroid == SB_A's position (SB_B ignored).
    members = {'SB_A': {'x': 60.0, 'y': 0.0}}  # SB_B absent from snapshot
    ex, done = _step(
        {'x': 0.0, 'y': 0.0}, members,
        me='SB_ME', members=['SB_A', 'SB_B'],
        anchor=h.PROXIMITY_ANCHOR_CENTROID,
        max_speed=100, target_spacing_cm=30.0, min_separation_cm=15.0,
    )
    kind, heading, speed = ex.sends[0]
    assert kind == 'roll'
    assert heading == 0            # straight toward SB_A at (60,0)
    assert speed == 60             # hypot(60,0)=60 <= max_speed 100


def test_centroid_no_fixes_idles():
    # CENTROID with an empty snapshot -> no anchor -> idle (stop).
    ex, done = _step(
        {'x': 0.0, 'y': 0.0}, {},
        me='SB_ME', members=['SB_A', 'SB_B'],
        anchor=h.PROXIMITY_ANCHOR_CENTROID,
        max_speed=60, target_spacing_cm=30.0, min_separation_cm=15.0,
    )
    assert done is True
    assert ex.sends == [('stop',)]


def test_min_sep_repulsion_pushes_away():
    # Anchor at own position (zero attraction); one neighbor 10cm east, inside
    # min-sep (15) and target-spacing (30) -> unit rolls due WEST (heading 180).
    members = {'SB_N': {'x': 10.0, 'y': 0.0}}
    ex, done = _step(
        {'x': 0.0, 'y': 0.0}, members,
        me='SB_ME', members=['SB_ME', 'SB_N'],
        anchor=h.PROXIMITY_ANCHOR_FIELD_POINT, anchor_x=0.0, anchor_y=0.0,
        max_speed=60, target_spacing_cm=30.0, min_separation_cm=15.0,
    )
    assert done is True
    kind, heading, speed = ex.sends[0]
    assert kind == 'roll'
    assert heading == 180          # away from the neighbor
    # push = (30-10)/10 = 2.0 -> |v| = 10*2.0 = 20
    assert speed == 20


def test_settle_stops_within_deadband():
    # At the anchor, no close neighbor -> net force ~0 -> stop and hold.
    ex, done = _step(
        {'x': 0.0, 'y': 0.0}, {},
        me='SB_ME', members=[],
        anchor=h.PROXIMITY_ANCHOR_FIELD_POINT, anchor_x=0.0, anchor_y=0.0,
        max_speed=60, target_spacing_cm=30.0, min_separation_cm=15.0,
    )
    assert done is True
    assert ex.sends == [('stop',)]


def test_field_bounds_clamp_applied_to_anchor():
    # Anchor 1000cm east but field bound clamps x to 100; own at x=50 ->
    # remaining vector is only 50cm, so speed reflects the clamped target.
    ex, done = _step(
        {'x': 50.0, 'y': 0.0}, {},
        me='SB_ME', members=[],
        anchor=h.PROXIMITY_ANCHOR_FIELD_POINT, anchor_x=1000.0, anchor_y=0.0,
        bounds=[-100.0, -100.0, 100.0, 100.0],
        max_speed=60, target_spacing_cm=30.0, min_separation_cm=15.0,
    )
    kind, heading, speed = ex.sends[0]
    assert kind == 'roll'
    assert heading == 0
    assert speed == 50             # min(60, 100-50) -> clamp made target 100
