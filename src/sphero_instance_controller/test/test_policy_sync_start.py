"""Offline tests for policy-level synchronized start (FleetPolicy now/start_offset).

Covers the three mechanisms the plan introduces:
  1. Epoch resolution in the task controller's policy_callback: msg `now=0.0`
     normalizes to immediate, and a nonzero start_offset below the ~0.5 s floor
     is bumped to the floor (OQ4). Direct-path `_compute_start_at` unchanged.
  2. Executor start_at gate (mechanism b): a future-dated one-shot task is
     withheld while a task in a DISJOINT lane proceeds (no cross-lane block).
  3. SM controller pending buffer (mechanism a): two lane-tagged configs sharing
     one future start_at both withhold until the epoch and both activate on the
     first tick at/after it; a no-start_at config activates immediately; a clear
     on a pending lane cancels it without ever firing.

Run against the BUILT package (msg module resolves from install), e.g.:
    python3 -m pytest --noconftest src/.../test/test_policy_sync_start.py
"""
import json

import pytest
import rclpy
from std_msgs.msg import String

from multirobot_msgs.msg import FleetPolicy, SimilarityCue, ProximityCue

from sphero_instance_controller.core.common.task import (
    TaskDescriptor,
    TaskStatus,
    TaskExecutorBase,
)
from sphero_instance_controller.sphero_instance_task_controller_node import (
    SpheroInstanceTaskController,
    MIN_POLICY_START_OFFSET_S,
)
from sphero_instance_controller.sphero_instance_statemachine_controller_node import (
    SpheroInstanceStateMachineController,
)


# ============================================================ helpers / fixtures


class FakeClock:
    """Deterministic stand-in for module-level ``time.time``."""

    def __init__(self, t=1000.0):
        self.now = t

    def time(self):
        return self.now

    def advance(self, dt):
        self.now += dt


class Recorder:
    """Stand-in publisher that records published JSON payloads."""

    def __init__(self):
        self.msgs = []

    def publish(self, msg):
        self.msgs.append(json.loads(msg.data))


def _fleet_policy(member, *, now=0.0, start_offset=0.0,
                  similarity=True, proximity=False):
    msg = FleetPolicy()
    msg.policy_id = 'p1'
    msg.active = True
    msg.members = [member]
    msg.enable_similarity = similarity
    msg.enable_proximity = proximity
    msg.enable_common_fate = False
    msg.now = float(now)
    msg.start_offset = float(start_offset)
    # IDENTITY steady cue (content irrelevant; _apply_similarity is spied).
    msg.similarity = SimilarityCue()
    msg.similarity.cue_type = SimilarityCue.IDENTITY
    msg.proximity = ProximityCue()
    return msg


@pytest.fixture
def task_node():
    rclpy.init()
    n = SpheroInstanceTaskController('SB-33E3')
    yield n
    n.destroy_node()
    rclpy.shutdown()


@pytest.fixture
def sm_node():
    rclpy.init()
    n = SpheroInstanceStateMachineController('SB-33E3')
    n.task_pub = Recorder()
    yield n
    n.destroy_node()
    rclpy.shutdown()


def _cfg_msg(config):
    m = String()
    m.data = json.dumps(config)
    return m


def _ctl_msg(**payload):
    m = String()
    m.data = json.dumps(payload)
    return m


def _timed_config(lane, name, start_at):
    """A minimal two-state ping-pong config carrying a synchronized-start epoch."""
    step = {'task_id': 't', 'task_type': 'set_led', 'parameters': {}}
    return {
        'name': name,
        'lane': lane,
        'initial_state': 'a',
        'start_at': start_at,
        'states': [
            {'name': 'a', 'tasks': [step],
             'exits': [{'condition': {'type': 'timer', 'duration': 0.5}, 'destination': 'b'}]},
            {'name': 'b', 'tasks': [step],
             'exits': [{'condition': {'type': 'timer', 'duration': 0.5}, 'destination': 'a'}]},
        ],
    }


# ==================================================== 1. epoch resolution (Step 2)


class TestEpochResolution:
    """policy_callback normalizes msg timing and applies the ~0.5 s floor."""

    def _captured_start_at(self, node, monkeypatch, **policy_kwargs):
        """Feed a FleetPolicy and return the start_at threaded to _apply_similarity."""
        captured = {}
        monkeypatch.setattr(
            node, '_apply_similarity',
            lambda cue, start_at=None: captured.__setitem__('start_at', start_at))
        node.policy_callback(_fleet_policy('SB-33E3', **policy_kwargs))
        return captured.get('start_at')

    def test_compute_start_at_direct_path_unchanged(self, task_node):
        # The shared resolver keeps its exact contract (no floor lives here).
        import sphero_instance_controller.sphero_instance_task_controller_node as m
        t_now = m.time.time()
        assert task_node._compute_start_at(None, None) is None
        assert task_node._compute_start_at(0.0, 0.0) is None
        assert task_node._compute_start_at(t_now + 100.0, 3.0) == pytest.approx(t_now + 103.0)
        # A tiny offset is NOT floored on the direct path.
        assert task_node._compute_start_at(t_now + 100.0, 0.2) == pytest.approx(t_now + 100.2)
        # Past target -> immediate.
        assert task_node._compute_start_at(t_now - 100.0, 1.0) is None

    def test_msg_now_zero_is_immediate(self, task_node, monkeypatch):
        # now defaults to 0.0 on the wire -> immediate (start_at None).
        assert self._captured_start_at(task_node, monkeypatch,
                                       now=0.0, start_offset=3.0) is None

    def test_msg_start_offset_zero_is_immediate(self, task_node, monkeypatch):
        import sphero_instance_controller.sphero_instance_task_controller_node as m
        assert self._captured_start_at(task_node, monkeypatch,
                                       now=m.time.time(), start_offset=0.0) is None

    def test_nonzero_offset_resolves_to_epoch(self, task_node, monkeypatch):
        import sphero_instance_controller.sphero_instance_task_controller_node as m
        t_now = m.time.time()
        start_at = self._captured_start_at(task_node, monkeypatch,
                                           now=t_now, start_offset=3.0)
        assert start_at == pytest.approx(t_now + 3.0, abs=0.1)

    def test_small_offset_floored(self, task_node, monkeypatch):
        # 0 < start_offset < 0.5 -> bumped to the floor, anchored to `now`.
        import sphero_instance_controller.sphero_instance_task_controller_node as m
        t_now = m.time.time()
        start_at = self._captured_start_at(task_node, monkeypatch,
                                           now=t_now, start_offset=0.2)
        assert start_at == pytest.approx(t_now + MIN_POLICY_START_OFFSET_S, abs=0.1)

    def test_offset_at_or_above_floor_not_bumped(self, task_node, monkeypatch):
        import sphero_instance_controller.sphero_instance_task_controller_node as m
        t_now = m.time.time()
        start_at = self._captured_start_at(task_node, monkeypatch,
                                           now=t_now, start_offset=0.5)
        assert start_at == pytest.approx(t_now + 0.5, abs=0.1)


# ================================================ 2. executor start_at gate (Step 3)


class _LaneExecutor(TaskExecutorBase):
    """Base executor with a trivial one-shot 'go' handler for lane-gating tests."""

    def __init__(self):
        self.fired = []
        super().__init__()

    def _register_default_handlers(self):
        self.register_handler(
            'go', lambda _e, t: (self.fired.append(t.task_id) or True))


class TestExecutorStartAtGate:

    def _clock(self, monkeypatch, t=1000.0):
        fake = FakeClock(t)
        import sphero_instance_controller.core.common.task as task_mod
        monkeypatch.setattr(task_mod.time, 'time', fake.time)
        return fake

    def test_future_task_withheld_while_disjoint_lane_proceeds(self, monkeypatch):
        clock = self._clock(monkeypatch)
        ex = _LaneExecutor()

        gated = TaskDescriptor(task_id='led_gated', task_type='go', parameters={},
                               lanes=frozenset({'led'}))
        gated.start_at = clock.now + 5.0
        ready = TaskDescriptor(task_id='drive_now', task_type='go', parameters={},
                               lanes=frozenset({'drive'}))
        ex.add_task(gated)
        ex.add_task(ready)

        ex.process_tasks()
        # Disjoint DRIVE task ran; the future-dated LED task stayed pending.
        assert 'drive_now' in ex.fired
        assert 'led_gated' not in ex.fired
        assert gated in ex.task_queue
        assert gated.status == TaskStatus.PENDING

        # Still withheld before the epoch.
        clock.advance(4.9)
        ex.process_tasks()
        assert 'led_gated' not in ex.fired

        # Promotes once the shared instant arrives.
        clock.advance(0.1)
        ex.process_tasks()
        assert 'led_gated' in ex.fired
        assert gated not in ex.task_queue


# ============================================ 3. SM controller pending buffer (Step 5)


class TestSmPendingBuffer:

    def _clock(self, monkeypatch, t=1000.0):
        fake = FakeClock(t)
        import sphero_instance_controller.sphero_instance_statemachine_controller_node as m
        monkeypatch.setattr(m.time, 'time', fake.time)
        return fake

    @staticmethod
    def _configured(node, lane):
        return node.state_machines[lane].get_status()['configured']

    def test_two_lanes_share_epoch_withhold_then_activate_together(self, sm_node, monkeypatch):
        clock = self._clock(monkeypatch)
        epoch = clock.now + 3.0

        sm_node.config_callback(_cfg_msg(_timed_config('led', 'blink', epoch)))
        sm_node.config_callback(_cfg_msg(_timed_config('drive', 'prox', epoch)))

        # Both withheld: pending, not configured.
        assert set(sm_node._pending_configs) == {'led', 'drive'}
        assert self._configured(sm_node, 'led') is False
        assert self._configured(sm_node, 'drive') is False

        # A tick before the epoch does NOT activate either lane.
        clock.advance(2.9)
        sm_node.update_callback()
        assert self._configured(sm_node, 'led') is False
        assert self._configured(sm_node, 'drive') is False
        assert set(sm_node._pending_configs) == {'led', 'drive'}

        # First tick at/after the epoch flushes BOTH lanes together.
        clock.advance(0.1)
        sm_node.update_callback()
        assert self._configured(sm_node, 'led') is True
        assert self._configured(sm_node, 'drive') is True
        assert sm_node._pending_configs == {}

    def test_no_start_at_activates_immediately(self, sm_node, monkeypatch):
        self._clock(monkeypatch)
        cfg = _timed_config('led', 'blink', 0.0)
        del cfg['start_at']  # omit entirely -> immediate (webapp / websocket path)
        sm_node.config_callback(_cfg_msg(cfg))
        assert self._configured(sm_node, 'led') is True
        assert 'led' not in sm_node._pending_configs

    def test_past_start_at_activates_immediately(self, sm_node, monkeypatch):
        clock = self._clock(monkeypatch)
        # start_at already in the past -> configure now (back-compat guard).
        sm_node.config_callback(_cfg_msg(_timed_config('led', 'blink', clock.now - 5.0)))
        assert self._configured(sm_node, 'led') is True
        assert 'led' not in sm_node._pending_configs

    def test_clear_on_pending_lane_cancels_without_firing(self, sm_node, monkeypatch):
        clock = self._clock(monkeypatch)
        epoch = clock.now + 3.0
        sm_node.config_callback(_cfg_msg(_timed_config('drive', 'prox', epoch)))
        assert 'drive' in sm_node._pending_configs

        # Revoke mid-wait drops the pending entry.
        sm_node.control_callback(_ctl_msg(action='clear', lane='drive'))
        assert 'drive' not in sm_node._pending_configs

        # Past the epoch it must NEVER activate.
        clock.advance(10.0)
        sm_node.update_callback()
        assert self._configured(sm_node, 'drive') is False

    def test_fresh_config_replaces_pending_on_same_lane(self, sm_node, monkeypatch):
        clock = self._clock(monkeypatch)
        sm_node.config_callback(_cfg_msg(_timed_config('led', 'first', clock.now + 3.0)))
        # A second timed config for the same lane supersedes the first pending one.
        sm_node.config_callback(_cfg_msg(_timed_config('led', 'second', clock.now + 5.0)))
        assert sm_node._pending_configs['led'][0]['name'] == 'second'
        assert sm_node._pending_configs['led'][1] == clock.now + 5.0
