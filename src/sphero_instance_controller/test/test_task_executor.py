"""Unit tests for the robot-agnostic TaskExecutorBase + Sphero handlers.

Covers:
  A. Generic queue / lifecycle (TaskExecutorBase)
  B. Registry semantics
  C. Cancel-on-stop hook
  D. Sphero handler regression smoke (all 16 handlers)
"""
import math
import pytest

from sphero_instance_controller.core.common.task import (
    TaskDescriptor,
    TaskStatus,
    TaskExecutorBase,
)
from sphero_instance_controller.core.sphero import sphero_task_handlers as h
from sphero_instance_controller.core.sphero.sphero_task_executor import (
    SpheroTaskExecutorBase,
)


# --------------------------------------------------------------------- fixtures


class FakeTime:
    """Stand-in for the ``time`` module inside the test target."""

    def __init__(self, t: float = 1000.0):
        self.now = t
        self.sleep_calls = []

    def time(self) -> float:
        return self.now

    def advance(self, dt: float) -> None:
        self.now += dt

    def sleep(self, dt: float) -> None:
        # Don't actually sleep in tests; just advance our clock.
        self.sleep_calls.append(dt)
        self.now += dt


@pytest.fixture
def clock(monkeypatch):
    """Monkeypatch `time.time` everywhere our code reads it."""
    fake = FakeTime()
    import sphero_instance_controller.core.common.task as task_mod
    import sphero_instance_controller.core.sphero.sphero_task_handlers as handlers_mod

    monkeypatch.setattr(task_mod.time, 'time', fake.time)
    monkeypatch.setattr(handlers_mod.time, 'time', fake.time)
    monkeypatch.setattr(handlers_mod.time, 'sleep', fake.sleep)
    return fake


def make_task(task_type: str, **params) -> TaskDescriptor:
    return TaskDescriptor(
        task_id=f't_{task_type}',
        task_type=task_type,
        parameters=dict(params),
    )


class RecordingExecutor(TaskExecutorBase):
    """Generic executor with hand-registered trivial handlers.

    Records every handler invocation as ('<task_type>', task) in self.calls.
    """

    def __init__(self):
        self.calls = []
        # Sentinel for what the next handler call should return; a list lets
        # tests script multi-tick behaviors.
        self._return_queue = []
        super().__init__()

    def _register_default_handlers(self):
        self.register_handler('one_shot', self._one_shot)
        self.register_handler('two_shot', self._two_shot)
        self.register_handler('boom', self._boom)

    def _one_shot(self, _executor, task):
        self.calls.append(('one_shot', task))
        return True

    def _two_shot(self, _executor, task):
        self.calls.append(('two_shot', task))
        if not task.parameters.get('_kicked'):
            task.parameters['_kicked'] = True
            return False
        return True

    def _boom(self, _executor, task):
        self.calls.append(('boom', task))
        raise RuntimeError('handler exploded')


class RecordingSphero(SpheroTaskExecutorBase):
    """SpheroTaskExecutorBase concrete impl that records every _send_* call."""

    def __init__(self, position=None, heading=0):
        self.sends = []
        self._fake_position = position or {'x': 0.0, 'y': 0.0}
        self._fake_heading = heading
        super().__init__(
            position_callback=lambda: self._fake_position,
            heading_callback=lambda: self._fake_heading,
        )

    def _record(self, name, **kwargs):
        self.sends.append((name, kwargs))

    def _send_raw_motor_command(self, left_mode, left_speed, right_mode, right_speed):
        self._record('raw_motor', left_mode=left_mode, left_speed=left_speed,
                     right_mode=right_mode, right_speed=right_speed)

    def _send_roll_command(self, heading, speed, duration=0):
        self._record('roll', heading=heading, speed=speed, duration=duration)

    def _send_stop_command(self):
        self._record('stop')

    def _send_led_command(self, red, green, blue, led_type='main'):
        self._record('led', red=red, green=green, blue=blue, led_type=led_type)

    def _send_heading_command(self, heading):
        self._record('heading', heading=heading)

    def _send_speed_command(self, speed):
        self._record('speed', speed=speed)

    def _send_spin_command(self, angle, duration=1.0):
        self._record('spin', angle=angle, duration=duration)

    def _send_matrix_command(self, pattern=None, red=255, green=255, blue=255):
        self._record('matrix', pattern=pattern, red=red, green=green, blue=blue)

    def _send_stabilization_command(self, enable):
        self._record('stabilization', enable=enable)

    def _send_collision_detection_command(self, action, mode='obstacle', sensitivity='HIGH'):
        self._record('collision', action=action, mode=mode, sensitivity=sensitivity)


# ====================================================================
# A. Generic queue / lifecycle
# ====================================================================


class TestQueueLifecycle:

    def test_add_task_appends_to_queue(self, clock):
        ex = RecordingExecutor()
        ex.add_task(make_task('one_shot'))
        ex.add_task(make_task('two_shot'))
        assert [t.task_type for t in ex.task_queue] == ['one_shot', 'two_shot']
        assert ex.current_task is None

    def test_process_tasks_picks_up_pending(self, clock):
        ex = RecordingExecutor()
        ex.add_task(make_task('two_shot'))
        ex.process_tasks()
        # two_shot returns False on first tick → still running
        assert ex.current_task is not None
        assert ex.current_task.status == TaskStatus.RUNNING
        assert ex.current_task.started_at == clock.now

    def test_process_tasks_marks_completed_when_handler_returns_true(self, clock):
        ex = RecordingExecutor()
        ex.add_task(make_task('one_shot'))
        ex.process_tasks()
        assert ex.current_task is None
        assert len(ex.task_history) == 1
        done = ex.task_history[0]
        assert done.status == TaskStatus.COMPLETED
        assert done.completed_at == clock.now

    def test_process_tasks_keeps_running_when_handler_returns_false(self, clock):
        ex = RecordingExecutor()
        ex.add_task(make_task('two_shot'))
        ex.process_tasks()
        assert ex.current_task is not None
        # Second tick completes it.
        ex.process_tasks()
        assert ex.current_task is None
        assert len(ex.task_history) == 1
        assert ex.task_history[0].status == TaskStatus.COMPLETED

    def test_handler_exception_marks_task_failed(self, clock):
        ex = RecordingExecutor()
        ex.add_task(make_task('boom'))
        ex.process_tasks()
        assert ex.current_task is None
        assert len(ex.task_history) == 1
        failed = ex.task_history[0]
        assert failed.status == TaskStatus.FAILED
        assert 'exploded' in failed.error_message

    def test_unknown_task_type_raises_value_error(self):
        ex = RecordingExecutor()
        with pytest.raises(ValueError, match='Unknown task type'):
            ex.execute_task(make_task('nope'))

    def test_unknown_task_type_via_process_tasks_marks_failed(self, clock):
        ex = RecordingExecutor()
        ex.add_task(make_task('nope'))
        ex.process_tasks()
        # Unknown handler → ValueError → task lifecycle records FAILED
        assert ex.current_task is None
        assert ex.task_history[0].status == TaskStatus.FAILED
        assert 'Unknown task type' in ex.task_history[0].error_message

    def test_to_dict_round_trip(self):
        task = make_task('one_shot', x=1, y=2)
        d = task.to_dict()
        assert d['task_id'] == 't_one_shot'
        assert d['task_type'] == 'one_shot'
        assert d['parameters'] == {'x': 1, 'y': 2}
        assert d['status'] == 'pending'
        assert d['started_at'] is None
        assert d['completed_at'] is None
        assert d['error_message'] is None

    def test_handler_preset_failed_status_not_overwritten(self, clock):
        """If a handler sets task.status before returning True, it should stick."""
        ex = RecordingExecutor()

        def fails_internally(_executor, task):
            task.status = TaskStatus.FAILED
            task.error_message = 'inner error'
            return True

        ex.register_handler('inner_fail', fails_internally)
        ex.add_task(make_task('inner_fail'))
        ex.process_tasks()
        assert ex.task_history[0].status == TaskStatus.FAILED
        assert ex.task_history[0].error_message == 'inner error'


# ====================================================================
# A2. Synchronized start (start_at gating)
# ====================================================================


class TestSynchronizedStart:

    def test_future_start_at_is_not_promoted(self, clock):
        ex = RecordingExecutor()
        task = make_task('one_shot')
        task.start_at = clock.now + 5.0
        ex.add_task(task)
        # Across several ticks the future-dated head stays pending in the queue.
        for _ in range(3):
            assert ex.process_tasks() is None
        assert ex.current_task is None
        assert ex.task_queue == [task]
        assert task.status == TaskStatus.PENDING
        assert ex.calls == []

    def test_future_start_at_promotes_once_due(self, clock):
        ex = RecordingExecutor()
        task = make_task('one_shot')
        task.start_at = clock.now + 5.0
        ex.add_task(task)
        ex.process_tasks()
        assert ex.current_task is None  # not yet due

        clock.advance(5.0)  # now time.time() == start_at
        ex.process_tasks()
        # one_shot returns True → runs and completes in this tick.
        assert ex.task_history[0].status == TaskStatus.COMPLETED
        assert ex.task_history[0].started_at == clock.now

    def test_past_start_at_promotes_immediately(self, clock):
        ex = RecordingExecutor()
        task = make_task('one_shot')
        task.start_at = clock.now - 2.0
        ex.add_task(task)
        ex.process_tasks()
        assert ex.current_task is None
        assert ex.task_history[0].status == TaskStatus.COMPLETED

    def test_none_start_at_promotes_immediately(self, clock):
        # Regression guard: absent start_at behaves exactly as before.
        ex = RecordingExecutor()
        task = make_task('one_shot')
        assert task.start_at is None
        ex.add_task(task)
        ex.process_tasks()
        assert ex.current_task is None
        assert ex.task_history[0].status == TaskStatus.COMPLETED

    def test_future_head_blocks_later_queued_tasks(self, clock):
        # Head-of-queue gating: a future-dated head holds the queue (FIFO).
        ex = RecordingExecutor()
        gated = make_task('one_shot')
        gated.start_at = clock.now + 5.0
        behind = make_task('two_shot')
        ex.add_task(gated)
        ex.add_task(behind)
        ex.process_tasks()
        assert ex.current_task is None
        assert ex.task_queue == [gated, behind]

    def test_to_dict_includes_start_at(self):
        task = make_task('one_shot')
        assert task.to_dict()['start_at'] is None
        task.start_at = 1234.5
        assert task.to_dict()['start_at'] == 1234.5


# ====================================================================
# B. Registry semantics
# ====================================================================


class TestRegistry:

    def test_register_handler_lowercases_task_type(self, clock):
        ex = RecordingExecutor()
        ex.register_handler('Roll', lambda _e, _t: True)
        # registered key is lowercased
        assert 'roll' in ex._handlers
        assert 'Roll' not in ex._handlers
        # mixed-case lookup at execute time also works
        ex.add_task(TaskDescriptor(task_id='t', task_type='ROLL', parameters={}))
        ex.process_tasks()
        assert ex.task_history[0].status == TaskStatus.COMPLETED

    def test_register_handler_overwrites_existing(self, clock):
        ex = RecordingExecutor()
        first_calls = []
        second_calls = []
        ex.register_handler('foo', lambda _e, t: first_calls.append(t) or True)
        ex.register_handler('foo', lambda _e, t: second_calls.append(t) or True)
        ex.add_task(make_task('foo'))
        ex.process_tasks()
        assert first_calls == []
        assert len(second_calls) == 1

    def test_default_handlers_registered_via_subclass_hook(self):
        ex = RecordingExecutor()
        # _register_default_handlers in our test subclass registers three.
        assert set(ex._handlers.keys()) == {'one_shot', 'two_shot', 'boom'}


# ====================================================================
# C. Cancel-on-stop hook
# ====================================================================


class TestCancelOnStop:

    def test_stop_with_no_delay_cancels_current_task(self, clock):
        ex = RecordingExecutor()
        # Register a 'stop' handler that just records the call.
        ex.register_handler('stop', lambda _e, _t: True)
        ex.add_task(make_task('two_shot'))
        ex.process_tasks()
        running = ex.current_task
        assert running is not None and running.status == TaskStatus.RUNNING

        # Queue stop with no delay; next tick should cancel two_shot, then run stop.
        ex.add_task(make_task('stop'))
        ex.process_tasks()
        # two_shot should be in history as CANCELLED
        cancelled = [t for t in ex.task_history if t.task_type == 'two_shot']
        assert len(cancelled) == 1
        assert cancelled[0].status == TaskStatus.CANCELLED
        # stop should have completed
        stopped = [t for t in ex.task_history if t.task_type == 'stop']
        assert len(stopped) == 1
        assert stopped[0].status == TaskStatus.COMPLETED

    def test_stop_with_delay_does_not_cancel_current(self, clock):
        ex = RecordingExecutor()
        ex.register_handler('stop', lambda _e, _t: True)
        ex.add_task(make_task('two_shot'))
        ex.process_tasks()
        running = ex.current_task
        assert running is not None

        # delay > 0 → don't preempt the in-flight task
        ex.add_task(make_task('stop', delay=1.0))
        ex.process_tasks()
        # two_shot should still be running (got its second tick → completed)
        # But the cancellation path should NOT have been taken: nothing was
        # added to history with CANCELLED status.
        cancelled = [t for t in ex.task_history if t.status == TaskStatus.CANCELLED]
        assert cancelled == []

    def test_custom_cancel_task_type_via_class_attr(self, clock):
        class AbortingExecutor(RecordingExecutor):
            cancel_task_type = 'abort'

        ex = AbortingExecutor()
        ex.register_handler('abort', lambda _e, _t: True)
        ex.add_task(make_task('two_shot'))
        ex.process_tasks()
        assert ex.current_task is not None

        # 'stop' should NOT cancel under the custom sentinel.
        ex.register_handler('stop', lambda _e, _t: True)
        ex.add_task(make_task('stop'))
        ex.process_tasks()
        cancelled = [t for t in ex.task_history if t.status == TaskStatus.CANCELLED]
        assert cancelled == []

        # 'abort' SHOULD cancel.
        # Reset by adding a fresh two_shot first.
        ex.add_task(make_task('two_shot'))
        # Drain any queue progress
        ex.process_tasks()  # picks up two_shot fresh? Actually current_task may
        # already be the previous two_shot mid-flight. Force a clean state:
        ex.task_queue.clear()
        ex.task_history.clear()
        ex.current_task = None

        ex.add_task(make_task('two_shot'))
        ex.process_tasks()
        ex.add_task(make_task('abort'))
        ex.process_tasks()
        cancelled = [t for t in ex.task_history if t.status == TaskStatus.CANCELLED]
        assert len(cancelled) == 1
        assert cancelled[0].task_type == 'two_shot'


# ====================================================================
# D. Sphero handler regression smoke
# ====================================================================


class TestSpheroHandlerSmoke:
    """Verify each Sphero handler still produces the expected _send_* call.

    Catches silent breakage during the handler relocation (task.py → sphero_task_handlers.py).
    """

    def test_set_led_with_color_name(self, clock):
        ex = RecordingSphero()
        ex.add_task(make_task('set_led', color='red'))
        ex.process_tasks()
        assert ('led', {'red': 255, 'green': 0, 'blue': 0, 'led_type': 'main'}) in ex.sends

    def test_set_led_with_explicit_rgb(self, clock):
        ex = RecordingSphero()
        ex.add_task(make_task('set_led', red=10, green=20, blue=30))
        ex.process_tasks()
        assert ('led', {'red': 10, 'green': 20, 'blue': 30, 'led_type': 'main'}) in ex.sends

    def test_roll_indefinite(self, clock):
        ex = RecordingSphero()
        ex.add_task(make_task('roll', heading=90, speed=100))
        ex.process_tasks()
        assert ('roll', {'heading': 90, 'speed': 100, 'duration': 0.0}) in ex.sends

    def test_roll_timed(self, clock):
        ex = RecordingSphero()
        ex.add_task(make_task('roll', heading=90, speed=100, duration=2.0))
        ex.process_tasks()
        # First tick: command sent, still running
        assert ('roll', {'heading': 90, 'speed': 100, 'duration': 2.0}) in ex.sends
        assert ex.current_task is not None
        clock.advance(2.5)
        ex.process_tasks()
        assert ex.current_task is None

    def test_heading(self, clock):
        ex = RecordingSphero()
        ex.add_task(make_task('heading', heading=180))
        ex.process_tasks()
        assert ('heading', {'heading': 180}) in ex.sends

    def test_speed(self, clock):
        ex = RecordingSphero()
        ex.add_task(make_task('speed', speed=42))
        ex.process_tasks()
        assert ('speed', {'speed': 42}) in ex.sends

    def test_matrix(self, clock):
        ex = RecordingSphero()
        ex.add_task(make_task('matrix', pattern='smile', red=1, green=2, blue=3))
        ex.process_tasks()
        assert ('matrix', {'pattern': 'smile', 'red': 1, 'green': 2, 'blue': 3}) in ex.sends

    def test_collision(self, clock):
        ex = RecordingSphero()
        ex.add_task(make_task('collision', action='start', mode='tap', sensitivity='LOW'))
        ex.process_tasks()
        assert ('collision', {'action': 'start', 'mode': 'tap', 'sensitivity': 'LOW'}) in ex.sends

    def test_reflect(self, clock, monkeypatch):
        ex = RecordingSphero(heading=0)
        # Pin random offset to 0 so the reflected heading is exactly reverse.
        monkeypatch.setattr(
            'sphero_instance_controller.core.sphero.sphero_task_handlers.random.randint',
            lambda lo, hi: 0,
        )
        ex.add_task(make_task('reflect', speed=80))
        ex.process_tasks()
        assert ('roll', {'heading': 180, 'speed': 80, 'duration': 0.0}) in ex.sends

    def test_stop(self, clock):
        ex = RecordingSphero()
        ex.add_task(make_task('stop'))
        ex.process_tasks()
        assert ('stop', {}) in ex.sends

    def test_spin(self, clock):
        ex = RecordingSphero()
        ex.add_task(make_task('spin', rotations=2, speed=100))
        ex.process_tasks()
        # angle = 720, duration = 8.0
        assert ('spin', {'angle': 720, 'duration': 8.0}) in ex.sends
        # Not yet completed (still within duration)
        assert ex.current_task is not None
        clock.advance(8.5)
        ex.process_tasks()
        assert ex.current_task is None

    def test_spin_with_duration(self, clock):
        ex = RecordingSphero()
        ex.add_task(make_task('spin', duration=60, speed=120))
        ex.process_tasks()
        # duration takes precedence: angle = 360 * 60 = 21600, duration = 60.0
        assert ('spin', {'angle': 21600, 'duration': 60.0}) in ex.sends
        # Still running before the full duration elapses
        assert ex.current_task is not None
        clock.advance(60.5)
        ex.process_tasks()
        assert ex.current_task is None

    def test_circle_ccw(self, clock):
        ex = RecordingSphero()
        ex.add_task(make_task('circle', radius=50, speed=100, duration=5.0, direction='ccw'))
        ex.process_tasks()
        # Computed: outer_ratio = 52.5/50 = 1.05 → 105; inner = 47.5/50 = 0.95 → 95
        # ccw means left=inner, right=outer
        assert ('raw_motor',
                {'left_mode': 'forward', 'left_speed': 95,
                 'right_mode': 'forward', 'right_speed': 105}) in ex.sends
        clock.advance(5.5)
        ex.process_tasks()
        # Stop sent at end
        assert ('stop', {}) in ex.sends
        assert ex.current_task is None

    def test_circle_cw_swaps_motors(self, clock):
        ex = RecordingSphero()
        ex.add_task(make_task('circle', radius=50, speed=100, duration=5.0, direction='cw'))
        ex.process_tasks()
        # cw: left=outer, right=inner
        assert ('raw_motor',
                {'left_mode': 'forward', 'left_speed': 105,
                 'right_mode': 'forward', 'right_speed': 95}) in ex.sends

    def test_move_to_sends_roll_at_target_heading(self, clock):
        ex = RecordingSphero(position={'x': 0.0, 'y': 0.0})
        ex.add_task(make_task('move_to', x=100, y=0, speed=80))
        ex.process_tasks()
        # heading = atan2(0, 100) deg = 0
        assert ('roll', {'heading': 0, 'speed': 80, 'duration': 0}) in ex.sends
        assert ex.current_task is not None  # still running

    def test_move_to_completes_when_within_tolerance(self, clock):
        ex = RecordingSphero(position={'x': 99.0, 'y': 0.0})
        # default tolerance is 10cm; we're 1cm from target
        ex.add_task(make_task('move_to', x=100, y=0))
        ex.process_tasks()
        assert ('stop', {}) in ex.sends
        assert ex.current_task is None
        assert ex.task_history[0].status == TaskStatus.COMPLETED

    def test_patrol_visits_waypoints_in_order(self, clock):
        ex = RecordingSphero(position={'x': 0.0, 'y': 0.0})
        wps = [{'x': 100, 'y': 0}, {'x': 100, 'y': 100}]
        ex.add_task(make_task('patrol', waypoints=wps, speed=80))
        ex.process_tasks()
        # First waypoint: heading = 0
        assert any(s == ('roll', {'heading': 0, 'speed': 80, 'duration': 0}) for s in ex.sends)

    def test_square_generates_four_waypoints(self, clock):
        ex = RecordingSphero(position={'x': 0.0, 'y': 0.0})
        task = make_task('square', side_length=100)
        ex.add_task(task)
        ex.process_tasks()
        wps = task.parameters['waypoints']
        assert len(wps) == 4
        assert wps[0] == {'x': 100, 'y': 0}
        assert wps[1] == {'x': 100, 'y': 100}
        assert wps[2] == {'x': 0, 'y': 100}
        assert wps[3] == {'x': 0, 'y': 0}

    def test_led_sequence_advances_on_interval(self, clock):
        ex = RecordingSphero()
        seq = [
            {'red': 255, 'green': 0, 'blue': 0},
            {'red': 0, 'green': 255, 'blue': 0},
        ]
        ex.add_task(make_task('led_sequence', sequence=seq, interval=1.0))
        ex.process_tasks()  # init, no led yet (interval not elapsed)
        clock.advance(1.1)
        ex.process_tasks()  # first color
        assert ('led', {'red': 255, 'green': 0, 'blue': 0, 'led_type': 'main'}) in ex.sends
        clock.advance(1.1)
        ex.process_tasks()  # second color
        assert ('led', {'red': 0, 'green': 255, 'blue': 0, 'led_type': 'main'}) in ex.sends
        # One more tick after sequence exhausted (and not looped) finishes the task.
        clock.advance(1.1)
        ex.process_tasks()
        assert ex.current_task is None

    def test_matrix_sequence_advances_on_interval(self, clock):
        ex = RecordingSphero()
        seq = [{'pattern': 'smile', 'red': 1, 'green': 2, 'blue': 3}]
        ex.add_task(make_task('matrix_sequence', sequence=seq, interval=2.0))
        ex.process_tasks()
        clock.advance(2.1)
        ex.process_tasks()
        assert ('matrix', {'pattern': 'smile', 'red': 1, 'green': 2, 'blue': 3}) in ex.sends

    def test_custom_executes_each_command_once(self, clock):
        ex = RecordingSphero()
        commands = [
            {'type': 'led', 'red': 1, 'green': 2, 'blue': 3, 'duration': 1.0},
            {'type': 'stop', 'duration': 0.5},
        ]
        ex.add_task(make_task('custom', commands=commands))
        ex.process_tasks()  # tick 1: led emitted, command_executed=True
        assert ('led', {'red': 1, 'green': 2, 'blue': 3, 'led_type': 'main'}) in ex.sends
        clock.advance(1.1)
        ex.process_tasks()  # tick 2: index advances, executed reset (no emit)
        assert ('stop', {}) not in ex.sends
        ex.process_tasks()  # tick 3: stop emitted
        assert ('stop', {}) in ex.sends
        clock.advance(0.6)
        ex.process_tasks()  # tick 4: stop duration elapsed, index advances
        ex.process_tasks()  # tick 5: index >= len(commands) → completed
        assert ex.current_task is None

    def test_jumping_bean_disables_then_re_enables_stabilization(self, clock):
        ex = RecordingSphero()
        # Short duration / interval to keep test fast.
        ex.add_task(make_task('jumping_bean', duration=0.2, flip_interval=0.1, speed=100))
        ex.process_tasks()
        names = [s[0] for s in ex.sends]
        # First call: stabilization off
        assert names[0] == 'stabilization' and ex.sends[0][1] == {'enable': False}
        # Last two calls: stop, then stabilization on
        assert names[-2] == 'stop'
        assert names[-1] == 'stabilization' and ex.sends[-1][1] == {'enable': True}
