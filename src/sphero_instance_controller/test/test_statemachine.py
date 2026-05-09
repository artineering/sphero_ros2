"""Unit tests for StateMachine — basic state mechanism only.

Skipped: ROS2 subscribe/unsubscribe callback wiring, tasks API.
"""
import pytest

from sphero_instance_controller.core.sphero.statemachine import (
    StateMachine,
    DynamicState,
    ExitSpec,
)


# --------------------------------------------------------------------- fixtures


class FakeTime:
    """Stand-in for the ``time`` module inside statemachine.py."""

    def __init__(self, t: float = 1000.0):
        self.now = t

    def time(self) -> float:
        return self.now

    def advance(self, dt: float) -> None:
        self.now += dt

    def set(self, t: float) -> None:
        self.now = t


@pytest.fixture
def clock(monkeypatch):
    fake = FakeTime()
    monkeypatch.setattr(
        'sphero_instance_controller.core.sphero.statemachine.time',
        fake,
    )
    return fake


@pytest.fixture
def log():
    records = []

    def logger(message: str):
        records.append(message)

    logger.records = records
    return logger


@pytest.fixture
def sm(clock, log):
    return StateMachine(logger=log)


# ----------------------------------------------------------------- inline configs


def _leaf(name: str, **extra):
    cfg = {'name': name}
    cfg.update(extra)
    return cfg


def _state(name: str, exits=None, **extra):
    cfg = {'name': name}
    if exits is not None:
        cfg['exits'] = exits
    cfg.update(extra)
    return cfg


def make_config(states, initial='A', name='demo'):
    return {'name': name, 'initial_state': initial, 'states': states}


# ----------------------------------------------------------------- A. validation


class TestValidateConfig:
    def test_rejects_non_dict(self, sm):
        assert sm.validate_config([]) is False

    def test_rejects_missing_states(self, sm):
        assert sm.validate_config({'initial_state': 'A'}) is False

    def test_rejects_empty_states(self, sm):
        assert sm.validate_config({'states': [], 'initial_state': 'A'}) is False

    def test_rejects_missing_initial_state(self, sm):
        assert sm.validate_config({'states': [{'name': 'A'}]}) is False

    def test_rejects_unknown_initial_state(self, sm):
        cfg = make_config([_leaf('A')], initial='Z')
        assert sm.validate_config(cfg) is False

    def test_rejects_exit_unknown_destination(self, sm):
        cfg = make_config([
            _state('A', exits=[{'condition': {'type': 'always'}, 'destination': 'Z'}]),
            _leaf('B'),
        ])
        assert sm.validate_config(cfg) is False

    def test_rejects_exit_missing_condition(self, sm):
        cfg = make_config([
            _state('A', exits=[{'destination': 'B'}]),
            _leaf('B'),
        ])
        assert sm.validate_config(cfg) is False

    def test_rejects_exit_missing_destination(self, sm):
        cfg = make_config([
            _state('A', exits=[{'condition': {'type': 'always'}}]),
            _leaf('B'),
        ])
        assert sm.validate_config(cfg) is False

    def test_rejects_legacy_top_level_transitions(self, sm, log):
        cfg = make_config([_leaf('A'), _leaf('B')])
        cfg['transitions'] = [{'source': 'A', 'destination': 'B'}]
        assert sm.validate_config(cfg) is False
        assert any('exits[]' in m for m in log.records)

    def test_rejects_topic_message_on_entry_condition(self, sm):
        cfg = make_config([
            _state('A', entry_condition={'type': 'topic_message', 'topic': '/x'}),
        ])
        assert sm.validate_config(cfg) is False

    def test_rejects_invalid_operator(self, sm):
        cfg = make_config([
            _state('A', exits=[{
                'condition': {'type': 'topic_value', 'topic': '/x', 'operator': '~=', 'value': 1},
                'destination': 'B',
            }]),
            _leaf('B'),
        ])
        assert sm.validate_config(cfg) is False

    def test_rejects_timer_missing_duration(self, sm):
        cfg = make_config([
            _state('A', exits=[{'condition': {'type': 'timer'}, 'destination': 'B'}]),
            _leaf('B'),
        ])
        assert sm.validate_config(cfg) is False

    def test_rejects_topic_value_missing_value(self, sm):
        cfg = make_config([
            _state('A', exits=[{
                'condition': {'type': 'topic_value', 'topic': '/x', 'operator': '=='},
                'destination': 'B',
            }]),
            _leaf('B'),
        ])
        assert sm.validate_config(cfg) is False

    def test_rejects_unknown_condition_type(self, sm):
        cfg = make_config([
            _state('A', exits=[{
                'condition': {'type': 'magic'},
                'destination': 'B',
            }]),
            _leaf('B'),
        ])
        assert sm.validate_config(cfg) is False

    def test_accepts_leaf_only_config(self, sm):
        cfg = make_config([_leaf('A')])
        assert sm.validate_config(cfg) is True

    def test_accepts_multi_exit_state(self, sm):
        cfg = make_config([
            _state('A', exits=[
                {'condition': {'type': 'topic_message', 'topic': '/halt'}, 'destination': 'B'},
                {'condition': {'type': 'timer', 'duration': 5}, 'destination': 'C'},
            ]),
            _leaf('B'),
            _leaf('C'),
        ])
        assert sm.validate_config(cfg) is True


# ----------------------------------------------------------------- B. configure


class TestConfigure:
    def test_invalid_returns_false(self, sm):
        assert sm.configure({}) is False
        assert sm.current_state is None

    def test_sets_initial_state(self, sm):
        cfg = make_config([
            _state('A', exits=[{'condition': {'type': 'always'}, 'destination': 'B'}]),
            _leaf('B'),
        ])
        assert sm.configure(cfg) is True
        assert sm.current_state == 'A'

    def test_stamps_entry_time(self, sm, clock):
        clock.set(500.0)
        cfg = make_config([_leaf('A')])
        sm.configure(cfg)
        assert sm.states['A'].entry_time == 500.0

    def test_parses_exits_into_exitspec(self, sm):
        cfg = make_config([
            _state('A', exits=[
                {'condition': {'type': 'always'}, 'destination': 'B'},
                {'condition': {'type': 'timer', 'duration': 2}, 'destination': 'B'},
            ]),
            _leaf('B'),
        ])
        sm.configure(cfg)
        a = sm.states['A']
        assert len(a.exits) == 2
        assert all(isinstance(e, ExitSpec) for e in a.exits)
        assert a.exits[0].destination == 'B'
        assert a.exits[1].condition == {'type': 'timer', 'duration': 2}

    def test_initial_state_condition_met_is_true(self, sm):
        cfg = make_config([_leaf('A')])
        sm.configure(cfg)
        assert sm.states['A'].condition_met is True


# ----------------------------------------------------------------- C. isLeafState


class TestIsLeafState:
    def test_no_exits_key_is_leaf(self):
        s = DynamicState('s', {'name': 's'})
        assert s.isLeafState() is True

    def test_empty_exits_array_is_leaf(self):
        s = DynamicState('s', {'name': 's', 'exits': []})
        assert s.isLeafState() is True

    def test_one_exit_is_not_leaf(self):
        s = DynamicState('s', {
            'name': 's',
            'exits': [{'condition': {'type': 'always'}, 'destination': 'x'}],
        })
        assert s.isLeafState() is False

    def test_multiple_exits_is_not_leaf(self):
        s = DynamicState('s', {
            'name': 's',
            'exits': [
                {'condition': {'type': 'always'}, 'destination': 'x'},
                {'condition': {'type': 'always'}, 'destination': 'y'},
            ],
        })
        assert s.isLeafState() is False


# ----------------------------------------------------------- D. condition types


def _hop_via(condition):
    """Two-state config: A --(condition)--> B (leaf)."""
    return make_config([
        _state('A', exits=[{'condition': condition, 'destination': 'B'}]),
        _leaf('B'),
    ])


class TestExitConditionAlways:
    def test_fires_immediately(self, sm):
        sm.configure(_hop_via({'type': 'always'}))
        sm.process()
        assert sm.current_state == 'B'


class TestExitConditionTimer:
    def test_blocks_before_duration(self, sm, clock):
        sm.configure(_hop_via({'type': 'timer', 'duration': 5}))
        clock.advance(4.0)
        sm.process()
        assert sm.current_state == 'A'

    def test_fires_at_duration_boundary(self, sm, clock):
        sm.configure(_hop_via({'type': 'timer', 'duration': 5}))
        clock.advance(5.0)
        sm.process()
        assert sm.current_state == 'B'

    def test_fires_after_duration(self, sm, clock):
        sm.configure(_hop_via({'type': 'timer', 'duration': 5}))
        clock.advance(10.0)
        sm.process()
        assert sm.current_state == 'B'


class TestExitConditionTopicValue:
    @pytest.mark.parametrize('operator,sent,expected,fires', [
        ('==', 5, 5, True),   ('==', 4, 5, False),
        ('!=', 4, 5, True),   ('!=', 5, 5, False),
        ('>',  6, 5, True),   ('>',  5, 5, False),
        ('<',  4, 5, True),   ('<',  5, 5, False),
        ('>=', 5, 5, True),   ('>=', 4, 5, False),
        ('<=', 5, 5, True),   ('<=', 6, 5, False),
    ])
    def test_per_operator(self, sm, operator, sent, expected, fires):
        sm.configure(_hop_via({
            'type': 'topic_value', 'topic': '/v', 'operator': operator, 'value': expected,
        }))
        sm.update_topic_value('/v', sent)
        sm.process()
        assert sm.current_state == ('B' if fires else 'A')

    def test_topic_never_received_does_not_fire(self, sm):
        sm.configure(_hop_via({
            'type': 'topic_value', 'topic': '/v', 'operator': '==', 'value': 1,
        }))
        sm.process()
        assert sm.current_state == 'A'

    def test_compare_exception_does_not_fire(self, sm, log):
        sm.configure(_hop_via({
            'type': 'topic_value', 'topic': '/v', 'operator': '>', 'value': 5,
        }))
        sm.update_topic_value('/v', object())  # unorderable vs int
        sm.process()
        assert sm.current_state == 'A'
        assert any('Error comparing topic values' in m for m in log.records)

    def test_value_update_between_ticks(self, sm):
        sm.configure(_hop_via({
            'type': 'topic_value', 'topic': '/v', 'operator': '==', 'value': 5,
        }))
        sm.update_topic_value('/v', 1)
        sm.process()
        assert sm.current_state == 'A'
        sm.update_topic_value('/v', 5)
        sm.process()
        assert sm.current_state == 'B'


class TestExitConditionTopicMessage:
    def test_does_not_fire_before_message(self, sm, clock):
        sm.configure(_hop_via({'type': 'topic_message', 'topic': '/halt'}))
        clock.advance(100.0)
        sm.process()
        assert sm.current_state == 'A'

    def test_fires_after_first_message(self, sm):
        sm.configure(_hop_via({'type': 'topic_message', 'topic': '/halt'}))
        sm.update_topic_value('/halt', None)
        sm.process()
        assert sm.current_state == 'B'

    def test_within_timeout(self, sm, clock):
        sm.configure(_hop_via({'type': 'topic_message', 'topic': '/halt', 'timeout': 2.0}))
        sm.update_topic_value('/halt', None)
        clock.advance(1.5)
        sm.process()
        assert sm.current_state == 'B'

    def test_past_timeout_does_not_fire(self, sm, clock):
        sm.configure(_hop_via({'type': 'topic_message', 'topic': '/halt', 'timeout': 2.0}))
        sm.update_topic_value('/halt', None)
        clock.advance(3.0)
        sm.process()
        assert sm.current_state == 'A'

    def test_at_timeout_boundary_inclusive(self, sm, clock):
        sm.configure(_hop_via({'type': 'topic_message', 'topic': '/halt', 'timeout': 2.0}))
        sm.update_topic_value('/halt', None)
        clock.advance(2.0)
        sm.process()
        assert sm.current_state == 'B'


# ----------------------------------------------------------- E. multi-exit ordering


class TestMultiExitOrdering:
    def test_picks_first_matching_in_declared_order(self, sm):
        cfg = make_config([
            _state('A', exits=[
                {'condition': {'type': 'always'}, 'destination': 'B'},
                {'condition': {'type': 'always'}, 'destination': 'C'},
            ]),
            _leaf('B'),
            _leaf('C'),
        ])
        sm.configure(cfg)
        sm.process()
        assert sm.current_state == 'B'

    def test_skips_non_matching_first_exit(self, sm):
        # First exit's condition is False (topic never received); second fires.
        cfg = make_config([
            _state('A', exits=[
                {'condition': {'type': 'topic_message', 'topic': '/never'}, 'destination': 'B'},
                {'condition': {'type': 'always'}, 'destination': 'C'},
            ]),
            _leaf('B'),
            _leaf('C'),
        ])
        sm.configure(cfg)
        sm.process()
        assert sm.current_state == 'C'

    def test_no_matching_exit_no_transition(self, sm, clock):
        cfg = make_config([
            _state('A', exits=[
                {'condition': {'type': 'topic_message', 'topic': '/never'}, 'destination': 'B'},
                {'condition': {'type': 'timer', 'duration': 100}, 'destination': 'C'},
            ]),
            _leaf('B'),
            _leaf('C'),
        ])
        sm.configure(cfg)
        clock.advance(1.0)
        sm.process()
        assert sm.current_state == 'A'

    def test_dynamic_first_match_wins(self, sm, clock):
        # Same source, two exits; whichever condition becomes true first wins on that tick.
        cfg = make_config([
            _state('A', exits=[
                {'condition': {'type': 'topic_value', 'topic': '/x', 'operator': '==', 'value': 1},
                 'destination': 'B'},
                {'condition': {'type': 'topic_value', 'topic': '/x', 'operator': '==', 'value': 2},
                 'destination': 'C'},
            ]),
            _leaf('B'),
            _leaf('C'),
        ])
        sm.configure(cfg)
        sm.update_topic_value('/x', 2)
        sm.process()
        assert sm.current_state == 'C'


# ----------------------------------------------------------------- F. leaf states


class TestLeafState:
    def test_leaf_state_never_transitions(self, sm, clock):
        sm.configure(make_config([_leaf('A')]))
        for _ in range(5):
            clock.advance(10.0)
            sm.process()
        assert sm.current_state == 'A'

    def test_process_on_leaf_returns_status_with_no_event(self, sm):
        sm.configure(make_config([_leaf('A')]))
        result = sm.process()
        assert result is not None
        assert result['current_state'] == 'A'
        assert result['events'] == []


# --------------------------------------------------------- G. transition_to_state


class TestTransitionToState:
    def test_happy_path(self, sm):
        sm.configure(make_config([
            _state('A', exits=[{'condition': {'type': 'always'}, 'destination': 'B'}]),
            _leaf('B'),
        ]))
        assert sm.transition_to_state('B') is True
        assert sm.current_state == 'B'

    def test_unknown_state_returns_false(self, sm, log):
        sm.configure(make_config([_leaf('A')]))
        assert sm.transition_to_state('Z') is False
        assert sm.current_state == 'A'
        assert any('unknown state' in m.lower() for m in log.records)

    def test_resets_entry_time_to_clock(self, sm, clock):
        sm.configure(make_config([
            _state('A', exits=[{'condition': {'type': 'always'}, 'destination': 'B'}]),
            _leaf('B'),
        ]))
        clock.advance(50.0)
        sm.transition_to_state('B')
        assert sm.states['B'].entry_time == clock.now

    def test_resets_task_completed_flag(self, sm):
        sm.configure(make_config([
            _state('A', exits=[{'condition': {'type': 'always'}, 'destination': 'B'}]),
            _leaf('B'),
        ]))
        sm.states['B'].task_completed = True
        sm.transition_to_state('B')
        assert sm.states['B'].task_completed is False


# --------------------------------------------------------------- H. state timeout


class TestStateTimeout:
    def test_emits_event_after_threshold(self, sm, clock):
        cfg = make_config([_leaf('A', timeout=5.0)])
        sm.configure(cfg)
        clock.advance(6.0)
        result = sm.process()
        assert any(e['type'] == 'state_timeout' for e in result['events'])

    def test_no_event_at_exact_threshold(self, sm, clock):
        # process() uses strict `>` on the timeout, so equal does not emit.
        cfg = make_config([_leaf('A', timeout=5.0)])
        sm.configure(cfg)
        clock.advance(5.0)
        result = sm.process()
        assert all(e['type'] != 'state_timeout' for e in result['events'])

    def test_event_includes_elapsed_and_state(self, sm, clock):
        cfg = make_config([_leaf('A', timeout=5.0)])
        sm.configure(cfg)
        clock.advance(7.0)
        result = sm.process()
        evt = next(e for e in result['events'] if e['type'] == 'state_timeout')
        assert evt['state'] == 'A'
        assert evt['elapsed'] == pytest.approx(7.0)

    def test_no_timeout_field_no_event(self, sm, clock):
        cfg = make_config([_leaf('A')])
        sm.configure(cfg)
        clock.advance(1000.0)
        result = sm.process()
        assert all(e['type'] != 'state_timeout' for e in result['events'])

    def test_timeout_and_transition_coexist(self, sm, clock):
        cfg = make_config([
            _state('A', timeout=5.0, exits=[
                {'condition': {'type': 'timer', 'duration': 5}, 'destination': 'B'},
            ]),
            _leaf('B'),
        ])
        sm.configure(cfg)
        clock.advance(6.0)
        result = sm.process()
        types = [e['type'] for e in result['events']]
        assert 'state_timeout' in types
        assert 'state_transition' in types
        # Timeout is checked first.
        assert types.index('state_timeout') < types.index('state_transition')


# --------------------------------------------------------------- I. process()


class TestProcessLifecycle:
    def test_unconfigured_returns_none(self, sm):
        assert sm.process() is None

    def test_no_event_returns_status_dict(self, sm):
        sm.configure(make_config([_leaf('A')]))
        result = sm.process()
        assert result['current_state'] == 'A'
        assert result['events'] == []
        assert 'time_in_state' in result

    def test_emits_state_transition_event_shape(self, sm, clock):
        sm.configure(make_config([
            _state('A', exits=[{'condition': {'type': 'always'}, 'destination': 'B'}]),
            _leaf('B'),
        ]))
        clock.advance(2.0)
        result = sm.process()
        evt = next(e for e in result['events'] if e['type'] == 'state_transition')
        assert evt['from'] == 'A'
        assert evt['to'] == 'B'
        assert evt['timestamp'] == clock.now


# ---------------------------------------------------------------- J. updaters


class TestUpdateTopicValue:
    def test_stamps_last_received(self, sm, clock):
        clock.set(123.0)
        sm.update_topic_value('/x', 42)
        assert sm.topic_values['/x'] == 42
        assert sm.topic_last_received['/x'] == 123.0

    def test_overwrite_replaces_value_and_stamp(self, sm, clock):
        clock.set(100.0)
        sm.update_topic_value('/x', 1)
        clock.set(200.0)
        sm.update_topic_value('/x', 99)
        assert sm.topic_values['/x'] == 99
        assert sm.topic_last_received['/x'] == 200.0


# ------------------------------------------------------------------- K. status


class TestGetStatus:
    def test_unconfigured(self, sm):
        s = sm.get_status()
        assert s['configured'] is False
        assert s['current_state'] is None
        assert 'timestamp' in s

    def test_configured_basic_shape(self, sm):
        sm.configure(make_config([
            _state('A', exits=[{'condition': {'type': 'always'}, 'destination': 'B'}]),
            _leaf('B'),
        ]))
        s = sm.get_status()
        assert s['configured'] is True
        assert s['current_state'] == 'A'
        assert s['num_states'] == 2
        assert s['num_exits'] == 1
        assert s['is_leaf_state'] is False

    def test_time_in_state_uses_clock(self, sm, clock):
        sm.configure(make_config([_leaf('A')]))
        clock.advance(3.0)
        s = sm.get_status()
        assert s['time_in_state'] == pytest.approx(3.0)

    def test_leaf_state_status_reflects_zero_exits(self, sm):
        sm.configure(make_config([_leaf('A')]))
        s = sm.get_status()
        assert s['is_leaf_state'] is True
        assert s['num_exits'] == 0


# --------------------------------------------------------------- L. pause/resume


class TestPauseResume:
    def test_pause_when_unconfigured_returns_false(self, sm):
        assert sm.pause() is False
        assert sm.paused is False

    def test_pause_happy_path_sets_flag(self, sm):
        sm.configure(make_config([_leaf('A')]))
        assert sm.pause() is True
        assert sm.paused is True

    def test_pause_when_already_paused_returns_false(self, sm):
        sm.configure(make_config([_leaf('A')]))
        sm.pause()
        assert sm.pause() is False

    def test_resume_when_not_paused_returns_false(self, sm):
        sm.configure(make_config([_leaf('A')]))
        assert sm.resume() is False

    def test_resume_clears_flag(self, sm):
        sm.configure(make_config([_leaf('A')]))
        sm.pause()
        assert sm.resume() is True
        assert sm.paused is False

    def test_paused_process_returns_status_with_no_events(self, sm, clock):
        sm.configure(_hop_via({'type': 'always'}))
        sm.pause()
        result = sm.process()
        assert result['paused'] is True
        assert result['events'] == []
        # Crucially, current_state did NOT advance even though `always` would normally fire.
        assert sm.current_state == 'A'

    def test_paused_process_does_not_fire_timer_exit(self, sm, clock):
        sm.configure(_hop_via({'type': 'timer', 'duration': 5}))
        clock.advance(2.0)
        sm.pause()
        clock.advance(10.0)  # would fire if not paused
        sm.process()
        assert sm.current_state == 'A'
        sm.resume()
        sm.process()  # immediately after resume the timer is still mid-flight
        assert sm.current_state == 'A'
        clock.advance(3.0)  # post-resume, 2 + 3 = 5s of unpaused time
        sm.process()
        assert sm.current_state == 'B'

    def test_resume_shifts_entry_time_forward_by_paused_duration(self, sm, clock):
        sm.configure(make_config([_leaf('A')]))
        original_entry = sm.states['A'].entry_time
        clock.advance(2.0)
        sm.pause()
        clock.advance(7.0)
        sm.resume()
        # entry_time should have moved forward by exactly the paused duration (7.0).
        assert sm.states['A'].entry_time == pytest.approx(original_entry + 7.0)

    def test_resume_shifts_topic_last_received_forward(self, sm, clock):
        sm.configure(_hop_via({'type': 'topic_message', 'topic': '/halt', 'timeout': 5.0}))
        sm.update_topic_value('/halt', None)
        ts_before = sm.topic_last_received['/halt']
        clock.advance(1.0)
        sm.pause()
        clock.advance(10.0)
        sm.resume()
        assert sm.topic_last_received['/halt'] == pytest.approx(ts_before + 10.0)
        # Within the 5s window measured from the *shifted* timestamp, the exit fires.
        sm.process()
        assert sm.current_state == 'B'


# ----------------------------------------------------------------- M. clear


class TestClear:
    def test_clear_resets_to_unconfigured(self, sm):
        sm.configure(_hop_via({'type': 'always'}))
        sm.clear()
        assert sm.current_state is None
        assert sm.states == {}
        assert sm.config is None
        assert sm.paused is False

    def test_clear_when_unconfigured_is_safe_noop(self, sm):
        # No exception, no state change.
        sm.clear()
        assert sm.current_state is None

    def test_clear_calls_unsubscribe_for_each_subscribed_topic(self, clock, log):
        unsub_calls = []
        sub_calls = []
        sm = StateMachine(
            logger=log,
            topic_subscribe_callback=lambda t, m, f: sub_calls.append(t),
            topic_unsubscribe_callback=lambda t: unsub_calls.append(t),
        )
        cfg = make_config([
            _state('A', exits=[
                {'condition': {'type': 'topic_value', 'topic': '/v',
                               'msg_type': 'std_msgs/Float32',
                               'operator': '>', 'value': 1.0},
                 'destination': 'B'},
            ]),
            _leaf('B'),
        ])
        sm.configure(cfg)
        sm.update_topic_value('/v', 0.0)
        sm.clear()
        assert '/v' in unsub_calls

    def test_clear_clears_topic_value_caches(self, sm):
        sm.configure(make_config([_leaf('A')]))
        sm.update_topic_value('/x', 42)
        sm.update_sensor_data({'velocity_x': 1.0})
        sm.clear()
        assert sm.topic_values == {}
        assert sm.topic_last_received == {}
        assert sm.sensor_topic_values == {}

    def test_clear_then_configure_again_works(self, sm, clock):
        sm.configure(_hop_via({'type': 'always'}))
        sm.clear()
        clock.advance(1.0)
        # Reconfigure with a fresh config; fresh state machine should be live.
        sm.configure(_hop_via({'type': 'timer', 'duration': 2.0}))
        assert sm.current_state == 'A'
        clock.advance(2.0)
        sm.process()
        assert sm.current_state == 'B'

    def test_clear_drops_paused_state(self, sm):
        sm.configure(make_config([_leaf('A')]))
        sm.pause()
        sm.clear()
        assert sm.paused is False


# ----------------------------------------------------- N. paused field in status


class TestStatusPausedField:
    def test_unconfigured_status_includes_paused(self, sm):
        s = sm.get_status()
        assert s['paused'] is False

    def test_configured_status_includes_paused(self, sm):
        sm.configure(make_config([_leaf('A')]))
        s = sm.get_status()
        assert s['paused'] is False
        sm.pause()
        s = sm.get_status()
        assert s['paused'] is True


# =================================================================
# NESTED STATE MACHINES
# =================================================================


def _composite(name, sub_initial, sub_states, **state_extra):
    """Compose a state with a sub_machine block."""
    cfg = {'name': name}
    cfg.update(state_extra)
    cfg['sub_machine'] = {
        'initial_state': sub_initial,
        'states': sub_states,
    }
    return cfg


# ----------------------------------------------------- O. composite validation


class TestCompositeValidation:
    def test_composite_state_config_parses(self, sm):
        cfg = make_config([
            _composite('outer', 'inner_a', [_leaf('inner_a'), _leaf('inner_b')]),
        ], initial='outer')
        assert sm.validate_config(cfg) is True

    def test_validate_rejects_sub_machine_unknown_initial_state(self, sm):
        cfg = make_config([
            _composite('outer', 'ghost', [_leaf('inner_a')]),
        ], initial='outer')
        assert sm.validate_config(cfg) is False

    def test_validate_rejects_sub_machine_exit_to_unknown_destination(self, sm):
        cfg = make_config([
            _composite('outer', 'a', [
                _state('a', exits=[{'condition': {'type': 'always'}, 'destination': 'ghost'}]),
            ]),
        ], initial='outer')
        assert sm.validate_config(cfg) is False

    def test_validate_recurses_through_arbitrary_depth(self, sm):
        # Three levels: top → mid → bottom (composite all the way down).
        cfg = make_config([
            _composite('top_a', 'mid_a', [
                _composite('mid_a', 'bot_a', [_leaf('bot_a'), _leaf('bot_b')]),
            ]),
        ], initial='top_a')
        assert sm.validate_config(cfg) is True


# ----------------------------------------------------- P. path-based runtime


class TestPathRuntime:
    def test_initial_path_is_root_plus_initial_substate(self, sm):
        cfg = make_config([
            _composite('outer', 'inner_a', [_leaf('inner_a'), _leaf('inner_b')]),
        ], initial='outer')
        sm.configure(cfg)
        assert sm.get_active_path() == ['outer', 'inner_a']

    def test_get_status_path_is_full_path(self, sm):
        cfg = make_config([
            _composite('outer', 'inner_a', [_leaf('inner_a')]),
        ], initial='outer')
        sm.configure(cfg)
        s = sm.get_status()
        assert s['path'] == ['outer', 'inner_a']

    def test_get_status_sub_status_present_when_composite(self, sm):
        cfg = make_config([
            _composite('outer', 'inner_a', [_leaf('inner_a')]),
        ], initial='outer')
        sm.configure(cfg)
        s = sm.get_status()
        assert s['sub_status'] is not None
        assert s['sub_status']['current_state'] == 'inner_a'
        assert s['is_composite'] is True

    def test_isComposite_method(self, sm):
        cfg = make_config([
            _composite('outer', 'inner_a', [_leaf('inner_a')]),
            _leaf('plain'),
        ], initial='outer')
        sm.configure(cfg)
        assert sm.states['outer'].isComposite() is True
        assert sm.states['plain'].isComposite() is False


# ----------------------------------------------------- Q. parent-first priority


class TestParentFirstPriority:
    def test_parent_exit_fires_when_both_parent_and_child_eligible(self, sm, clock):
        # Parent has an `always` exit to idle. Child also has an `always` exit.
        # Parent-first means we leave the composite entirely on the first tick.
        cfg = make_config([
            _composite('outer', 'inner_a', [
                _state('inner_a', exits=[{'condition': {'type': 'always'}, 'destination': 'inner_b'}]),
                _leaf('inner_b'),
            ], exits=[{'condition': {'type': 'always'}, 'destination': 'idle'}]),
            _leaf('idle'),
        ], initial='outer')
        sm.configure(cfg)
        sm.process()
        assert sm.get_active_path() == ['idle']

    def test_child_exit_fires_when_parent_has_no_match(self, sm, clock):
        cfg = make_config([
            _composite('outer', 'inner_a', [
                _state('inner_a', exits=[{'condition': {'type': 'always'}, 'destination': 'inner_b'}]),
                _leaf('inner_b'),
            ]),  # no parent exits
        ], initial='outer')
        sm.configure(cfg)
        sm.process()
        assert sm.get_active_path() == ['outer', 'inner_b']

    def test_supervisor_exit_preempts_running_child(self, sm, clock):
        # Parent listens for /halt; child is mid-timer. Halt arrives → parent fires
        # before the child's timer would complete.
        cfg = make_config([
            _composite('patrol', 'north', [
                _state('north', exits=[{'condition': {'type': 'timer', 'duration': 100}, 'destination': 'east'}]),
                _leaf('east'),
            ], exits=[{'condition': {'type': 'topic_message', 'topic': '/halt'}, 'destination': 'idle'}]),
            _leaf('idle'),
        ], initial='patrol')
        sm.configure(cfg)
        clock.advance(2.0)
        sm.process()
        assert sm.get_active_path() == ['patrol', 'north']
        sm.update_topic_value('/halt', None)
        sm.process()
        assert sm.get_active_path() == ['idle']


# ----------------------------------------------------- R. no-history reset


class TestNoHistoryReset:
    def _round_trip_config(self):
        return make_config([
            _composite('patrol', 'north', [
                _state('north', exits=[{'condition': {'type': 'timer', 'duration': 0.05}, 'destination': 'east'}]),
                _leaf('east'),
            ], exits=[{'condition': {'type': 'topic_message', 'topic': '/halt'}, 'destination': 'idle'}]),
            _state('idle', exits=[{'condition': {'type': 'topic_message', 'topic': '/go'}, 'destination': 'patrol'}]),
        ], initial='patrol')

    def test_old_parent_substate_is_reset_on_outer_exit(self, sm, clock):
        sm.configure(self._round_trip_config())
        # Drive child into 'east'.
        clock.advance(0.06)
        sm.process()
        assert sm.get_active_path() == ['patrol', 'east']
        # Outer halt → patrol exits.
        sm.update_topic_value('/halt', None)
        sm.process()
        # Inner SM should have been reset.
        patrol_sub = sm.states['patrol'].sub_machine
        assert patrol_sub.current_state is None
        assert patrol_sub.states['east'].entry_time is None

    def test_re_entering_composite_starts_at_initial(self, sm, clock):
        sm.configure(self._round_trip_config())
        clock.advance(0.06)
        sm.process()  # → patrol·east
        sm.update_topic_value('/halt', None)
        sm.process()  # → idle
        sm.update_topic_value('/go', None)
        sm.process()  # → patrol·north (NOT east)
        assert sm.get_active_path() == ['patrol', 'north']


# ----------------------------------------------------- S. pause/resume nesting


class TestPauseResumeNested:
    def test_pause_freezes_nested_timers(self, sm, clock):
        cfg = make_config([
            _composite('outer', 'inner', [
                _state('inner', exits=[{'condition': {'type': 'timer', 'duration': 5.0}, 'destination': 'done'}]),
                _leaf('done'),
            ]),
        ], initial='outer')
        sm.configure(cfg)
        clock.advance(2.0)
        sm.pause()
        clock.advance(20.0)  # would massively overshoot the 5s timer if pause didn't freeze it
        sm.process()
        assert sm.get_active_path() == ['outer', 'inner']

    def test_resume_shifts_nested_entry_times_forward_by_paused_duration(self, sm, clock):
        cfg = make_config([
            _composite('outer', 'inner', [
                _state('inner', exits=[{'condition': {'type': 'timer', 'duration': 5.0}, 'destination': 'done'}]),
                _leaf('done'),
            ]),
        ], initial='outer')
        sm.configure(cfg)
        inner = sm.states['outer'].sub_machine.states['inner']
        original_entry = inner.entry_time
        clock.advance(2.0)
        sm.pause()
        clock.advance(7.0)
        sm.resume()
        # entry_time at the inner level shifts by the paused duration.
        assert inner.entry_time == pytest.approx(original_entry + 7.0)
        # Now timer fires after the remaining 3s.
        clock.advance(3.0)
        sm.process()
        assert sm.get_active_path() == ['outer', 'done']


# ----------------------------------------------------- T. clear with nesting


class TestClearNested:
    def test_clear_unsubscribes_topics_at_all_depths(self, clock, log):
        unsub_calls = []
        sub_calls = []
        sm = StateMachine(
            logger=log,
            topic_subscribe_callback=lambda t, m, f: sub_calls.append(t),
            topic_unsubscribe_callback=lambda t: unsub_calls.append(t),
        )
        cfg = make_config([
            _composite('outer', 'inner', [
                _state('inner', exits=[{
                    'condition': {'type': 'topic_value', 'topic': '/inner_topic',
                                  'msg_type': 'std_msgs/Float32',
                                  'operator': '>', 'value': 1.0},
                    'destination': 'done',
                }]),
                _leaf('done'),
            ], exits=[{
                'condition': {'type': 'topic_message', 'topic': '/halt',
                              'msg_type': 'std_msgs/Empty'},
                'destination': 'idle',
            }]),
            _leaf('idle'),
        ], initial='outer')
        sm.configure(cfg)
        sm.update_topic_value('/inner_topic', 0.0)
        sm.update_topic_value('/halt', None)
        sm.clear()
        assert '/halt' in unsub_calls
        assert '/inner_topic' in unsub_calls

    def test_clear_resets_full_tree(self, sm):
        cfg = make_config([
            _composite('outer', 'inner', [_leaf('inner')]),
        ], initial='outer')
        sm.configure(cfg)
        sm.clear()
        assert sm.current_state is None
        assert sm.states == {}
        assert sm.get_active_path() == []


# ------------------------------------------------- U. state_transition path shape


class TestTransitionEventPath:
    def test_transition_event_at_root_has_path_of_one(self, sm):
        cfg = make_config([
            _state('a', exits=[{'condition': {'type': 'always'}, 'destination': 'b'}]),
            _leaf('b'),
        ], initial='a')
        sm.configure(cfg)
        result = sm.process()
        evt = next(e for e in result['events'] if e['type'] == 'state_transition')
        assert evt['path'] == ['b']

    def test_transition_event_in_subsm_has_path_with_parent_name(self, sm):
        cfg = make_config([
            _composite('outer', 'inner_a', [
                _state('inner_a', exits=[{'condition': {'type': 'always'}, 'destination': 'inner_b'}]),
                _leaf('inner_b'),
            ]),
        ], initial='outer')
        sm.configure(cfg)
        result = sm.process()
        evt = next(e for e in result['events'] if e['type'] == 'state_transition')
        assert evt['path'] == ['outer', 'inner_b']
        assert evt['from'] == 'inner_a'
        assert evt['to'] == 'inner_b'


# =================================================================
# EDGE-TRIGGERED topic_message
# =================================================================


class TestTopicMessageEdgeTrigger:
    def test_topic_message_no_timeout_fires_once_then_clears(self, sm):
        # No-timeout topic_message used to be sticky (would re-fire forever).
        # New semantics: receipt of a message fires exactly one transition; the
        # timestamp is cleared on fire, so the next tick won't see it again.
        cfg = make_config([
            _state('A', exits=[{
                'condition': {'type': 'topic_message', 'topic': '/halt'},
                'destination': 'B',
            }]),
            _state('B', exits=[{
                'condition': {'type': 'topic_message', 'topic': '/halt'},
                'destination': 'A',
            }]),
        ], initial='A')
        sm.configure(cfg)
        sm.update_topic_value('/halt', None)
        sm.process()
        assert sm.current_state == 'B'
        # Tick again with no fresh message: B should NOT immediately exit back.
        sm.process()
        assert sm.current_state == 'B'

    def test_topic_message_re_fires_on_new_message(self, sm):
        cfg = make_config([
            _state('A', exits=[{
                'condition': {'type': 'topic_message', 'topic': '/event'},
                'destination': 'B',
            }]),
            _state('B', exits=[{
                'condition': {'type': 'topic_message', 'topic': '/event'},
                'destination': 'A',
            }]),
        ], initial='A')
        sm.configure(cfg)
        sm.update_topic_value('/event', None); sm.process()
        assert sm.current_state == 'B'
        sm.update_topic_value('/event', None); sm.process()
        assert sm.current_state == 'A'
        sm.update_topic_value('/event', None); sm.process()
        assert sm.current_state == 'B'


# =================================================================
# LAZY PER-STATE SUBSCRIPTIONS (refcounted)
# =================================================================


def _ws_sm(sub_calls, unsub_calls):
    return StateMachine(
        logger=lambda m: None,
        topic_subscribe_callback=lambda t, m, f: sub_calls.append(t),
        topic_unsubscribe_callback=lambda t: unsub_calls.append(t),
    )


class TestLazySubscriptions:
    def test_configure_subscribes_only_active_path_topics(self):
        # State A is initial and references /a. State B is inactive and
        # references /b. Only /a should be subscribed at configure-time.
        subs, unsubs = [], []
        sm = _ws_sm(subs, unsubs)
        cfg = make_config([
            _state('A', exits=[{
                'condition': {'type': 'topic_value', 'topic': '/a',
                              'msg_type': 'std_msgs/Float32',
                              'operator': '>', 'value': 1.0},
                'destination': 'B',
            }]),
            _state('B', exits=[{
                'condition': {'type': 'topic_value', 'topic': '/b',
                              'msg_type': 'std_msgs/Float32',
                              'operator': '>', 'value': 1.0},
                'destination': 'A',
            }]),
        ], initial='A')
        sm.configure(cfg)
        assert subs == ['/a']
        assert '/b' not in subs

    def test_transition_subscribes_destination_unsubscribes_source(self):
        subs, unsubs = [], []
        sm = _ws_sm(subs, unsubs)
        cfg = make_config([
            _state('A', exits=[{
                'condition': {'type': 'topic_value', 'topic': '/a',
                              'msg_type': 'std_msgs/Float32',
                              'operator': '>', 'value': 1.0},
                'destination': 'B',
            }]),
            _state('B', exits=[{
                'condition': {'type': 'topic_value', 'topic': '/b',
                              'msg_type': 'std_msgs/Float32',
                              'operator': '>', 'value': 1.0},
                'destination': 'A',
            }]),
        ], initial='A')
        sm.configure(cfg)
        sm.update_topic_value('/a', 5.0)
        sm.process()
        assert sm.current_state == 'B'
        assert '/a' in unsubs       # source unsubscribed
        assert '/b' in subs          # destination subscribed

    def test_clear_releases_active_subscriptions(self):
        subs, unsubs = [], []
        sm = _ws_sm(subs, unsubs)
        cfg = make_config([
            _state('A', exits=[{
                'condition': {'type': 'topic_value', 'topic': '/a',
                              'msg_type': 'std_msgs/Float32',
                              'operator': '>', 'value': 1.0},
                'destination': 'B',
            }]),
            _leaf('B'),
        ], initial='A')
        sm.configure(cfg)
        assert sm._active_subscriptions == {'/a': 1}
        sm.clear()
        assert sm._active_subscriptions == {}
        assert '/a' in unsubs

    def test_refcount_holds_topic_subscribed_across_levels(self):
        # Parent and child both reference /shared, with disjoint thresholds so
        # parent-first priority doesn't preempt the child. Refcount should hold
        # the underlying subscription alive until BOTH levels release it.
        subs, unsubs = [], []
        sm = _ws_sm(subs, unsubs)
        cfg = make_config([
            _composite('outer', 'inner_a', [
                _state('inner_a', exits=[{
                    'condition': {'type': 'topic_value', 'topic': '/shared',
                                  'msg_type': 'std_msgs/Float32',
                                  'operator': '>', 'value': 100.0},
                    'destination': 'inner_b',
                }]),
                _leaf('inner_b'),
            ], exits=[{
                'condition': {'type': 'topic_value', 'topic': '/shared',
                              'msg_type': 'std_msgs/Float32',
                              'operator': '>', 'value': 1000.0},
                'destination': 'idle',
            }]),
            _leaf('idle'),
        ], initial='outer')
        sm.configure(cfg)
        # /shared subscribed exactly once, refcount 2 (parent + inner_a).
        assert sm._active_subscriptions['/shared'] == 2
        assert subs.count('/shared') == 1
        # Set /shared above the child threshold but below the parent threshold.
        # Child's exit should fire (parent still references /shared).
        sm.update_topic_value('/shared', 500.0)
        sm.process()
        assert sm.get_active_path() == ['outer', 'inner_b']
        assert sm._active_subscriptions['/shared'] == 1
        assert unsubs.count('/shared') == 0   # underlying sub still alive
        # Now drive parent above its threshold → outer → idle, releases /shared.
        sm.update_topic_value('/shared', 2000.0)
        sm.process()
        assert sm.get_active_path() == ['idle']
        assert '/shared' not in sm._active_subscriptions
        assert unsubs.count('/shared') == 1
