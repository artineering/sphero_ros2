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
