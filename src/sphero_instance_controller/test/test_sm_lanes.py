"""Offline tests for the state-machine controller LANE model (phase 2, step 0).

Drives the real node logic headless (rclpy.init, no spin): two lane-tagged
configs load into two SMs and both tick; an absent `lane` lands on `default`; a
lane-scoped clear targets only its lane; `scope:"all"` clears every lane.
"""
import json

import pytest
import rclpy
from std_msgs.msg import String

from sphero_instance_controller.sphero_instance_statemachine_controller_node import (
    SpheroInstanceStateMachineController,
    DEFAULT_SM_LANE,
)


class TaskRecorder:
    """Stand-in for the node's task publisher; records published task dicts."""

    def __init__(self):
        self.tasks = []

    def publish(self, msg):
        self.tasks.append(json.loads(msg.data))


def _ping_pong_config(lane, task_type, name):
    """Two-state timer(0) ping-pong: fires `task_type` on every entry/tick."""
    step = {'task_id': 't', 'task_type': task_type, 'parameters': {}}
    return {
        'name': name,
        'lane': lane,
        'initial_state': 'a',
        'states': [
            {'name': 'a', 'tasks': [step],
             'exits': [{'condition': {'type': 'timer', 'duration': 0.0}, 'destination': 'b'}]},
            {'name': 'b', 'tasks': [step],
             'exits': [{'condition': {'type': 'timer', 'duration': 0.0}, 'destination': 'a'}]},
        ],
    }


def _cfg_msg(config):
    m = String()
    m.data = json.dumps(config)
    return m


def _ctl_msg(**payload):
    m = String()
    m.data = json.dumps(payload)
    return m


@pytest.fixture
def node():
    rclpy.init()
    n = SpheroInstanceStateMachineController('SB-TEST')
    n.task_pub = TaskRecorder()
    yield n
    n.destroy_node()
    rclpy.shutdown()


def test_two_lane_tagged_configs_both_load_and_tick(node):
    node.config_callback(_cfg_msg(_ping_pong_config('led', 'set_led', 'blink')))
    node.config_callback(_cfg_msg(_ping_pong_config('drive', 'proximity_step', 'prox')))

    assert node.state_machines['led'].get_status()['configured'] is True
    assert node.state_machines['drive'].get_status()['configured'] is True

    node.task_pub.tasks.clear()
    node.update_callback()  # one tick advances BOTH lanes

    lanes_fired = {t['task_id'].split('_')[1] for t in node.task_pub.tasks}
    assert 'led' in lanes_fired      # task_id sm_led_<state>_<idx>
    assert 'drive' in lanes_fired
    types = {t['task_type'] for t in node.task_pub.tasks}
    assert {'set_led', 'proximity_step'} <= types


def test_absent_lane_lands_on_default(node):
    cfg = _ping_pong_config('drive', 'roll', 'x')
    del cfg['lane']  # omit lane entirely
    node.config_callback(_cfg_msg(cfg))

    assert node.state_machines[DEFAULT_SM_LANE].get_status()['configured'] is True
    for lane in ('drive', 'led', 'matrix', 'config'):
        assert node.state_machines[lane].get_status()['configured'] is False


def test_lane_scoped_clear_targets_only_its_lane(node):
    node.config_callback(_cfg_msg(_ping_pong_config('led', 'set_led', 'blink')))
    node.config_callback(_cfg_msg(_ping_pong_config('drive', 'proximity_step', 'prox')))

    node.control_callback(_ctl_msg(action='clear', lane='led'))

    assert node.state_machines['led'].get_status()['configured'] is False
    assert node.state_machines['drive'].get_status()['configured'] is True


def test_scope_all_clear_targets_every_lane(node):
    node.config_callback(_cfg_msg(_ping_pong_config('led', 'set_led', 'blink')))
    node.config_callback(_cfg_msg(_ping_pong_config('drive', 'proximity_step', 'prox')))

    node.control_callback(_ctl_msg(action='clear', scope='all'))

    assert node.state_machines['led'].get_status()['configured'] is False
    assert node.state_machines['drive'].get_status()['configured'] is False
