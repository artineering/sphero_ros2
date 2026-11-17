"""Unit tests for the shared connect-queue helpers (topic naming, parsing)."""
import json
import socket

from sphero_instance_controller import ble_connect_queue as cq


def test_host_token_sanitises():
    assert cq.host_token('rpi4-node01') == 'rpi4_node01'
    assert cq.host_token('rpi5-main.local') == 'rpi5_main_local'
    assert cq.host_token('4pi') == 'h_4pi'
    assert cq.host_token('') == 'unknown'


def test_topic_matches_for_both_sides():
    assert cq.connect_queue_topic('rpi4-node03') == '/worker/rpi4_node03/connect_queue'


def test_resolve_host_env_override(monkeypatch):
    monkeypatch.setenv(cq.ENV_WORKER_HOST, 'rpi4-node02')
    assert cq.resolve_host() == 'rpi4-node02'
    monkeypatch.delenv(cq.ENV_WORKER_HOST)
    assert cq.resolve_host() == socket.gethostname()


def test_parse_queue_rejects_bad_input():
    for bad in (None, '', 'nope', '[]', '{}', '{"pending": 3}'):
        assert cq.parse_queue(bad) is None
    q = cq.parse_queue(json.dumps({'pending': []}))
    assert q == {'pending': []}


def test_others_pending_excludes_self_and_junk():
    q = {'pending': [{'name': 'SB-A'}, {'name': 'SB-SELF'}, 'junk', {'x': 1}]}
    assert cq.others_pending(q, 'SB-SELF') == ['SB-A']


def test_release_matches_rules():
    rel = {'spawn_ids': ['a', 'b'], 'stamp': 100.0}
    assert cq.release_matches(rel, 'b', 500.0, 2.0) == (True, 'named')
    assert cq.release_matches(rel, 'c', 50.0, 2.0) == (False, 'not_named')
    star = {'spawn_ids': ['*'], 'stamp': 100.0}
    assert cq.release_matches(star, 'c', 99.0, 2.0)[0]
    assert cq.release_matches(star, 'c', 102.0, 2.0)[0]
    assert cq.release_matches(star, 'c', 102.5, 2.0)[1].startswith('stale_release_all')
    assert cq.release_matches({'spawn_ids': ['*']}, 'c', 1.0, 2.0) == (
        False, 'release_all_without_stamp')


def test_parse_release():
    assert cq.parse_release('{"spawn_ids": []}') == {'spawn_ids': []}
    for bad in (None, 'x', '{}', '{"spawn_ids": "*"}'):
        assert cq.parse_release(bad) is None
