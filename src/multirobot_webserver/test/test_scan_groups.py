#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Hardware-free tests for named NEARBY scan groups: JSON file store (path via
SPHERO_SCAN_GROUPS_FILE), validation, overwrite/delete, corrupt-file handling,
and the /api/scan_groups routes via the Flask test client -- no rclpy init.
"""

import json

import pytest

from multirobot_webserver import multirobot_webapp as webapp


@pytest.fixture
def gfile(tmp_path, monkeypatch):
    path = tmp_path / 'sub' / 'scan_groups.json'  # dir created on first write
    monkeypatch.setenv(webapp.SCAN_GROUPS_ENV, str(path))
    return path


@pytest.fixture
def client(gfile):
    return webapp.app.test_client()


# --- store ------------------------------------------------------------------

def test_missing_file_is_empty(gfile):
    assert webapp.list_scan_groups() == {}
    assert not gfile.exists()


def test_round_trip_dedups_sorts_and_trims(gfile):
    webapp.put_scan_group('  lab A ', ['SB-3660', 'SB-1FA8', 'SB-3660'])
    assert json.loads(gfile.read_text()) == {
        'groups': {'lab A': ['SB-1FA8', 'SB-3660']}}
    assert webapp.list_scan_groups() == {'lab A': ['SB-1FA8', 'SB-3660']}
    # No temp files left behind by the atomic write.
    assert [p.name for p in gfile.parent.iterdir()] == ['scan_groups.json']


def test_overwrite_and_delete(gfile):
    webapp.put_scan_group('g', ['SB-3660'])
    webapp.put_scan_group('h', ['SB-1FA8'])
    webapp.put_scan_group('g', ['SB-3AAC', 'SB-33E3'])
    assert webapp.list_scan_groups() == {'g': ['SB-33E3', 'SB-3AAC'],
                                         'h': ['SB-1FA8']}
    assert webapp.delete_scan_group('g') == {'h': ['SB-1FA8']}
    assert webapp.delete_scan_group('g') is None
    assert webapp.list_scan_groups() == {'h': ['SB-1FA8']}


@pytest.mark.parametrize('content', [
    '{not json', '[]', '{"groups": []}', '{"groups": {"a": "SB-3660"}}',
    '{"groups": {"a": [1]}}', '{}'])
def test_corrupt_file_reads_empty_and_is_moved_aside(gfile, content):
    gfile.parent.mkdir(parents=True)
    gfile.write_text(content)
    assert webapp.list_scan_groups() == {}
    assert not gfile.exists()
    aside = list(gfile.parent.glob('scan_groups.json.corrupt-*'))
    assert len(aside) == 1 and aside[0].read_text() == content
    # Writes still work afterwards.
    webapp.put_scan_group('g', ['SB-3660'])
    assert webapp.list_scan_groups() == {'g': ['SB-3660']}


@pytest.mark.parametrize('name,robots', [
    ('', ['SB-3660']),
    ('   ', ['SB-3660']),
    ('x' * 41, ['SB-3660']),
    (None, ['SB-3660']),
    ('g', []),
    ('g', None),
    ('g', 'SB-3660'),
    ('g', ['SB-3660', 7]),
    ('g', ['SB3660']),
    ('g', ['SB-<b>']),
    ('g', ['SB-123456789']),
    ('g', [f'SB-{i:04d}' for i in range(65)]),
])
def test_validation_rejects(gfile, name, robots):
    with pytest.raises(ValueError):
        webapp.put_scan_group(name, robots)
    assert not gfile.exists()


def test_validation_accepts_limits(gfile):
    webapp.put_scan_group('x' * 40, [f'SB-{i:04d}' for i in range(64)])


def test_group_count_cap(gfile, monkeypatch):
    monkeypatch.setattr(webapp, 'SCAN_GROUPS_MAX', 2)
    webapp.put_scan_group('a', ['SB-3660'])
    webapp.put_scan_group('b', ['SB-3660'])
    with pytest.raises(ValueError):
        webapp.put_scan_group('c', ['SB-3660'])
    webapp.put_scan_group('a', ['SB-1FA8'])  # overwrite allowed at cap


# --- routes -----------------------------------------------------------------

def test_routes_crud(client):
    assert client.get('/api/scan_groups').get_json() == {
        'success': True, 'groups': {}}
    r = client.put('/api/scan_groups/lab%20A',
                   json={'robots': ['SB-3660', 'SB-1FA8']})
    assert r.status_code == 200
    assert r.get_json()['groups'] == {'lab A': ['SB-1FA8', 'SB-3660']}
    r = client.put('/api/scan_groups/lab%20A', json={'robots': ['SB-3AAC']})
    assert r.get_json()['groups'] == {'lab A': ['SB-3AAC']}
    # Slash in a name survives URL encoding.
    r = client.put('/api/scan_groups/a%2Fb', json={'robots': ['SB-3AAC']})
    assert r.status_code == 200 and 'a/b' in r.get_json()['groups']
    r = client.delete('/api/scan_groups/lab%20A')
    assert r.status_code == 200
    assert r.get_json()['groups'] == {'a/b': ['SB-3AAC']}
    assert client.delete('/api/scan_groups/lab%20A').status_code == 404
    assert client.get('/api/scan_groups').get_json()['groups'] == {
        'a/b': ['SB-3AAC']}


@pytest.mark.parametrize('body', [
    None, 'nope', {'robots': []}, {'robots': ['bad']}, {}])
def test_route_put_400(client, body):
    if body is None:
        r = client.put('/api/scan_groups/g', data='{bad',
                       content_type='application/json')
    else:
        r = client.put('/api/scan_groups/g', json=body)
    assert r.status_code == 400
    data = r.get_json()
    assert data['success'] is False and data['message']


def test_route_put_400_name_too_long(client):
    r = client.put('/api/scan_groups/' + 'x' * 41, json={'robots': ['SB-3660']})
    assert r.status_code == 400
