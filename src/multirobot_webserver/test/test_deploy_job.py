#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Hardware-free tests for the batched deploy job (one robot per worker per
batch): planning, stability wait, timeout, failure, drop recording, cancel,
the manager glue and the Flask routes. Workers and robot status are faked; the
job runs synchronously on a fake clock whose sleep() advances time.
"""

import types
from unittest import mock

from multirobot_webserver import multirobot_webapp as webapp
from multirobot_webserver.deploy_job import DeployJob, plan_batches
from multirobot_webserver.multirobot_webapp import (
    DeployBusy, SpheroInstanceManager)
from multirobot_webserver.worker_registry import WorkerRegistry

W = ['rpi4-node01', 'rpi4-node02', 'rpi4-node03', 'rpi4-node04']


class FakeFleet:
    """Scripted fleet. timeline[name] = [(t_from_spawn, status, holding)];
    the last entry whose t <= now - spawn time applies. Robots connect and
    hold by default; a release naming a robot's spawn_id (or '*') clears its
    holding unless the robot is in `stuck`."""

    def __init__(self, timeline=None, existing=None, fail_spawn=(),
                 stuck=(), startup_delay=None, after_release=None):
        self.t = 0.0
        self.timeline = timeline or {}
        self.spawned = {}            # name -> (worker, t)
        self.existing = existing or {}  # name -> [(t_abs, status)]
        self.fail_spawn = set(fail_spawn)
        self.stuck = set(stuck)
        self.released_ids = set()
        self.released_at = {}        # spawn_id or '*' -> t
        # name -> seconds from its release to startup_complete (default 3)
        self.startup_delay = startup_delay or {}
        # name -> [(t_from_release, status)] overriding status after release
        self.after_release = after_release or {}
        self.releases = []           # (spawn_ids, reason, t)
        self.logs = []

    def clock(self):
        return self.t

    def sleep(self, dt):
        self.t += dt

    def spawn(self, name, worker):
        if name in self.fail_spawn:
            return {'success': False, 'message': f'agent {worker} unreachable'}
        self.spawned[name] = (worker, self.t)
        return {'success': True}

    def release(self, spawn_ids, reason):
        self.releases.append((list(spawn_ids), reason, self.t))
        self.released_ids |= set(spawn_ids)
        for sid in spawn_ids:
            self.released_at.setdefault(sid, self.t)
        return len(self.releases)

    def states(self):
        out = {}
        for name, steps in self.existing.items():
            cur = [s for t, s in steps if t <= self.t][-1]
            out[name] = {'status': cur, 'holding': False, 'stage': None,
                         'worker': 'rpi4-node03', 'spawn_id': f'sid-{name}'}
        for name, (worker, t0) in self.spawned.items():
            steps = self.timeline.get(name, [(0.0, 'running', True)])
            cur = [s for s in steps if s[0] <= self.t - t0][-1]
            sid = f'sid-{name}'
            t_rel = self.released_at.get(sid, self.released_at.get('*'))
            released = t_rel is not None
            status = cur[1]
            startup = False
            if released:
                dt = self.t - t_rel
                for t, st in self.after_release.get(name, []):
                    if t <= dt:
                        status = st
                startup = (name not in self.stuck and status == 'running'
                           and dt >= self.startup_delay.get(name, 3.0))
            out[name] = {'status': status,
                         'holding': cur[2] and not released,
                         'startup_complete': startup,
                         'stage': 'connecting' if status == 'connecting'
                         else None, 'worker': worker, 'spawn_id': sid}
        return out

    def job(self, batches, **kw):
        kw.setdefault('stable_sec', 10.0)
        kw.setdefault('batch_timeout', 180.0)
        return DeployJob('j1', batches, spawn=self.spawn,
                         robot_states=self.states, release=self.release,
                         clock=self.clock, wall=self.clock, sleep=self.sleep,
                         log=self.logs.append, **kw)


# --- plan_batches --------------------------------------------------------------

def test_plan_one_robot_per_worker_per_batch_in_selection_order():
    names = [f'SB-{i:04d}' for i in range(10)]
    batches, rest = plan_batches(names, [(w, 4) for w in W])
    assert [len(b) for b in batches] == [4, 4, 2]
    assert [n for b in batches for n, _ in b] == names
    for b in batches:
        assert [w for _, w in b] == W[:len(b)]
    assert rest == []


def test_plan_skips_offline_worker():
    # The caller passes online workers only: node02 is offline.
    online = [(w, 4) for w in W if w != 'rpi4-node02']
    batches, _ = plan_batches(['A', 'B', 'C', 'D'], online)
    assert batches == [[('A', 'rpi4-node01'), ('B', 'rpi4-node03'),
                        ('C', 'rpi4-node04')], [('D', 'rpi4-node01')]]


def test_plan_respects_free_slots_and_reports_unassigned():
    batches, rest = plan_batches(['A', 'B', 'C', 'D', 'E'],
                                 [('w1', 1), ('w2', 2)])
    assert batches == [[('A', 'w1'), ('B', 'w2')], [('C', 'w2')]]
    assert rest == ['D', 'E']


# --- DeployJob -----------------------------------------------------------------

def test_stable_needs_connected_and_holding_then_next_batch():
    fleet = FakeFleet(timeline={
        # Connected but not (yet) holding does not count as stable.
        'A': [(0, 'connecting', False), (20, 'running', False),
              (25, 'running', True)],
        'B': [(0, 'connecting', False), (5, 'running', True)],
    })
    job = fleet.job([[('A', 'w1'), ('B', 'w2')], [('C', 'w1')]])
    job.run()
    snap = job.snapshot()
    assert snap['state'] == 'done' and snap['stage'] == 'released'
    assert [b['state'] for b in snap['batches']] == ['stable', 'stable']
    # C spawned only after A was running AND holding for 10 s.
    assert fleet.spawned['C'][1] >= 35.0
    assert snap['deployed'] == 3 and snap['failed'] == 0
    robots = {r['name']: r for r in snap['robots']}
    assert robots['A']['connect_s'] == 20.0
    assert robots['A']['worker'] == 'w1' and robots['A']['batch'] == 1
    assert any('batch 1/2 start: A@w1, B@w2' in ln for ln in fleet.logs)
    assert any('batch 1/2 stable after' in ln
               and 'drops during batch: none' in ln for ln in fleet.logs)


def test_no_release_between_batches_then_staggered_one_at_a_time():
    fleet = FakeFleet(startup_delay={'A': 4.8})
    job = fleet.job([[('A', 'w1'), ('B', 'w2')], [('C', 'w1')]])
    job.run()
    # No release before the last batch is stable; then one robot per
    # message, interleaved across workers, each after the previous robot's
    # startup_complete plus the 1.5 s gap.
    assert [(ids, r) for ids, r, _ in fleet.releases] == [
        (['sid-A'], 'deploy_complete'), (['sid-B'], 'deploy_complete'),
        (['sid-C'], 'deploy_complete')]
    t = [t for _, _, t in fleet.releases]
    assert t[0] >= fleet.spawned['C'][1] + 10.0
    assert t[1] >= t[0] + 4.8 + 1.5
    assert t[2] >= t[1] + 3.0 + 1.5
    snap = job.snapshot()
    assert snap['state'] == 'done' and snap['stage'] == 'released'
    rel = snap['releases']
    assert [(e['wave'], e['total_waves'], e['seq'], e['startup'])
            for e in rel] == [(1, 3, 1, 'ok'), (2, 3, 2, 'ok'),
                              (3, 3, 3, 'ok')]
    assert rel[0]['robot_startup'] == {'A': {'startup': 'ok', 't_s': 5.0}}
    assert snap['release_progress'] is None
    assert snap['startup_timeouts'] == [] and snap['startup_drops'] == []
    assert any(ln == '[deploy] release 1/3: A@w1 startup ok in 5.0s; '
               'drops: none' for ln in fleet.logs)


def test_interleave_across_workers():
    fleet = FakeFleet()
    job = fleet.job([[('A', 'w1'), ('B', 'w2')], [('C', 'w1')],
                     [('D', 'w1'), ('E', 'w2')], [('F', 'w3')]])
    assert job._interleave(['A', 'B', 'C', 'D', 'E', 'F']) == [
        'A', 'B', 'F', 'C', 'E', 'D']


def test_wave_size_constant_groups_robots():
    fleet = FakeFleet()
    job = fleet.job([[('A', 'w1'), ('B', 'w2'), ('C', 'w3')]], wave_size=2)
    job.run()
    assert [ids for ids, _, _ in fleet.releases] == [
        ['sid-A', 'sid-B'], ['sid-C']]


def test_release_skips_failed_and_unconnected_robots():
    fleet = FakeFleet(timeline={'B': [(0, 'connecting', False)]},
                      fail_spawn={'C'})
    job = fleet.job([[('A', 'w1'), ('B', 'w2'), ('C', 'w3')]],
                    batch_timeout=30.0)
    job.run()
    assert fleet.releases == [(['sid-A'], 'deploy_complete', mock.ANY)]


def test_startup_timeout_recorded_and_moves_on():
    fleet = FakeFleet(stuck={'A'})
    job = fleet.job([[('A', 'w1'), ('B', 'w2')]])
    job.run()
    snap = job.snapshot()
    t = [t for _, _, t in fleet.releases]
    assert t[1] >= t[0] + 15.0 + 1.5
    assert [e['startup'] for e in snap['releases']] == ['timeout', 'ok']
    assert snap['startup_timeouts'] == ['A']
    assert snap['state'] == 'done'
    assert any(ln.startswith('[deploy] release 1/2: A@w1 startup timeout')
               for ln in fleet.logs)
    assert any('startup not ok: A' in ln for ln in fleet.logs)


def test_startup_drop_recorded_and_moves_on():
    fleet = FakeFleet(after_release={'A': [(2, 'reconnecting')]})
    job = fleet.job([[('A', 'w1'), ('B', 'w2')]])
    job.run()
    snap = job.snapshot()
    assert [e['startup'] for e in snap['releases']] == ['dropped', 'ok']
    assert snap['startup_drops'] == ['A']
    t = [t for _, _, t in fleet.releases]
    assert t[1] - t[0] < 5.0


def test_other_robots_dropping_during_a_wave_are_recorded():
    # X (not in the job) and A (released in wave 1) are knocked off while B
    # starts up in wave 2.
    fleet = FakeFleet(existing={'X': [(0, 'running'), (16, 'reconnecting')]},
                      after_release={'A': [(6, 'reconnecting')]})
    job = fleet.job([[('A', 'w1'), ('B', 'w2')]])
    job.run()
    snap = job.snapshot()
    t0 = fleet.releases[0][2]
    assert t0 == 10.0  # A released at t=10; B at t=14.5
    w1, w2 = snap['releases']
    assert w1['drops'] == []
    assert sorted(d['name'] for d in w2['drops']) == ['A', 'X']
    assert any(ln.startswith('[deploy] release 2/2: B@w2 startup ok')
               and 'drops: A(w1), X(rpi4-node03)' in ln
               for ln in fleet.logs)


def test_flap_resets_stable_window():
    fleet = FakeFleet(timeline={
        'A': [(0, 'running', True), (6, 'reconnecting', False),
              (8, 'running', True)],
    })
    job = fleet.job([[('A', 'w1')]])
    job.run()
    snap = job.snapshot()
    assert snap['batches'][0]['state'] == 'stable'
    # Stable needed 10 s after the flap ended at t=8.
    assert snap['batches'][0]['duration_s'] >= 18.0
    # The in-batch flap after connecting is recorded as a drop.
    drops = snap['batches'][0]['drops']
    assert [(d['name'], d['status'], d['in_batch']) for d in drops] == [
        ('A', 'reconnecting', True)]


def test_timeout_moves_on():
    fleet = FakeFleet(timeline={'A': [(0, 'connecting', False)]})
    job = fleet.job([[('A', 'w1'), ('B', 'w2')], [('C', 'w1')]],
                    batch_timeout=30.0)
    job.run()
    snap = job.snapshot()
    assert [b['state'] for b in snap['batches']] == ['timeout', 'stable']
    robots = {r['name']: r for r in snap['robots']}
    assert robots['A']['state'] == 'timeout'
    assert robots['B']['state'] == 'running'
    assert 'C' in fleet.spawned
    res = {r['name']: r for r in snap['results']}
    assert not res['A']['success']
    assert 'not connected after 30s' in res['A']['reason']
    assert res['B']['success'] and res['C']['success']
    assert any('batch 1/2 timeout after' in ln and 'A connecting' in ln
               for ln in fleet.logs)


def test_connected_but_never_holding_times_out():
    fleet = FakeFleet(timeline={'A': [(0, 'running', False)]})
    job = fleet.job([[('A', 'w1')]], batch_timeout=30.0)
    job.run()
    assert job.snapshot()['batches'][0]['state'] == 'timeout'
    assert any('A not holding' in ln for ln in fleet.logs)


def test_failed_robot_is_done_and_does_not_block():
    fleet = FakeFleet(timeline={'A': [(0, 'connecting', False),
                                      (3, 'disconnected', False)]},
                      fail_spawn={'B'})
    job = fleet.job([[('A', 'w1'), ('B', 'w2'), ('C', 'w3')]])
    job.run()
    snap = job.snapshot()
    res = {r['name']: r for r in snap['results']}
    assert res['A'] == {'name': 'A', 'success': False,
                        'reason': 'ended disconnected'}
    assert res['B']['reason'] == 'agent w2 unreachable'
    assert res['C']['success']
    assert snap['batches'][0]['state'] == 'stable'
    assert snap['batches'][0]['duration_s'] < 20.0
    assert any('spawn failed: B@w2' in ln for ln in fleet.logs)


def test_all_failed_batch_returns_immediately():
    fleet = FakeFleet(fail_spawn={'A'})
    job = fleet.job([[('A', 'w1')], [('B', 'w1')]])
    job.run()
    snap = job.snapshot()
    assert snap['batches'][0]['state'] == 'failed'
    assert fleet.spawned['B'][1] == 0.0
    assert snap['deployed'] == 1


def test_drops_of_connected_robots_recorded_per_batch_and_calm_wait():
    # X was connected before the job; it drops while batch 1 connects and is
    # reconnecting until t=40, so batch 2 waits for calm.
    fleet = FakeFleet(
        timeline={'A': [(0, 'connecting', False), (4, 'running', True)]},
        existing={'X': [(0, 'running'), (3, 'reconnecting'), (40, 'running')],
                  'Y': [(0, 'running')]})
    job = fleet.job([[('A', 'w1')], [('B', 'w1')]])
    job.run()
    snap = job.snapshot()
    b1, b2 = snap['batches']
    assert [(d['name'], d['worker'], d['status'], d['in_batch'])
            for d in b1['drops']] == [('X', 'rpi4-node03', 'reconnecting',
                                       False)]
    assert b1['calm'] is True
    assert b2['drops'] == []
    assert fleet.spawned['B'][1] >= 40.0
    assert any('batch 1/2 drop: X(rpi4-node03) reconnecting' in ln
               for ln in fleet.logs)
    assert any('batch 1/2 stable after' in ln
               and 'drops during batch: X(rpi4-node03)' in ln
               for ln in fleet.logs)
    # Robots that were not part of the job are never released by it.
    assert [ids for ids, _, _ in fleet.releases] == [['sid-A'], ['sid-B']]


def test_calm_timeout_moves_on():
    fleet = FakeFleet(existing={'X': [(0, 'running'), (1, 'reconnecting')]})
    job = fleet.job([[('A', 'w1')], [('B', 'w1')]], batch_timeout=30.0)
    job.run()
    snap = job.snapshot()
    assert snap['batches'][0]['calm'] is False
    assert 'B' in fleet.spawned
    assert any('fleet not calm' in ln and 'X still reconnecting' in ln
               for ln in fleet.logs)


def test_cancel_stops_after_current_batch_and_releases_spawned():
    fleet = FakeFleet()
    job = fleet.job([[('A', 'w1')], [('B', 'w1')], [('C', 'w1')]])
    orig = fleet.spawn

    def spawn(name, worker):
        if name == 'A':
            job.cancel()
        return orig(name, worker)
    job._spawn = spawn
    job.run()
    snap = job.snapshot()
    assert snap['state'] == 'cancelled'
    assert [b['state'] for b in snap['batches']] == [
        'stable', 'cancelled', 'cancelled']
    assert set(fleet.spawned) == {'A'}
    assert [(ids, reason) for ids, reason, _ in fleet.releases] == [
        (['sid-A'], 'manual')]
    assert snap['releases'][0]['startup'] == 'ok'
    res = {r['name']: r for r in snap['results']}
    assert res['A']['success'] and res['B']['reason'] == 'cancelled'


def test_cancel_release_includes_unconnected_spawned_robot():
    fleet = FakeFleet(timeline={'B': [(0, 'connecting', False)]})
    job = fleet.job([[('A', 'w1'), ('B', 'w2')], [('C', 'w1')]],
                    batch_timeout=20.0)
    orig = fleet.spawn

    def spawn(name, worker):
        job.cancel()
        return orig(name, worker)
    job._spawn = spawn
    job.run()
    # Connected A is staggered (waits for its startup); unconnected B goes
    # in one trailing message, which stays latched for when it connects.
    assert [(ids, reason) for ids, reason, _ in fleet.releases] == [
        (['sid-A'], 'manual'), (['sid-B'], 'manual')]
    assert job.snapshot()['releases'][1]['wave'] is None


def test_release_now_mid_job_then_final_release_for_the_rest():
    fleet = FakeFleet()
    job = fleet.job([[('A', 'w1')], [('B', 'w1')]])
    orig = fleet.spawn

    def spawn(name, worker):
        if name == 'B':
            assert job.release_now(background=False) == ['A']
        return orig(name, worker)
    job._spawn = spawn
    job.run()
    assert [(ids, reason) for ids, reason, _ in fleet.releases] == [
        (['sid-A'], 'manual'), (['sid-B'], 'deploy_complete')]
    assert job.snapshot()['batches'][1]['state'] == 'stable'


def test_release_now_released_robot_still_counts_as_stable():
    # A is released by RELEASE NOW while its own batch is still settling.
    fleet = FakeFleet()
    job = fleet.job([[('A', 'w1')]])
    orig_states = fleet.states
    done = []

    def states():
        st = orig_states()
        if 'A' in st and fleet.t >= 2 and not done:
            done.append(True)  # release_now() reads states() itself
            assert job.release_now(background=False) == ['A']
            st = orig_states()
        return st
    job._robot_states = states
    job.run()
    snap = job.snapshot()
    assert snap['error'] is None
    assert snap['batches'][0]['state'] == 'stable'
    assert len(fleet.releases) == 1  # nothing left for the final release


def test_missing_spawn_id_falls_back_to_wildcard():
    fleet = FakeFleet()
    orig = fleet.states

    def states():
        st = orig()
        for v in st.values():
            v['spawn_id'] = None
        return st
    job = DeployJob('j1', [[('A', 'w1')]], spawn=fleet.spawn,
                    robot_states=states, release=fleet.release,
                    clock=fleet.clock, wall=fleet.clock, sleep=fleet.sleep,
                    log=fleet.logs.append)
    job.run()
    assert fleet.releases[0][0] == ['*']


def test_snapshot_includes_unassigned_and_skipped():
    fleet = FakeFleet()
    job = fleet.job([[('A', 'w1')]], unassigned=['Z'], skipped=['S'])
    job.run()
    snap = job.snapshot()
    assert snap['deployed'] == 1 and snap['failed'] == 2
    res = {r['name']: r['reason'] for r in snap['results'] if not r['success']}
    assert res == {'Z': 'no free worker slot', 'S': 'S already deployed'}


# --- Manager glue ----------------------------------------------------------------

def _registry(online=(True, True, True, True)):
    reg = WorkerRegistry.from_config({'workers': [
        {'name': w, 'host': f'10.0.0.1{i}'} for i, w in enumerate(W, 1)]})
    for w, on in zip(W, online):
        reg.set_online(w, on)
    return reg


def _mgr(registry, instances=()):
    import threading
    mgr = types.SimpleNamespace(
        worker_registry=registry, instances={n: {} for n in instances},
        _deploy_lock=threading.Lock(), _deploy_jobs={})
    # Online flags come from the registry as set; no HTTP.
    mgr._agent_status_cached = lambda w: None
    mgr._dedupe = SpheroInstanceManager._dedupe
    for meth in ('_deploy_workers', 'should_batch_deploy',
                 'active_deploy_job', 'get_deploy_job', 'start_deploy_job',
                 '_deploy_spawn', '_deploy_robot_states',
                 'publish_ble_release'):
        fn = getattr(SpheroInstanceManager, meth)
        setattr(mgr, meth, (lambda f: lambda *a, **k: f(mgr, *a, **k))(fn))
    return mgr


def test_should_batch_needs_two_robots_and_two_workers():
    mgr = _mgr(_registry())
    assert mgr.should_batch_deploy(['A', 'B'])
    assert not mgr.should_batch_deploy(['A'])
    assert not mgr.should_batch_deploy(['A', 'A'])
    assert not _mgr(_registry(), instances=['A']).should_batch_deploy(
        ['A', 'B'])
    assert not _mgr(_registry((True, False, False, False))) \
        .should_batch_deploy(['A', 'B'])
    assert not _mgr(None).should_batch_deploy(['A', 'B'])


def test_start_job_plans_skips_offline_and_rejects_concurrent():
    mgr = _mgr(_registry((True, False, True, True)), instances=['S'])
    job = mgr.start_deploy_job(['A', 'B', 'S', 'C', 'D'], run=False)
    assert [[(r, w) for r, w in zip(b['robots'], b['workers'])]
            for b in job.batches] == [
        [('A', 'rpi4-node01'), ('B', 'rpi4-node03'), ('C', 'rpi4-node04')],
        [('D', 'rpi4-node01')]]
    assert job.skipped == ['S']
    try:
        mgr.start_deploy_job(['E', 'F'], run=False)
        assert False, 'expected DeployBusy'
    except DeployBusy as exc:
        assert exc.job_id == job.job_id
    assert mgr.active_deploy_job() is job
    assert mgr.get_deploy_job() is job
    assert mgr.get_deploy_job(job.job_id) is job


def test_add_sphero_remote_with_explicit_worker():
    reg = _registry((True, False, True, True))
    mgr = types.SimpleNamespace(worker_registry=reg)
    res = SpheroInstanceManager._add_sphero_remote(mgr, 'A', 'rpi4-node02')
    assert not res['success'] and 'rpi4-node02 unavailable' in res['message']
    res = SpheroInstanceManager._add_sphero_remote(mgr, 'A', 'nope')
    assert not res['success']
    # An available worker goes straight to the agent spawn (select_worker
    # would pick node01; we asked for node04).
    mgr.next_port = 5001
    mgr.external_localization_enabled = True
    mgr._agent_spawn = mock.Mock(side_effect=RuntimeError('boom'))
    res = SpheroInstanceManager._add_sphero_remote(mgr, 'A', 'rpi4-node04')
    assert mgr._agent_spawn.call_args[0][0].name == 'rpi4-node04'
    assert not res['success'] and res['message'] == 'boom'


# --- Routes ------------------------------------------------------------------------

def _route_mgr(batch=True, active=None, busy=False):
    job = DeployJob('abc12345', [[('A', 'w1'), ('B', 'w2')]],
                    spawn=lambda n, w: {'success': True},
                    robot_states=lambda: {})
    m = mock.Mock()
    m.active_deploy_job.return_value = active
    m.should_batch_deploy.return_value = batch
    if busy:
        m.start_deploy_job.side_effect = DeployBusy('zzz')
    else:
        m.start_deploy_job.return_value = job
    m.get_deploy_job.side_effect = (
        lambda job_id=None: job if job_id in (None, 'abc12345') else None)
    m.add_spheros_batch.return_value = {
        'success': True, 'deployed': 1, 'failed': 0, 'results': []}
    return m, job


def test_route_batch_starts_job():
    m, job = _route_mgr()
    with mock.patch.object(webapp, 'manager', m):
        r = webapp.app.test_client().post(
            '/api/spheros/batch', json={'names': ['A', 'B']})
    assert r.status_code == 202
    body = r.get_json()
    assert body['batched'] and body['job_id'] == 'abc12345'
    assert body['job']['total_batches'] == 1
    m.start_deploy_job.assert_called_once_with(['A', 'B'])
    m.add_spheros_batch.assert_not_called()


def test_route_batch_immediate_path_when_not_batching():
    m, _ = _route_mgr(batch=False)
    with mock.patch.object(webapp, 'manager', m):
        r = webapp.app.test_client().post(
            '/api/spheros/batch', json={'names': ['A']})
    assert r.status_code == 200 and r.get_json()['deployed'] == 1
    m.start_deploy_job.assert_not_called()


def test_route_batch_409_while_job_running():
    m, job = _route_mgr(active=types.SimpleNamespace(job_id='run1'))
    with mock.patch.object(webapp, 'manager', m):
        r = webapp.app.test_client().post(
            '/api/spheros/batch', json={'names': ['A', 'B']})
    assert r.status_code == 409 and r.get_json()['job_id'] == 'run1'
    m, _ = _route_mgr(busy=True)  # lost the race inside start_deploy_job
    with mock.patch.object(webapp, 'manager', m):
        r = webapp.app.test_client().post(
            '/api/spheros/batch', json={'names': ['A', 'B']})
    assert r.status_code == 409 and r.get_json()['job_id'] == 'zzz'


def test_route_job_status_latest_and_cancel():
    m, job = _route_mgr()
    c = webapp.app.test_client()
    with mock.patch.object(webapp, 'manager', m):
        r = c.get('/api/deploy_jobs/abc12345')
        assert r.status_code == 200
        assert r.get_json()['job']['job_id'] == 'abc12345'
        assert c.get('/api/deploy_jobs/latest').get_json()['job'][
            'job_id'] == 'abc12345'
        assert c.get('/api/deploy_jobs/nope').status_code == 404
        r = c.post('/api/deploy_jobs/abc12345/release')
        assert r.status_code == 200 and r.get_json()['released'] == []
        assert c.post('/api/deploy_jobs/nope/release').status_code == 404
        r = c.post('/api/deploy_jobs/abc12345/cancel')
        assert r.status_code == 200
        assert r.get_json()['job']['cancel_requested'] is True
        assert c.post('/api/deploy_jobs/nope/cancel').status_code == 404
        m.get_deploy_job.side_effect = lambda job_id=None: None
        assert c.get('/api/deploy_jobs/latest').get_json()['job'] is None


def test_route_fleet_release_publishes_wildcard():
    m, _ = _route_mgr()
    m.publish_ble_release.return_value = 42
    with mock.patch.object(webapp, 'manager', m):
        r = webapp.app.test_client().post('/api/fleet/ble_release')
    assert r.status_code == 200 and r.get_json()['seq'] == 42
    m.publish_ble_release.assert_called_once_with(['*'], 'manual')
    m.publish_ble_release.return_value = None
    with mock.patch.object(webapp, 'manager', m):
        r = webapp.app.test_client().post('/api/fleet/ble_release')
    assert r.status_code == 503


def test_route_job_release_now_runs_staggered_in_background():
    job = mock.Mock()
    job.release_now.return_value = ['A', 'B']
    job.snapshot.return_value = {'job_id': 'j1'}
    m = mock.Mock()
    m.get_deploy_job.return_value = job
    with mock.patch.object(webapp, 'manager', m):
        r = webapp.app.test_client().post('/api/deploy_jobs/j1/release')
    assert r.get_json()['released'] == ['A', 'B']
    job.release_now.assert_called_once_with()  # background=True default


def test_release_now_background_thread_releases():
    import threading
    fleet = FakeFleet()
    job = fleet.job([[('A', 'w1')]])
    fleet.spawn('A', 'w1')
    job.robots['A']['state'] = 'running'
    before = {t.name for t in threading.enumerate()}
    assert job.release_now() == ['A']
    for t in threading.enumerate():
        if t.name not in before:
            t.join(5)
    assert fleet.releases[0][:2] == (['sid-A'], 'manual')
    assert job.snapshot()['releases'][0]['startup'] == 'ok'


# --- fleet_hold spawn flag and the release publisher ---------------------------

def _spawn_body(fleet_hold):
    mgr = types.SimpleNamespace(worker_registry=None)
    mgr._agent_headers = lambda: {}
    worker = types.SimpleNamespace(name='w1', base_url='http://w1:8181')
    resp = mock.Mock(status_code=200)
    resp.json.return_value = {'success': True}
    with mock.patch.object(webapp.requests, 'post',
                           return_value=resp) as post:
        if fleet_hold is None:
            SpheroInstanceManager._agent_spawn(mgr, worker, 'A', 5001, True)
        else:
            SpheroInstanceManager._agent_spawn(mgr, worker, 'A', 5001, True,
                                               fleet_hold=fleet_hold)
    return post.call_args.kwargs['json']


def test_agent_spawn_body_fleet_hold_only_when_asked():
    assert _spawn_body(True)['fleet_hold'] is True
    assert 'fleet_hold' not in _spawn_body(False)
    assert 'fleet_hold' not in _spawn_body(None)


def test_job_spawns_with_fleet_hold_immediate_add_does_not():
    reg = _registry()
    mgr = types.SimpleNamespace(worker_registry=reg, instances={})
    mgr._add_sphero_remote = mock.Mock(return_value={'success': True})
    SpheroInstanceManager._deploy_spawn(mgr, 'A', 'rpi4-node02')
    mgr._add_sphero_remote.assert_called_once_with(
        'A', worker_name='rpi4-node02', fleet_hold=True)
    # Immediate path (single add / one worker): add_sphero ->
    # _add_sphero_remote(name) with fleet_hold left at its False default.
    mgr._add_sphero_remote.reset_mock()
    SpheroInstanceManager.add_sphero(mgr, 'B')
    mgr._add_sphero_remote.assert_called_once_with('B')
    # And _add_sphero_remote forwards the flag to the agent spawn.
    mgr = types.SimpleNamespace(worker_registry=reg, next_port=5001,
                                external_localization_enabled=True)
    mgr._agent_spawn = mock.Mock(side_effect=RuntimeError('stop'))
    SpheroInstanceManager._add_sphero_remote(mgr, 'A')
    assert mgr._agent_spawn.call_args.kwargs['fleet_hold'] is False
    SpheroInstanceManager._add_sphero_remote(mgr, 'A', 'rpi4-node01',
                                             fleet_hold=True)
    assert mgr._agent_spawn.call_args.kwargs['fleet_hold'] is True


def test_fleet_node_publish_ble_release_payload_and_seq():
    import json
    import threading
    sent = []
    node = types.SimpleNamespace(
        _release_lock=threading.Lock(), _release_seq=1000,
        release_pub=types.SimpleNamespace(publish=sent.append))
    logger = types.SimpleNamespace(info=lambda text: None)
    node.get_logger = lambda: logger
    s1 = webapp.FleetNode.publish_ble_release(node, ['sid-A', 'sid-B'],
                                              'deploy_complete')
    s2 = webapp.FleetNode.publish_ble_release(node, ['*'], 'manual')
    assert (s1, s2) == (1001, 1002)
    p1, p2 = (json.loads(m.data) for m in sent)
    assert p1['seq'] == 1001 and p1['spawn_ids'] == ['sid-A', 'sid-B']
    assert p1['reason'] == 'deploy_complete' and p1['stamp'] > 0
    assert p2 == dict(p2, seq=1002, spawn_ids=['*'], reason='manual')
    assert webapp.FLEET_RELEASE_TOPIC == '/fleet/ble_release'


def test_manager_release_without_fleet_node_returns_none():
    mgr = types.SimpleNamespace(fleet_node=None)
    assert SpheroInstanceManager.publish_ble_release(mgr, ['*'],
                                                     'manual') is None
