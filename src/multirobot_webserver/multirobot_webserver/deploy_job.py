#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Batched fleet deploy: connect robots one-per-worker per batch.

Every robot is spawned with fleet_hold, so once connected it stays holding
(BLE quiet, no sensor stream) until the webserver publishes a release on
/fleet/ble_release. A DeployJob spawns one batch (at most one robot per
worker), waits until every robot in it is connected AND holding, stable for
STABLE_SEC, then waits for the fleet to be calm (no earlier robot
'reconnecting') before the next batch. After the last batch it releases the
connected, holding job robots in a STAGGERED sequence: RELEASE_WAVE_SIZE
robots per /fleet/ble_release, interleaved across workers, each wave waiting
for its robots' startup_complete (or RELEASE_STARTUP_TIMEOUT) plus
RELEASE_GAP_SEC before the next. Every robot that drops while a batch or a
release wave is in flight is recorded against it, which shows which connects
and startups cause drops. Cancel and RELEASE NOW use the same sequence.

Pure logic, no Flask/ROS. The manager supplies three callables:
  spawn(name, worker) -> {'success': bool, 'message': str}
  robot_states() -> {name: {'status', 'holding', 'stage', 'worker',
                            'spawn_id', 'startup_complete'}}
  release(spawn_ids, reason) -> seq (int) or None
and the clock/sleep are injectable for tests.
"""

import threading
import time
from typing import Callable, Dict, List, Optional, Sequence, Tuple

# A batch is stable once all its robots are 'running' and not holding for
# this long.
STABLE_SEC = 10.0
# Per-batch limit for reaching stable, and separately for the fleet-calm wait.
BATCH_TIMEOUT_SEC = 180.0
POLL_SEC = 1.0
# Staggered fleet release: robots released per wave (fleet-wide, interleaved
# across workers), how long to wait for a released robot's startup_complete,
# and the gap before the next wave. One-at-a-time because a simultaneous
# release of 16 robots (~27 BLE commands each in 4-6 s) dropped 9 of them.
RELEASE_WAVE_SIZE = 1
RELEASE_STARTUP_TIMEOUT = 15.0
RELEASE_GAP_SEC = 1.5

# Status values that end a robot's attempt (it will not connect by waiting).
DONE_FAIL_STATUSES = ('failed', 'disconnected', 'stopped', 'unknown')
# Status values that count as a drop for a robot that was connected.
DROP_STATUSES = ('reconnecting', 'disconnected', 'failed', 'stopped')
# Robot states that count as deployed at the end of the job.
SUCCESS_STATES = ('stable', 'running')


def plan_batches(names: Sequence[str],
                 workers: Sequence[Tuple[str, int]]
                 ) -> Tuple[List[List[Tuple[str, str]]], List[str]]:
    """Split `names` (selection order) into batches of one robot per worker.

    `workers` is the ordered list of (worker name, free slots) of the ONLINE
    workers; offline workers are simply left out by the caller. Each batch
    walks the workers in that fixed order and skips a worker with no slot
    left. Returns (batches of (robot, worker)), unassigned robots).
    """
    free = {w: int(n) for w, n in workers}
    order = [w for w, _ in workers]
    batches: List[List[Tuple[str, str]]] = []
    i = 0
    while i < len(names):
        batch = []
        for w in order:
            if i >= len(names):
                break
            if free[w] <= 0:
                continue
            batch.append((names[i], w))
            free[w] -= 1
            i += 1
        if not batch:
            break
        batches.append(batch)
    return batches, list(names[i:])


class DeployJob:
    """One batched deploy. run() is blocking; start() runs it on a thread."""

    def __init__(self, job_id: str,
                 batches: List[List[Tuple[str, str]]],
                 spawn: Callable[[str, str], Dict],
                 robot_states: Callable[[], Dict[str, Dict]],
                 release: Optional[Callable[[List[str], str],
                                            Optional[int]]] = None,
                 unassigned: Sequence[str] = (),
                 skipped: Sequence[str] = (),
                 stable_sec: float = STABLE_SEC,
                 batch_timeout: float = BATCH_TIMEOUT_SEC,
                 poll_sec: float = POLL_SEC,
                 wave_size: int = RELEASE_WAVE_SIZE,
                 startup_timeout: float = RELEASE_STARTUP_TIMEOUT,
                 release_gap: float = RELEASE_GAP_SEC,
                 clock: Callable[[], float] = time.monotonic,
                 wall: Callable[[], float] = time.time,
                 sleep: Callable[[float], None] = time.sleep,
                 log: Callable[[str], None] = print):
        self.job_id = job_id
        self._spawn = spawn
        self._robot_states = robot_states
        self._release_fn = release
        self.wave_size = max(1, int(wave_size))
        self.startup_timeout = startup_timeout
        self.release_gap = release_gap
        self._release_mutex = threading.Lock()
        # Current release wave {index, total, robots, reason}, else None.
        self.release_progress: Optional[Dict] = None
        self.stable_sec = stable_sec
        self.batch_timeout = batch_timeout
        self.poll_sec = poll_sec
        self._clock = clock
        self._wall = wall
        self._sleep = sleep
        self._log = log
        self._lock = threading.Lock()
        self._cancel = False
        self.state = 'running'          # running | done | cancelled | error
        # spawning | waiting_stable | waiting_calm | releasing |
        # waiting_startup
        self.phase: Optional[str] = None
        # Fleet-level progress: batches -> holding (all connected, holding)
        # -> released.
        self.stage = 'batches'
        # Release events, one per wave: {wave, total_waves, seq, reason,
        # robots, workers, spawn_ids, at, startup ('ok'|'timeout'|'dropped'|
        # None), startup_s, robot_startup, drops}. wave is None for the
        # unstaggered trailing/abort message.
        self.releases: List[Dict] = []
        self.current = 0                # 1-based batch index, 0 = not started
        self.started_at = wall()
        self.ended_at: Optional[float] = None
        self.error: Optional[str] = None
        self.robots: Dict[str, Dict] = {}
        self.batches: List[Dict] = []
        for bi, batch in enumerate(batches, start=1):
            for name, worker in batch:
                self.robots[name] = {
                    'name': name, 'worker': worker, 'batch': bi,
                    'state': 'queued', 'status': None, 'stage': None,
                    'holding': False, 'reason': None, 'connect_s': None,
                    'spawn_id': None, 'released': False, 'startup': None,
                }
            self.batches.append({
                'index': bi, 'robots': [n for n, _ in batch],
                'workers': [w for _, w in batch], 'state': 'pending',
                'started_at': None, 'ended_at': None, 'duration_s': None,
                'calm': None, 'drops': [],
            })
        self.unassigned = list(unassigned)
        self.skipped = list(skipped)

    # --- control / reporting ---------------------------------------------

    @property
    def running(self) -> bool:
        return self.state == 'running'

    def start(self) -> threading.Thread:
        t = threading.Thread(target=self.run, name=f'deploy-{self.job_id}',
                             daemon=True)
        t.start()
        return t

    def cancel(self) -> None:
        """Stop after the current batch (it still finishes its wait)."""
        with self._lock:
            self._cancel = True

    def snapshot(self) -> Dict:
        with self._lock:
            robots = [dict(self.robots[n]) for b in self.batches
                      for n in b['robots']]
            batches = [dict(b, drops=[dict(d) for d in b['drops']])
                       for b in self.batches]
            releases = [dict(r, drops=[dict(d) for d in r['drops']],
                             robot_startup=dict(r['robot_startup']))
                        for r in self.releases]
            progress = (dict(self.release_progress)
                        if self.release_progress else None)
            results = []
            for r in robots:
                ok = r['state'] in SUCCESS_STATES
                item = {'name': r['name'], 'success': ok}
                if not ok:
                    item['reason'] = r['reason'] or r['state']
                results.append(item)
            results += [{'name': n, 'success': False,
                         'reason': 'no free worker slot'}
                        for n in self.unassigned]
            results += [{'name': n, 'success': False,
                         'reason': f'{n} already deployed'}
                        for n in self.skipped]
            deployed = sum(1 for x in results if x['success'])
            return {
                'job_id': self.job_id,
                'state': self.state,
                'phase': self.phase,
                'stage': self.stage,
                'releases': releases,
                'release_progress': progress,
                'startup_timeouts': [n for r in releases
                                     for n, v in r['robot_startup'].items()
                                     if v['startup'] == 'timeout'],
                'startup_drops': [n for r in releases
                                  for n, v in r['robot_startup'].items()
                                  if v['startup'] == 'dropped'],
                'current_batch': self.current,
                'total_batches': len(self.batches),
                'started_at': self.started_at,
                'ended_at': self.ended_at,
                'cancel_requested': self._cancel,
                'error': self.error,
                'stable_sec': self.stable_sec,
                'batch_timeout_s': self.batch_timeout,
                'batches': batches,
                'robots': robots,
                'results': results,
                'deployed': deployed,
                'failed': len(results) - deployed,
            }

    # --- main loop -------------------------------------------------------

    def run(self) -> None:
        try:
            for bi in range(len(self.batches)):
                with self._lock:
                    cancel = self._cancel
                if cancel:
                    self._cancel_rest(bi)
                    break
                self._run_batch(bi)
            with self._lock:
                self.stage = 'holding'
                self.phase = 'releasing'
                cancel = self._cancel
            # The last batch already waited for fleet calm.
            if cancel:
                self._staggered_release('manual', spawned=True)
            else:
                self._staggered_release('deploy_complete')
            with self._lock:
                self.stage = 'released'
        except Exception as exc:  # noqa: BLE001 - never kill the server
            with self._lock:
                self.error = f'{exc.__class__.__name__}: {exc}'
            self._log(f'[deploy] job {self.job_id} aborted: {self.error}')
            try:  # leave nothing holding (one message, no stagger)
                self._release_at_once('manual')
            except Exception as exc2:  # noqa: BLE001
                self._log(f'[deploy] job {self.job_id} release after abort '
                          f'failed: {exc2}')
        with self._lock:
            if self.error is not None:
                self.state = 'error'
            elif self._cancel:
                self.state = 'cancelled'
            else:
                self.state = 'done'
            self.phase = None
            self.ended_at = self._wall()
        snap = self.snapshot()
        drops = sum(len(b['drops']) for b in snap['batches'])
        rel_drops = sum(len(e['drops']) for e in snap['releases'])
        bad = snap['startup_timeouts'] + snap['startup_drops']
        self._log(f'[deploy] job {self.job_id} {snap["state"]}: '
                  f'{snap["deployed"]}/{len(snap["results"])} deployed, '
                  f'{drops} drop(s) across {len(snap["batches"])} batch(es), '
                  f'{rel_drops} drop(s) during release'
                  + (f'; startup not ok: {", ".join(bad)}' if bad else ''))

    # --- fleet release (staggered) ------------------------------------------

    def release_now(self, background: bool = True) -> List[str]:
        """RELEASE NOW: release every job robot holding right now, with the
        same staggered sequence as the end of the job. Returns the robots
        planned for release; runs on its own thread unless background=False.
        The job keeps going; released robots need not hold to be stable."""
        planned = [n for n, _ in self._release_candidates(
            self._robot_states(), spawned=False)]
        if not planned:
            return []
        if background:
            threading.Thread(
                target=self._staggered_release, args=('manual',),
                name=f'deploy-{self.job_id}-release', daemon=True).start()
        else:
            self._staggered_release('manual')
        return planned

    def _release_candidates(self, states: Dict[str, Dict], spawned: bool
                            ) -> List[Tuple[str, Optional[str]]]:
        """(name, spawn_id) of job robots still to release, in job order.

        Default: connected ('running') and holding. spawned=True
        (cancel/abort): every robot spawned so far, connected or not.
        """
        out = []
        with self._lock:
            for name, rec in self.robots.items():
                if rec['released'] or rec['state'] in ('queued', 'spawning',
                                                       'cancelled'):
                    continue
                if rec['state'] == 'failed' and rec['connect_s'] is None \
                        and rec['status'] is None:
                    continue  # spawn failed: nothing to release
                st = states.get(name)
                if st is None:
                    continue
                if not spawned and not (st.get('status') == 'running'
                                        and st.get('holding')):
                    continue
                out.append((name, st.get('spawn_id') or rec['spawn_id']))
        return out

    def _interleave(self, names: List[str]) -> List[str]:
        """Round-robin across workers in batch (config) order, so
        consecutive releases land on different Pis."""
        order: List[str] = []
        for b in self.batches:
            for w in b['workers']:
                if w not in order:
                    order.append(w)
        queues = {w: [n for n in names if self.robots[n]['worker'] == w]
                  for w in order}
        out: List[str] = []
        while any(queues.values()):
            for w in order:
                if queues[w]:
                    out.append(queues[w].pop(0))
        return out

    def _publish(self, names: List[str], spawn_ids: List[str], reason: str,
                 wave: Optional[int], total: Optional[int]) -> Dict:
        seq = self._release_fn(spawn_ids, reason) if self._release_fn else None
        event = {'wave': wave, 'total_waves': total, 'seq': seq,
                 'reason': reason, 'robots': names,
                 'workers': [self.robots[n]['worker'] for n in names],
                 'spawn_ids': spawn_ids, 'at': self._wall(),
                 'startup': None, 'startup_s': None, 'robot_startup': {},
                 'drops': []}
        with self._lock:
            for name in names:
                self.robots[name]['released'] = True
            self.releases.append(event)
        return event

    def _release_at_once(self, reason: str) -> None:
        """One message for everything spawned (abort path only)."""
        cands = self._release_candidates(self._robot_states(), spawned=True)
        if not cands:
            return
        ids = [s for _, s in cands if s]
        if len(ids) < len(cands):
            ids = ['*']
        self._publish([n for n, _ in cands], ids, reason, None, None)
        self._log(f'[deploy] releasing {len(cands)} robots at once: '
                  f'{", ".join(n for n, _ in cands)} (reason={reason})')

    def _staggered_release(self, reason: str, spawned: bool = False) -> None:
        """Release RELEASE_WAVE_SIZE robots at a time, interleaved across
        workers. After each wave, wait for every robot in it to report
        startup_complete (or drop, or RELEASE_STARTUP_TIMEOUT), then
        RELEASE_GAP_SEC, then the next wave. Drops of other robots during a
        wave are recorded against it.

        spawned=True (cancel): robots not connected yet can't start up, so
        they go in one trailing message without a wait; that message stays
        latched for when they connect. Robots with no spawn_id yet also go
        last, as ['*'].
        """
        with self._release_mutex:  # one sequence at a time (RELEASE NOW)
            states = self._robot_states()
            cands = self._release_candidates(states, spawned)
            if not cands:
                return
            sid = dict(cands)
            connected = [n for n, s in cands
                         if s and (states.get(n) or {}).get('status')
                         == 'running']
            rest = [n for n, _ in cands if n not in connected]
            order = self._interleave(connected)
            waves = [order[i:i + self.wave_size]
                     for i in range(0, len(order), self.wave_size)]
            self._log(f'[deploy] releasing {len(order)} robots in '
                      f'{len(waves)} wave(s) (reason={reason}): '
                      + ', '.join(f'{n}@{self.robots[n]["worker"]}'
                                  for n in order))
            for wi, wave in enumerate(waves, start=1):
                if wi > 1:
                    self._sleep(self.release_gap)
                with self._lock:
                    self.release_progress = {
                        'index': wi, 'total': len(waves), 'robots': wave,
                        'reason': reason}
                watch = {n for n, st in self._robot_states().items()
                         if st.get('status') == 'running' and n not in wave}
                event = self._publish(wave, [sid[n] for n in wave], reason,
                                      wi, len(waves))
                self._wait_startup(event, watch)
            if rest:
                ids = [sid[n] for n in rest if sid[n]]
                if len(ids) < len(rest):
                    ids = ['*']
                self._publish(rest, ids, reason, None, None)
                self._log(f'[deploy] releasing {len(rest)} unconnected '
                          f'robot(s) at once: {", ".join(rest)} '
                          f'(spawn_ids={ids})')
            with self._lock:
                self.release_progress = None

    def _wait_startup(self, event: Dict, watch) -> None:
        """Wait until every robot in the wave reports startup_complete,
        drops, or times out; record other robots' drops."""
        with self._lock:
            self.phase = 'waiting_startup'
        names = event['robots']
        start = self._clock()
        result: Dict[str, Tuple[str, float]] = {}
        seen_drops = set()
        while True:
            states = self._robot_states()
            now = self._clock()
            for n in names:
                if n in result:
                    continue
                st = states.get(n)
                if st is None:
                    result[n] = ('dropped', now - start)
                elif st.get('startup_complete'):
                    result[n] = ('ok', now - start)
                elif st.get('status') in DROP_STATUSES:
                    result[n] = ('dropped', now - start)
                elif now - start >= self.startup_timeout:
                    result[n] = ('timeout', now - start)
            for n in sorted(watch):
                st = states.get(n)
                if n in seen_drops or st is None:
                    continue
                if st.get('status') in DROP_STATUSES:
                    seen_drops.add(n)
                    with self._lock:
                        event['drops'].append({
                            'name': n, 'worker': st.get('worker'),
                            'status': st.get('status'),
                            't_s': round(now - start, 1)})
            if len(result) == len(names):
                break
            self._sleep(self.poll_sec)
        worst = 'ok'
        for verdict, _ in result.values():
            if verdict == 'dropped' or (verdict == 'timeout'
                                        and worst == 'ok'):
                worst = verdict
        with self._lock:
            event['robot_startup'] = {n: {'startup': v, 't_s': round(t, 1)}
                                      for n, (v, t) in result.items()}
            event['startup'] = worst
            event['startup_s'] = round(max(t for _, t in result.values()), 1)
            for n, (v, _) in result.items():
                self.robots[n]['startup'] = v
            drops = [f'{d["name"]}({d["worker"] or "local"})'
                     for d in event['drops']]
        who = ', '.join(f'{n}@{self.robots[n]["worker"]}' for n in names)
        verdicts = ', '.join(sorted({v for v, _ in result.values()}))
        tag = (f'release {event["wave"]}/{event["total_waves"]}'
               if event['wave'] else 'release')
        self._log(f'[deploy] {tag}: {who} startup {verdicts} in '
                  f'{event["startup_s"]:.1f}s; drops: '
                  f'{", ".join(drops) if drops else "none"}')

    def _cancel_rest(self, first: int) -> None:
        with self._lock:
            for b in self.batches[first:]:
                b['state'] = 'cancelled'
                for n in b['robots']:
                    self.robots[n]['state'] = 'cancelled'
                    self.robots[n]['reason'] = 'cancelled'
        self._log(f'[deploy] job {self.job_id} cancelled before batch '
                  f'{first + 1}/{len(self.batches)}')

    def _tag(self, bi: int) -> str:
        return f'batch {bi + 1}/{len(self.batches)}'

    def _run_batch(self, bi: int) -> None:
        b = self.batches[bi]
        tag = self._tag(bi)
        states = self._robot_states()
        # Robots connected before this batch: any drop of theirs is charged
        # to this batch.
        watch = {n for n, st in states.items()
                 if st.get('status') == 'running' and n not in b['robots']}
        with self._lock:
            self.current = bi + 1
            self.phase = 'spawning'
            b['state'] = 'spawning'
            b['started_at'] = self._wall()
        t_batch = self._clock()
        self._log(f'[deploy] {tag} start: ' + ', '.join(
            f'{n}@{w}' for n, w in zip(b['robots'], b['workers'])))

        for name, worker in zip(b['robots'], b['workers']):
            with self._lock:
                self.robots[name]['state'] = 'spawning'
            try:
                res = self._spawn(name, worker) or {}
            except Exception as exc:  # noqa: BLE001
                res = {'success': False, 'message': str(exc)}
            with self._lock:
                rec = self.robots[name]
                if res.get('success'):
                    rec['state'] = 'connecting'
                else:
                    rec['state'] = 'failed'
                    rec['reason'] = res.get('message') or 'spawn failed'
            if not res.get('success'):
                self._log(f'[deploy] {tag} spawn failed: {name}@{worker}: '
                          f'{res.get("message") or "spawn failed"}')

        with self._lock:
            self.phase = 'waiting_stable'
            b['state'] = 'waiting'
        outcome, waited, pending = self._wait_stable(bi, t_batch, watch)
        with self._lock:
            b['state'] = outcome
        drops = self._drops_text(b)
        if outcome == 'stable':
            self._log(f'[deploy] {tag} stable after {waited:.1f}s; '
                      f'drops during batch: {drops}')
        elif outcome == 'timeout':
            self._log(f'[deploy] {tag} timeout after {waited:.1f}s '
                      f'(not stable: {pending}); drops during batch: {drops}')
        else:
            self._log(f'[deploy] {tag} all robots failed; '
                      f'drops during batch: {drops}')

        with self._lock:
            self.phase = 'waiting_calm'
            calm_watch = watch | {n for n in b['robots']
                                  if self.robots[n]['state'] in SUCCESS_STATES}
        calm, busy = self._wait_calm(bi, t_batch, calm_watch)
        with self._lock:
            b['calm'] = calm
            b['ended_at'] = self._wall()
            b['duration_s'] = round(self._clock() - t_batch, 1)
        if not calm:
            self._log(f'[deploy] {tag} fleet not calm after '
                      f'{self.batch_timeout:.0f}s ({busy} still reconnecting); '
                      f'moving on')

    def _drops_text(self, b: Dict) -> str:
        with self._lock:
            items = [f'{d["name"]}({d["worker"] or "local"})'
                     for d in b['drops']]
        return ', '.join(items) if items else 'none'

    def _record_drops(self, bi: int, states: Dict[str, Dict], watch,
                      t_batch: float) -> None:
        """Charge each watched robot's first drop to batch `bi` (once)."""
        b = self.batches[bi]
        new = []
        with self._lock:
            seen = {d['name'] for d in b['drops']}
            # In-batch robots that already connected also count if they drop.
            in_batch = {n for n in b['robots']
                        if self.robots[n]['connect_s'] is not None}
            for name in sorted(set(watch) | in_batch):
                st = states.get(name)
                if name in seen or st is None:
                    continue
                if st.get('status') in DROP_STATUSES:
                    d = {'name': name, 'worker': st.get('worker'),
                         'status': st.get('status'),
                         'in_batch': name in b['robots'],
                         't_s': round(self._clock() - t_batch, 1)}
                    b['drops'].append(d)
                    new.append(d)
        for d in new:
            self._log(f'[deploy] {self._tag(bi)} drop: {d["name"]}'
                      f'({d["worker"] or "local"}) {d["status"]} at '
                      f'+{d["t_s"]:.1f}s')

    def _wait_stable(self, bi: int, t_batch: float, watch):
        """Wait for the batch to be stable. Returns (outcome, waited, pending
        text); outcome is 'stable', 'timeout' or 'failed' (nothing left)."""
        b = self.batches[bi]
        start = self._clock()
        stable_since = None
        while True:
            states = self._robot_states()
            now = self._clock()
            live = []
            ok = True
            with self._lock:
                for name in b['robots']:
                    rec = self.robots[name]
                    if rec['state'] in ('failed', 'timeout'):
                        continue
                    st = states.get(name)
                    if st is None:
                        rec['state'] = 'failed'
                        rec['reason'] = 'removed during deploy'
                        continue
                    status = st.get('status')
                    rec['status'] = status
                    rec['stage'] = st.get('stage')
                    rec['holding'] = bool(st.get('holding'))
                    if st.get('spawn_id'):
                        rec['spawn_id'] = st.get('spawn_id')
                    if status in DONE_FAIL_STATUSES:
                        rec['state'] = 'failed'
                        rec['reason'] = f'ended {status}'
                        continue
                    live.append(name)
                    if status == 'running':
                        if rec['connect_s'] is None:
                            rec['connect_s'] = round(now - t_batch, 1)
                        rec['state'] = 'running'
                    else:
                        rec['state'] = 'connecting'
                    # Fleet hold: connected robots stay holding until the
                    # fleet release (a robot released early by RELEASE NOW
                    # only needs to be connected).
                    if status != 'running' or not (rec['holding']
                                                   or rec['released']):
                        ok = False
            self._record_drops(bi, states, watch, t_batch)
            if not live:
                return 'failed', now - start, ''
            if ok:
                if stable_since is None:
                    stable_since = now
                if now - stable_since >= self.stable_sec:
                    with self._lock:
                        for name in live:
                            self.robots[name]['state'] = 'stable'
                    return 'stable', now - start, ''
            else:
                stable_since = None
            if now - start >= self.batch_timeout:
                pending = []
                with self._lock:
                    for name in live:
                        rec = self.robots[name]
                        if rec['status'] != 'running':
                            rec['state'] = 'timeout'
                            rec['reason'] = (
                                f'not connected after {self.batch_timeout:.0f}s'
                                f' ({rec["status"]})')
                            pending.append(f'{name} {rec["status"]}')
                        elif not (rec['holding'] or rec['released']):
                            pending.append(f'{name} not holding')
                        else:
                            pending.append(f'{name} flapping')
                return 'timeout', now - start, ', '.join(pending)
            self._sleep(self.poll_sec)

    def _wait_calm(self, bi: int, t_batch: float, watch):
        """Wait until no watched robot is 'reconnecting'. Returns (calm,
        comma-separated busy names)."""
        start = self._clock()
        while True:
            states = self._robot_states()
            self._record_drops(bi, states, watch, t_batch)
            busy = sorted(n for n in watch
                          if (states.get(n) or {}).get('status') == 'reconnecting')
            if not busy:
                return True, ''
            if self._clock() - start >= self.batch_timeout:
                return False, ', '.join(busy)
            self._sleep(self.poll_sec)
