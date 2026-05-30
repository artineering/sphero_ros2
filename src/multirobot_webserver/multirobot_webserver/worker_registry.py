#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Worker registry + least-loaded assignment for the distributed BLE fleet.

Loads the worker list from config/workers.yaml and tracks each worker's live
Sphero count, capacity, and online flag in memory. `select_worker()` picks the
online worker with the most free capacity (least-loaded), with a deterministic
tie-break by config order, hard-capping at each worker's capacity and rejecting
when all online workers are full.

Phase 2 scope: registry + selection + capacity accounting only. Wiring this to
the remote agents (and to add_sphero/remove) is Phase 3.
"""

import os
from typing import Dict, List, Optional

import yaml

# Per-Pi BLE link cap. Config constant; 4 is the committed/validated value.
DEFAULT_CAPACITY = 4


class Worker:
    """In-memory state for one worker node."""

    def __init__(self, name: str, host: str, port: int = 8181,
                 ssh_user: str = '', capacity: int = DEFAULT_CAPACITY,
                 online: bool = True, order: int = 0):
        self.name = name
        self.host = host
        self.port = port
        self.ssh_user = ssh_user
        self.capacity = capacity
        self.online = online
        self.order = order          # config order; deterministic tie-break key
        self.count = 0              # live Spheros assigned to this worker

    @property
    def free(self) -> int:
        return self.capacity - self.count

    @property
    def base_url(self) -> str:
        """HTTP base URL of this worker's launcher agent."""
        return f'http://{self.host}:{self.port}'

    def as_dict(self) -> Dict:
        return {
            'name': self.name,
            'host': self.host,
            'port': self.port,
            'capacity': self.capacity,
            'count': self.count,
            'free': self.free,
            'online': self.online,
        }


class WorkerRegistry:
    """Holds all workers and performs least-loaded selection."""

    def __init__(self, workers: List[Worker], agent_token: str = ''):
        # Preserve config order for deterministic tie-breaks.
        self._workers: Dict[str, Worker] = {w.name: w for w in workers}
        # Bearer token presented to every worker launcher agent. Empty string
        # means "no auth header" (dev mode). Env SPHERO_AGENT_TOKEN overrides
        # the config value at construction time.
        self.agent_token: str = os.environ.get(
            'SPHERO_AGENT_TOKEN', agent_token or ''
        )

    @classmethod
    def from_yaml(cls, path: str) -> 'WorkerRegistry':
        with open(path, 'r') as fh:
            data = yaml.safe_load(fh) or {}
        return cls.from_config(data)

    @classmethod
    def from_config(cls, data: Dict) -> 'WorkerRegistry':
        workers = []
        for i, entry in enumerate(data.get('workers', []) or []):
            workers.append(Worker(
                name=entry['name'],
                host=entry.get('host', ''),
                port=int(entry.get('port', 8181)),
                ssh_user=entry.get('ssh_user', ''),
                capacity=int(entry.get('capacity', DEFAULT_CAPACITY)),
                online=bool(entry.get('online', True)),
                order=i,
            ))
        agent_token = ''
        agent_cfg = data.get('agent') or {}
        if isinstance(agent_cfg, dict):
            agent_token = str(agent_cfg.get('token', '') or '')
        return cls(workers, agent_token=agent_token)

    def get(self, name: str) -> Optional[Worker]:
        return self._workers.get(name)

    def all(self) -> List[Worker]:
        return sorted(self._workers.values(), key=lambda w: w.order)

    def total_capacity(self) -> int:
        return sum(w.capacity for w in self._workers.values() if w.online)

    def total_count(self) -> int:
        return sum(w.count for w in self._workers.values())

    def select_worker(self) -> Worker:
        """
        Return the online, below-capacity worker with the most free slots.

        Tie-break: config order (then name) for determinism. Raises
        RuntimeError when no online worker has a free slot.
        """
        candidates = [
            w for w in self._workers.values() if w.online and w.free > 0
        ]
        if not candidates:
            used = self.total_count()
            cap = self.total_capacity()
            raise RuntimeError(
                f'all workers at capacity ({used}/{cap} BLE slots used)'
            )
        # Most free first; ties broken by config order then name.
        candidates.sort(key=lambda w: (-w.free, w.order, w.name))
        return candidates[0]

    def assign(self, worker: Worker) -> None:
        """Account one Sphero onto `worker` (caller selected it)."""
        worker.count += 1

    def release(self, name: str) -> None:
        """Account one Sphero off `name` (no-op below zero)."""
        worker = self._workers.get(name)
        if worker is not None and worker.count > 0:
            worker.count -= 1

    def set_online(self, name: str, online: bool) -> None:
        worker = self._workers.get(name)
        if worker is not None:
            worker.online = online
