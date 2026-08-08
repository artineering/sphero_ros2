#!/usr/bin/env python3
"""Greedy nearest-neighbour association (re-acquisition only)."""

from overhead_tracking.association import associate


def test_exact_pairs_match():
    m, up, um = associate([[0, 0], [100, 0]], [[1, 1], [99, 1]], 10.0)
    assert sorted(m) == [(0, 0), (1, 1)]
    assert up == set() and um == set()


def test_gate_rejects_distant_pairs():
    m, up, um = associate([[0, 0]], [[500, 500]], 10.0)
    assert m == [] and up == {0} and um == {0}


def test_greedy_takes_the_closest_first():
    m, _, _ = associate([[0, 0]], [[30, 0], [5, 0]], 50.0)
    assert m == [(0, 1)]


def test_surplus_measurements_are_reported_unmatched():
    m, up, um = associate([[0, 0]], [[1, 0], [200, 0]], 10.0)
    assert m == [(0, 0)] and up == set() and um == {1}


def test_surplus_predictions_are_reported_unmatched():
    m, up, um = associate([[0, 0], [200, 0]], [[1, 0]], 10.0)
    assert m == [(0, 0)] and up == {1} and um == set()


def test_empty_inputs_are_safe():
    assert associate([], [], 10.0) == ([], set(), set())
    m, up, um = associate([[0, 0]], [], 10.0)
    assert m == [] and up == {0} and um == set()


def test_each_index_is_used_at_most_once():
    m, _, _ = associate([[0, 0], [1, 0], [2, 0]], [[0, 0], [1, 0], [2, 0]], 10.0)
    assert len({i for i, _ in m}) == len(m) == len({j for _, j in m})
