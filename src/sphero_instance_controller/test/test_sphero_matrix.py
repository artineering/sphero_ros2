"""Sphero.set_matrix blanks the previous graphic before drawing the new one."""
from unittest.mock import MagicMock

from sphero_instance_controller.core.sphero.sphero import Sphero


class FakeMatrixApi:
    """Records matrix calls and models the 8x8 frame."""

    def __init__(self):
        self.calls = []
        self.lit = set()

    def clear_matrix(self):
        self.calls.append('clear')
        self.lit.clear()

    def set_matrix_pixel(self, row, col, color):
        self.calls.append('pixel')
        self.lit.add((row, col))


def _grid(cells):
    return [1 if (i // 8, i % 8) in cells else 0 for i in range(64)]


def test_graphic_b_leaves_no_pixels_from_a():
    api = FakeMatrixApi()
    s = Sphero(MagicMock(), api, 'SB-TEST')
    a = {(0, 0), (1, 1), (7, 7)}
    b = {(1, 1), (3, 4)}

    assert s.set_matrix(custom_matrix=_grid(a))
    api.calls.clear()
    assert s.set_matrix(custom_matrix=_grid(b))

    assert api.lit == b
    assert api.calls[0] == 'clear'
    assert api.calls.count('clear') == 1
    assert api.calls[1:] == ['pixel'] * len(b)


def test_invalid_graphic_does_not_blank():
    api = FakeMatrixApi()
    s = Sphero(MagicMock(), api, 'SB-TEST')
    assert not s.set_matrix(custom_matrix=[1, 0])
    assert api.calls == []
