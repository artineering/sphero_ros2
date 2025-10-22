"""
Board module for the Battleship game.

This module provides the Board class for managing game boards of any size,
with support for alphabetic row notation and integer column notation.
"""

from typing import Tuple, List, Dict, Any
from enum import Enum


class CellState(Enum):
    """Represents the state of a cell on the board."""
    EMPTY = 'O'
    SHIP = 'S'
    HIT = 'X'
    MISS = 'M'


class Board:
    """
    Represents a Battleship game board.

    The board supports configurable dimensions and uses alphabetic notation
    for rows (A, B, C, ...) and integer notation for columns (1, 2, 3, ...).
    """

    def __init__(self, rows: int = 10, cols: int = 10):
        """
        Initialize a new game board.

        Args:
            rows: Number of rows (default 10, max 26 for A-Z)
            cols: Number of columns (default 10)

        Raises:
            ValueError: If rows > 26 or if dimensions are less than 1
        """
        if rows > 26:
            raise ValueError("Maximum 26 rows supported (A-Z)")
        if rows < 1 or cols < 1:
            raise ValueError("Board dimensions must be at least 1x1")

        self.rows = rows
        self.cols = cols

        # Initialize board with empty cells
        self._grid: List[List[CellState]] = [
            [CellState.EMPTY for _ in range(cols)] for _ in range(rows)
        ]

        # Track ship positions
        self._ships: Dict[str, List[Tuple[int, int]]] = {}

        # Track attack history
        self._attacks: List[Tuple[int, int]] = []
        self._hits: List[Tuple[int, int]] = []
        self._misses: List[Tuple[int, int]] = []

    def _alpha_to_index(self, row_alpha: str) -> int:
        """
        Convert alphabetic row notation to zero-based index.

        Args:
            row_alpha: Row letter (A, B, C, ...)

        Returns:
            Zero-based row index

        Raises:
            ValueError: If row notation is invalid
        """
        row_alpha = row_alpha.upper().strip()
        if len(row_alpha) != 1 or not row_alpha.isalpha():
            raise ValueError(f"Invalid row notation: {row_alpha}")

        index = ord(row_alpha) - ord('A')
        if index < 0 or index >= self.rows:
            raise ValueError(f"Row {row_alpha} out of bounds (valid: A-{chr(ord('A') + self.rows - 1)})")

        return index

    def _index_to_alpha(self, row_index: int) -> str:
        """
        Convert zero-based row index to alphabetic notation.

        Args:
            row_index: Zero-based row index

        Returns:
            Row letter (A, B, C, ...)
        """
        return chr(ord('A') + row_index)

    def _col_to_index(self, col_num: int) -> int:
        """
        Convert column number to zero-based index.

        Args:
            col_num: Column number (1-based)

        Returns:
            Zero-based column index

        Raises:
            ValueError: If column number is out of bounds
        """
        if col_num < 1 or col_num > self.cols:
            raise ValueError(f"Column {col_num} out of bounds (valid: 1-{self.cols})")

        return col_num - 1

    def _index_to_col(self, col_index: int) -> int:
        """
        Convert zero-based column index to column number.

        Args:
            col_index: Zero-based column index

        Returns:
            Column number (1-based)
        """
        return col_index + 1

    def get_cell(self, row: str, col: int) -> CellState:
        """
        Get the state of a cell using alphabetic row and integer column.

        Args:
            row: Row letter (A, B, C, ...)
            col: Column number (1-based)

        Returns:
            The state of the cell

        Raises:
            ValueError: If coordinates are invalid

        Example:
            >>> board = Board()
            >>> state = board.get_cell('A', 1)  # Top-left cell
        """
        row_idx = self._alpha_to_index(row)
        col_idx = self._col_to_index(col)
        return self._grid[row_idx][col_idx]

    def set_cell(self, row: str, col: int, state: CellState) -> None:
        """
        Set the state of a cell using alphabetic row and integer column.

        Args:
            row: Row letter (A, B, C, ...)
            col: Column number (1-based)
            state: New state for the cell

        Raises:
            ValueError: If coordinates are invalid

        Example:
            >>> board = Board()
            >>> board.set_cell('A', 1, CellState.SHIP)
        """
        row_idx = self._alpha_to_index(row)
        col_idx = self._col_to_index(col)
        self._grid[row_idx][col_idx] = state

    def is_valid_coordinate(self, row: str, col: int) -> bool:
        """
        Check if a coordinate is valid on this board.

        Args:
            row: Row letter (A, B, C, ...)
            col: Column number (1-based)

        Returns:
            True if coordinate is valid, False otherwise
        """
        try:
            self._alpha_to_index(row)
            self._col_to_index(col)
            return True
        except ValueError:
            return False

    def display(self, hide_ships: bool = False) -> str:
        """
        Generate a string representation of the board.

        Args:
            hide_ships: If True, hide unhit ships (for opponent view)

        Returns:
            String representation of the board

        Example:
            >>> board = Board(5, 5)
            >>> print(board.display())
        """
        # Calculate column number width
        col_width = len(str(self.cols))

        # Header with column numbers
        header = "   "  # Space for row letters
        for col_idx in range(self.cols):
            col_num = self._index_to_col(col_idx)
            header += f"{col_num:>{col_width + 1}}"

        lines = [header]
        lines.append("  " + "-" * (self.cols * (col_width + 1) + 2))

        # Board rows
        for row_idx in range(self.rows):
            row_letter = self._index_to_alpha(row_idx)
            row_str = f"{row_letter} |"

            for col_idx in range(self.cols):
                cell_state = self._grid[row_idx][col_idx]

                # Hide ships if requested (for opponent view)
                if hide_ships and cell_state == CellState.SHIP:
                    display_char = CellState.EMPTY.value
                else:
                    display_char = cell_state.value

                row_str += f" {display_char}" + " " * col_width

            row_str += "|"
            lines.append(row_str)

        lines.append("  " + "-" * (self.cols * (col_width + 1) + 2))

        return "\n".join(lines)

    def display_with_legend(self, hide_ships: bool = False) -> str:
        """
        Display the board with a legend explaining the symbols.

        Args:
            hide_ships: If True, hide unhit ships (for opponent view)

        Returns:
            String representation with legend
        """
        board_str = self.display(hide_ships)

        legend = "\nLegend:"
        legend += f"\n  {CellState.EMPTY.value} = Empty/Unknown"
        if not hide_ships:
            legend += f"\n  {CellState.SHIP.value} = Ship"
        legend += f"\n  {CellState.HIT.value} = Hit"
        legend += f"\n  {CellState.MISS.value} = Miss"

        return board_str + legend

    def place_ship(self, ship_id: str, coordinates: List[Tuple[str, int]]) -> bool:
        """
        Place a ship on the board.

        Args:
            ship_id: Unique identifier for the ship
            coordinates: List of (row, col) tuples for ship placement

        Returns:
            True if ship was placed successfully, False otherwise

        Example:
            >>> board = Board()
            >>> coords = [('A', 1), ('A', 2), ('A', 3)]
            >>> board.place_ship('destroyer', coords)
        """
        # Convert to indices and validate
        indices = []
        for row, col in coordinates:
            try:
                row_idx = self._alpha_to_index(row)
                col_idx = self._col_to_index(col)
                indices.append((row_idx, col_idx))
            except ValueError:
                return False

        # Check if all cells are empty
        for row_idx, col_idx in indices:
            if self._grid[row_idx][col_idx] != CellState.EMPTY:
                return False

        # Place the ship
        for row_idx, col_idx in indices:
            self._grid[row_idx][col_idx] = CellState.SHIP

        self._ships[ship_id] = indices
        return True

    def attack(self, row: str, col: int) -> Tuple[bool, bool]:
        """
        Attack a cell on the board.

        Args:
            row: Row letter (A, B, C, ...)
            col: Column number (1-based)

        Returns:
            Tuple of (is_hit, already_attacked)

        Raises:
            ValueError: If coordinates are invalid
        """
        row_idx = self._alpha_to_index(row)
        col_idx = self._col_to_index(col)

        # Check if already attacked
        if (row_idx, col_idx) in self._attacks:
            return (False, True)

        # Record attack
        self._attacks.append((row_idx, col_idx))

        current_state = self._grid[row_idx][col_idx]

        if current_state == CellState.SHIP:
            # Hit!
            self._grid[row_idx][col_idx] = CellState.HIT
            self._hits.append((row_idx, col_idx))
            return (True, False)
        else:
            # Miss
            self._grid[row_idx][col_idx] = CellState.MISS
            self._misses.append((row_idx, col_idx))
            return (False, False)

    def is_ship_sunk(self, ship_id: str) -> bool:
        """
        Check if a specific ship has been sunk.

        Args:
            ship_id: Unique identifier for the ship

        Returns:
            True if all ship cells have been hit, False otherwise
        """
        if ship_id not in self._ships:
            return False

        ship_coords = self._ships[ship_id]
        return all(coord in self._hits for coord in ship_coords)

    def all_ships_sunk(self) -> bool:
        """
        Check if all ships on the board have been sunk.

        Returns:
            True if all ships are sunk, False otherwise
        """
        return all(self.is_ship_sunk(ship_id) for ship_id in self._ships)

    def get_stats(self) -> Dict[str, Any]:
        """
        Get statistics about the board state.

        Returns:
            Dictionary containing board statistics
        """
        return {
            'total_attacks': len(self._attacks),
            'hits': len(self._hits),
            'misses': len(self._misses),
            'ships_placed': len(self._ships),
            'ships_sunk': sum(1 for ship_id in self._ships if self.is_ship_sunk(ship_id)),
            'all_ships_sunk': self.all_ships_sunk(),
            'accuracy': len(self._hits) / len(self._attacks) if self._attacks else 0.0
        }

    def to_message_dict(self, hide_ships: bool = False) -> Dict[str, Any]:
        """
        Generate a dictionary representation suitable for ROS2 messages.

        Args:
            hide_ships: If True, hide unhit ships (for opponent view)

        Returns:
            Dictionary containing board state information

        Example:
            >>> board = Board()
            >>> msg_dict = board.to_message_dict()
            >>> # Can be used to populate a ROS2 message
        """
        # Create grid representation
        grid_data = []
        for row_idx in range(self.rows):
            row_data = []
            for col_idx in range(self.cols):
                cell_state = self._grid[row_idx][col_idx]

                # Hide ships if requested
                if hide_ships and cell_state == CellState.SHIP:
                    row_data.append(CellState.EMPTY.value)
                else:
                    row_data.append(cell_state.value)

            grid_data.append(row_data)

        # Convert attacks to string notation
        attacks_str = [
            f"{self._index_to_alpha(r)}{self._index_to_col(c)}"
            for r, c in self._attacks
        ]

        hits_str = [
            f"{self._index_to_alpha(r)}{self._index_to_col(c)}"
            for r, c in self._hits
        ]

        misses_str = [
            f"{self._index_to_alpha(r)}{self._index_to_col(c)}"
            for r, c in self._misses
        ]

        return {
            'rows': self.rows,
            'cols': self.cols,
            'grid': grid_data,
            'attacks': attacks_str,
            'hits': hits_str,
            'misses': misses_str,
            'ships_sunk': [ship_id for ship_id in self._ships if self.is_ship_sunk(ship_id)],
            'all_ships_sunk': self.all_ships_sunk(),
            'stats': self.get_stats()
        }

    def __str__(self) -> str:
        """String representation of the board."""
        return self.display()

    def __repr__(self) -> str:
        """Detailed representation of the board."""
        return f"Board(rows={self.rows}, cols={self.cols}, ships={len(self._ships)})"
