#!/usr/bin/env python3
"""
Game Controller Node for Battleship Game.

This node manages the game state, player boards, and coordinates gameplay.
It subscribes to board management topics and publishes game state updates.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
from typing import Dict, Optional, Tuple, List

# Import the Board class
import sys
import os
sys.path.append(os.path.dirname(__file__))
from core.board import Board, CellState


class GameControllerNode(Node):
    """
    Central game controller for managing Battleship game state.

    Manages multiple player boards, processes game actions, and publishes
    game state updates.
    """

    def __init__(self):
        """Initialize the game controller node."""
        super().__init__('game_controller_node')

        # Dictionary to store player boards
        self._boards: Dict[str, Board] = {}

        # Game state tracking
        self._game_active = False
        self._current_turn = None
        self._players: List[str] = []

        # Create subscribers for board management
        self.new_board_sub = self.create_subscription(
            String,
            'battleship/new_board',
            self.new_board_callback,
            10
        )

        self.setup_board_sub = self.create_subscription(
            String,
            'battleship/setup_board',
            self.setup_board_callback,
            10
        )

        self.display_board_sub = self.create_subscription(
            String,
            'battleship/display_board',
            self.display_board_callback,
            10
        )

        self.attack_sub = self.create_subscription(
            String,
            'battleship/attack',
            self.attack_callback,
            10
        )

        # Create publishers for game state and responses
        self.board_state_pub = self.create_publisher(
            String,
            'battleship/board_state',
            10
        )

        self.attack_result_pub = self.create_publisher(
            String,
            'battleship/attack_result',
            10
        )

        self.game_status_pub = self.create_publisher(
            String,
            'battleship/game_status',
            10
        )

        self.get_logger().info('Game Controller Node initialized')
        self.get_logger().info('Subscribed to:')
        self.get_logger().info('  - battleship/new_board')
        self.get_logger().info('  - battleship/setup_board')
        self.get_logger().info('  - battleship/display_board')
        self.get_logger().info('  - battleship/attack')

    def new_board_callback(self, msg: String):
        """
        Handle new board creation requests.

        Expected JSON format:
        {
            "player_id": "player1",
            "rows": 10,
            "cols": 10
        }
        """
        try:
            data = json.loads(msg.data)
            player_id = data.get('player_id')
            rows = data.get('rows', 10)
            cols = data.get('cols', 10)

            if not player_id:
                self.get_logger().error('Missing player_id in new_board request')
                return

            # Create new board for player
            try:
                board = Board(rows=rows, cols=cols)
                self._boards[player_id] = board

                if player_id not in self._players:
                    self._players.append(player_id)

                self.get_logger().info(
                    f'Created new {rows}x{cols} board for player: {player_id}'
                )

                # Publish confirmation
                status_msg = String()
                status_msg.data = json.dumps({
                    'action': 'board_created',
                    'player_id': player_id,
                    'rows': rows,
                    'cols': cols,
                    'success': True
                })
                self.game_status_pub.publish(status_msg)

            except ValueError as e:
                self.get_logger().error(f'Failed to create board: {str(e)}')
                status_msg = String()
                status_msg.data = json.dumps({
                    'action': 'board_created',
                    'player_id': player_id,
                    'success': False,
                    'error': str(e)
                })
                self.game_status_pub.publish(status_msg)

        except json.JSONDecodeError as e:
            self.get_logger().error(f'Invalid JSON in new_board message: {str(e)}')
        except Exception as e:
            self.get_logger().error(f'Error in new_board_callback: {str(e)}')

    def setup_board_callback(self, msg: String):
        """
        Handle board setup requests (ship placement).

        Expected JSON format:
        {
            "player_id": "player1",
            "ships": [
                {
                    "ship_id": "carrier",
                    "coordinates": ["A1", "A2", "A3", "A4", "A5"]
                },
                {
                    "ship_id": "battleship",
                    "coordinates": ["C1", "C2", "C3", "C4"]
                }
            ]
        }
        """
        try:
            data = json.loads(msg.data)
            player_id = data.get('player_id')
            ships = data.get('ships', [])

            if not player_id:
                self.get_logger().error('Missing player_id in setup_board request')
                return

            if player_id not in self._boards:
                self.get_logger().error(f'No board exists for player: {player_id}')
                return

            board = self._boards[player_id]
            placement_results = []

            # Place each ship
            for ship_data in ships:
                ship_id = ship_data.get('ship_id')
                coord_strings = ship_data.get('coordinates', [])

                # Parse coordinates from strings like "A1" to ("A", 1)
                coordinates = []
                for coord_str in coord_strings:
                    if len(coord_str) < 2:
                        self.get_logger().error(f'Invalid coordinate format: {coord_str}')
                        continue

                    row = coord_str[0].upper()
                    try:
                        col = int(coord_str[1:])
                        coordinates.append((row, col))
                    except ValueError:
                        self.get_logger().error(f'Invalid coordinate format: {coord_str}')
                        continue

                # Attempt to place ship
                success = board.place_ship(ship_id, coordinates)
                placement_results.append({
                    'ship_id': ship_id,
                    'success': success,
                    'coordinates': coord_strings
                })

                if success:
                    self.get_logger().info(
                        f'Placed ship "{ship_id}" for {player_id} at {coord_strings}'
                    )
                else:
                    self.get_logger().warning(
                        f'Failed to place ship "{ship_id}" for {player_id}'
                    )

            # Publish setup results
            status_msg = String()
            status_msg.data = json.dumps({
                'action': 'board_setup',
                'player_id': player_id,
                'results': placement_results
            })
            self.game_status_pub.publish(status_msg)

        except json.JSONDecodeError as e:
            self.get_logger().error(f'Invalid JSON in setup_board message: {str(e)}')
        except Exception as e:
            self.get_logger().error(f'Error in setup_board_callback: {str(e)}')

    def display_board_callback(self, msg: String):
        """
        Handle board display requests.

        Expected JSON format:
        {
            "player_id": "player1",
            "hide_ships": false
        }
        """
        try:
            data = json.loads(msg.data)
            player_id = data.get('player_id')
            hide_ships = data.get('hide_ships', False)

            if not player_id:
                self.get_logger().error('Missing player_id in display_board request')
                return

            if player_id not in self._boards:
                self.get_logger().error(f'No board exists for player: {player_id}')
                return

            board = self._boards[player_id]

            # Get board state as message dict
            board_dict = board.to_message_dict(hide_ships=hide_ships)
            board_dict['player_id'] = player_id

            # Publish board state
            state_msg = String()
            state_msg.data = json.dumps(board_dict)
            self.board_state_pub.publish(state_msg)

            self.get_logger().info(f'Published board state for {player_id}')

        except json.JSONDecodeError as e:
            self.get_logger().error(f'Invalid JSON in display_board message: {str(e)}')
        except Exception as e:
            self.get_logger().error(f'Error in display_board_callback: {str(e)}')

    def attack_callback(self, msg: String):
        """
        Handle attack requests.

        Expected JSON format:
        {
            "attacker_id": "player1",
            "target_id": "player2",
            "coordinate": "A5"
        }
        """
        try:
            data = json.loads(msg.data)
            attacker_id = data.get('attacker_id')
            target_id = data.get('target_id')
            coordinate = data.get('coordinate')

            if not all([attacker_id, target_id, coordinate]):
                self.get_logger().error('Missing required fields in attack request')
                return

            if target_id not in self._boards:
                self.get_logger().error(f'No board exists for target: {target_id}')
                return

            # Parse coordinate
            if len(coordinate) < 2:
                self.get_logger().error(f'Invalid coordinate format: {coordinate}')
                return

            row = coordinate[0].upper()
            try:
                col = int(coordinate[1:])
            except ValueError:
                self.get_logger().error(f'Invalid coordinate format: {coordinate}')
                return

            # Execute attack on target's board
            board = self._boards[target_id]
            is_hit, already_attacked = board.attack(row, col)

            # Check if ship was sunk
            ship_sunk = False
            ship_id = ""
            for sid in board._ships:
                if board.is_ship_sunk(sid):
                    # Check if this ship was just sunk
                    ship_coords = board._ships[sid]
                    row_idx = board._alpha_to_index(row)
                    col_idx = board._col_to_index(col)
                    if (row_idx, col_idx) in ship_coords:
                        ship_sunk = True
                        ship_id = sid
                        break

            # Check for game over
            game_over = board.all_ships_sunk()
            winner_id = attacker_id if game_over else ""

            # Publish attack result
            result_msg = String()
            result_msg.data = json.dumps({
                'attacker_id': attacker_id,
                'target_id': target_id,
                'coordinate': coordinate,
                'is_hit': is_hit,
                'already_attacked': already_attacked,
                'ship_sunk': ship_sunk,
                'ship_id': ship_id,
                'game_over': game_over,
                'winner_id': winner_id
            })
            self.attack_result_pub.publish(result_msg)

            # Log the attack
            result_str = "HIT" if is_hit else "MISS"
            if already_attacked:
                result_str = "ALREADY ATTACKED"
            self.get_logger().info(
                f'{attacker_id} attacked {target_id} at {coordinate}: {result_str}'
            )

            if ship_sunk:
                self.get_logger().info(f'Ship "{ship_id}" has been sunk!')

            if game_over:
                self.get_logger().info(f'GAME OVER! Winner: {winner_id}')

        except json.JSONDecodeError as e:
            self.get_logger().error(f'Invalid JSON in attack message: {str(e)}')
        except ValueError as e:
            self.get_logger().error(f'Invalid coordinate in attack: {str(e)}')
        except Exception as e:
            self.get_logger().error(f'Error in attack_callback: {str(e)}')

    def get_board_display(self, player_id: str, hide_ships: bool = False) -> str:
        """
        Get ASCII display of a player's board.

        Args:
            player_id: The player whose board to display
            hide_ships: Whether to hide unhit ships

        Returns:
            String representation of the board
        """
        if player_id not in self._boards:
            return f"No board exists for {player_id}"

        return self._boards[player_id].display(hide_ships=hide_ships)


def main(args=None):
    """Main entry point for the game controller node."""
    rclpy.init(args=args)

    node = GameControllerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt, shutting down')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
