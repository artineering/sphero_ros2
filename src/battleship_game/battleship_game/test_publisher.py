#!/usr/bin/env python3
"""
Test publisher for battleship game messages.

This script sends test messages to verify the game controller functionality.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import time


class TestPublisher(Node):
    """Test publisher for sending sample messages to the game controller."""

    def __init__(self):
        """Initialize the test publisher node."""
        super().__init__('test_publisher')

        # Create publishers for all topics
        self.new_board_pub = self.create_publisher(
            String,
            'battleship/new_board',
            10
        )

        self.setup_board_pub = self.create_publisher(
            String,
            'battleship/setup_board',
            10
        )

        self.display_board_pub = self.create_publisher(
            String,
            'battleship/display_board',
            10
        )

        self.attack_pub = self.create_publisher(
            String,
            'battleship/attack',
            10
        )

        # Subscribe to responses
        self.game_status_sub = self.create_subscription(
            String,
            'battleship/game_status',
            self.game_status_callback,
            10
        )

        self.board_state_sub = self.create_subscription(
            String,
            'battleship/board_state',
            self.board_state_callback,
            10
        )

        self.attack_result_sub = self.create_subscription(
            String,
            'battleship/attack_result',
            self.attack_result_callback,
            10
        )

        self.get_logger().info('Test Publisher Node initialized')
        self.get_logger().info('Waiting 1 second for connections...')

        # Wait for connections to establish
        time.sleep(1)

    def game_status_callback(self, msg: String):
        """Handle game status messages."""
        data = json.loads(msg.data)
        self.get_logger().info(f'Game Status: {json.dumps(data, indent=2)}')

    def board_state_callback(self, msg: String):
        """Handle board state messages."""
        data = json.loads(msg.data)
        self.get_logger().info(f'Board State received for player: {data.get("player_id")}')
        self.get_logger().info(f'  Rows: {data.get("rows")}, Cols: {data.get("cols")}')
        self.get_logger().info(f'  Stats: {data.get("stats")}')

    def attack_result_callback(self, msg: String):
        """Handle attack result messages."""
        data = json.loads(msg.data)
        self.get_logger().info(f'Attack Result: {json.dumps(data, indent=2)}')

    def test_new_board(self, player_id: str = "player1", rows: int = 10, cols: int = 10):
        """
        Test creating a new board.

        Args:
            player_id: The player ID
            rows: Number of rows
            cols: Number of columns
        """
        msg = String()
        msg.data = json.dumps({
            "player_id": player_id,
            "rows": rows,
            "cols": cols
        })

        self.get_logger().info(f'Publishing new_board message: {msg.data}')
        self.new_board_pub.publish(msg)

    def test_setup_board(self, player_id: str = "player1"):
        """
        Test setting up a board with ships.

        Args:
            player_id: The player ID
        """
        msg = String()
        msg.data = json.dumps({
            "player_id": player_id,
            "ships": [
                {
                    "ship_id": "carrier",
                    "coordinates": ["A1", "A2", "A3", "A4", "A5"]
                },
                {
                    "ship_id": "battleship",
                    "coordinates": ["C1", "C2", "C3", "C4"]
                },
                {
                    "ship_id": "destroyer",
                    "coordinates": ["E5", "E6"]
                }
            ]
        })

        self.get_logger().info(f'Publishing setup_board message: {msg.data}')
        self.setup_board_pub.publish(msg)

    def test_display_board(self, player_id: str = "player1", hide_ships: bool = False):
        """
        Test displaying a board.

        Args:
            player_id: The player ID
            hide_ships: Whether to hide ships
        """
        msg = String()
        msg.data = json.dumps({
            "player_id": player_id,
            "hide_ships": hide_ships
        })

        self.get_logger().info(f'Publishing display_board message: {msg.data}')
        self.display_board_pub.publish(msg)

    def test_attack(self, attacker_id: str = "player1",
                    target_id: str = "player2", coordinate: str = "A1"):
        """
        Test attacking a coordinate.

        Args:
            attacker_id: The attacking player ID
            target_id: The target player ID
            coordinate: The coordinate to attack
        """
        msg = String()
        msg.data = json.dumps({
            "attacker_id": attacker_id,
            "target_id": target_id,
            "coordinate": coordinate
        })

        self.get_logger().info(f'Publishing attack message: {msg.data}')
        self.attack_pub.publish(msg)

    def run_full_test(self):
        """Run a complete test sequence."""
        self.get_logger().info('=== Starting Full Test Sequence ===')

        # Test 1: Create boards for two players
        self.get_logger().info('\n--- Test 1: Creating boards ---')
        self.test_new_board("player1", 10, 10)
        time.sleep(0.5)
        self.test_new_board("player2", 10, 10)
        time.sleep(1)

        # Test 2: Setup player1's board with ships
        self.get_logger().info('\n--- Test 2: Setting up player1 board ---')
        self.test_setup_board("player1")
        time.sleep(1)

        # Test 3: Setup player2's board with ships
        self.get_logger().info('\n--- Test 3: Setting up player2 board ---')
        msg = String()
        msg.data = json.dumps({
            "player_id": "player2",
            "ships": [
                {
                    "ship_id": "carrier",
                    "coordinates": ["B2", "B3", "B4", "B5", "B6"]
                },
                {
                    "ship_id": "destroyer",
                    "coordinates": ["D1", "D2"]
                }
            ]
        })
        self.setup_board_pub.publish(msg)
        time.sleep(1)

        # Test 4: Display both boards
        self.get_logger().info('\n--- Test 4: Displaying boards ---')
        self.test_display_board("player1", hide_ships=False)
        time.sleep(0.5)
        self.test_display_board("player2", hide_ships=True)
        time.sleep(1)

        # Test 5: Perform some attacks
        self.get_logger().info('\n--- Test 5: Testing attacks ---')
        self.test_attack("player1", "player2", "A1")  # Miss
        time.sleep(0.5)
        self.test_attack("player1", "player2", "B2")  # Hit
        time.sleep(0.5)
        self.test_attack("player1", "player2", "B3")  # Hit
        time.sleep(0.5)

        self.get_logger().info('\n=== Test Sequence Complete ===')


def main(args=None):
    """Main entry point for the test publisher."""
    rclpy.init(args=args)

    node = TestPublisher()

    # Create a spinner in a separate thread
    import threading
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    # Show menu
    print("\n" + "="*50)
    print("Battleship Game Test Publisher")
    print("="*50)
    print("\nOptions:")
    print("  1 - Test new_board (create board for player1)")
    print("  2 - Test setup_board (place ships for player1)")
    print("  3 - Test display_board (display player1's board)")
    print("  4 - Test attack (player1 attacks player2 at A1)")
    print("  5 - Run full test sequence")
    print("  q - Quit")
    print("="*50 + "\n")

    try:
        while rclpy.ok():
            choice = input("Enter your choice: ").strip().lower()

            if choice == '1':
                node.test_new_board("player1", 10, 10)
                time.sleep(0.5)

            elif choice == '2':
                node.test_setup_board("player1")
                time.sleep(0.5)

            elif choice == '3':
                node.test_display_board("player1", hide_ships=False)
                time.sleep(0.5)

            elif choice == '4':
                # First create player2's board if not exists
                node.test_new_board("player2", 10, 10)
                time.sleep(0.3)
                node.test_setup_board("player2")
                time.sleep(0.3)
                node.test_attack("player1", "player2", "A1")
                time.sleep(0.5)

            elif choice == '5':
                node.run_full_test()
                time.sleep(2)

            elif choice == 'q':
                break

            else:
                print("Invalid choice. Please try again.")

    except KeyboardInterrupt:
        print("\nShutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
