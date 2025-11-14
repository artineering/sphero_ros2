#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Test script for connecting to multiple Sphero robots.

This script connects to four Spheros using the multi-robot webserver:
- SB-3660 (Red LED)
- SB-74FB (Green LED)
- SB-3716 (Blue LED)
- SB-58EF (Yellow LED)

After connecting all Spheros, it sets unique LED colors for each one.
Press Ctrl+C to disconnect all Spheros and exit.
"""

import requests
import time
import signal
import sys


class MultiSpheroTester:
    """Test harness for multiple Sphero connections."""

    def __init__(self, base_url='http://localhost:5000'):
        """Initialize the tester with the multi-robot webserver URL."""
        self.base_url = base_url
        self.sphero_names = ['SB-3660', 'SB-74FB', 'SB-3716', 'SB-58EF']
        self.connected_spheros = []
        self.sphero_ports = {}  # Map sphero_name -> port
        self.running = True

        # Define unique colors for each Sphero (RGB)
        self.sphero_colors = {
            'SB-3660': (255, 0, 0),      # Red
            'SB-74FB': (0, 255, 0),      # Green
            'SB-3716': (0, 0, 255),      # Blue
            'SB-58EF': (255, 255, 0)     # Yellow
        }

    def connect_sphero(self, sphero_name: str) -> bool:
        """
        Connect to a Sphero by adding it to the multi-robot webserver.

        Args:
            sphero_name: Name of the Sphero (e.g., 'SB-3660')

        Returns:
            True if connection successful, False otherwise
        """
        try:
            print(f"➕ Connecting to {sphero_name}...")
            response = requests.post(
                f'{self.base_url}/api/spheros',
                json={'sphero_name': sphero_name},
                timeout=30
            )

            if response.status_code in [200, 201]:
                result = response.json()
                if result.get('success'):
                    instance = result.get('instance', {})
                    port = instance.get('port', 'unknown')
                    url = instance.get('url', 'unknown')
                    print(f"✓ {sphero_name} connected successfully")
                    print(f"  Port: {port}")
                    print(f"  URL: {url}")
                    self.connected_spheros.append(sphero_name)
                    self.sphero_ports[sphero_name] = port
                    return True
                else:
                    print(f"✗ Failed to connect to {sphero_name}: {result.get('message', 'Unknown error')}")
                    return False
            else:
                print(f"✗ Failed to connect to {sphero_name}: HTTP {response.status_code}")
                return False

        except requests.exceptions.ConnectionError:
            print(f"✗ Cannot connect to multi-robot webserver at {self.base_url}")
            print("  Make sure the server is running: ros2 run multirobot_webserver multirobot_webapp.py")
            return False
        except Exception as e:
            print(f"✗ Error connecting to {sphero_name}: {e}")
            return False

    def disconnect_sphero(self, sphero_name: str) -> bool:
        """
        Disconnect from a Sphero by removing it from the multi-robot webserver.

        Args:
            sphero_name: Name of the Sphero

        Returns:
            True if disconnection successful, False otherwise
        """
        try:
            print(f"➖ Disconnecting {sphero_name}...")
            response = requests.delete(
                f'{self.base_url}/api/spheros/{sphero_name}',
                timeout=15
            )

            if response.status_code == 200:
                result = response.json()
                if result.get('success'):
                    print(f"✓ {sphero_name} disconnected")
                    return True
                else:
                    print(f"⚠️  {sphero_name} disconnect returned: {result.get('message', 'Unknown')}")
                    return False
            else:
                print(f"⚠️  Failed to disconnect {sphero_name}: HTTP {response.status_code}")
                return False

        except Exception as e:
            print(f"⚠️  Error disconnecting {sphero_name}: {e}")
            return False

    def set_led_color(self, sphero_name: str, red: int, green: int, blue: int) -> bool:
        """
        Set the LED color for a Sphero.

        Args:
            sphero_name: Name of the Sphero
            red: Red value (0-255)
            green: Green value (0-255)
            blue: Blue value (0-255)

        Returns:
            True if successful, False otherwise
        """
        if sphero_name not in self.sphero_ports:
            print(f"⚠️  {sphero_name} is not connected")
            return False

        port = self.sphero_ports[sphero_name]
        try:
            response = requests.post(
                f'http://localhost:{port}/api/led',
                json={
                    'red': red,
                    'green': green,
                    'blue': blue,
                    'type': 'main'
                },
                timeout=5
            )

            if response.status_code == 200:
                print(f"💡 Set {sphero_name} LED to RGB({red}, {green}, {blue})")
                return True
            else:
                print(f"⚠️  Failed to set LED for {sphero_name}: HTTP {response.status_code}")
                return False

        except Exception as e:
            print(f"⚠️  Error setting LED for {sphero_name}: {e}")
            return False

    def set_all_led_colors(self):
        """Set unique LED colors for all connected Spheros."""
        print()
        print("="*60)
        print("Setting Unique LED Colors")
        print("="*60)
        print()

        for sphero_name in self.connected_spheros:
            if sphero_name in self.sphero_colors:
                r, g, b = self.sphero_colors[sphero_name]
                color_name = self._get_color_name(r, g, b)
                print(f"Setting {sphero_name} to {color_name}...")
                self.set_led_color(sphero_name, r, g, b)
                time.sleep(0.5)

        print()
        print("="*60)
        print("All LEDs configured")
        print("="*60)
        print()

    def _get_color_name(self, r: int, g: int, b: int) -> str:
        """Get human-readable color name from RGB values."""
        if r == 255 and g == 0 and b == 0:
            return "Red"
        elif r == 0 and g == 255 and b == 0:
            return "Green"
        elif r == 0 and g == 0 and b == 255:
            return "Blue"
        elif r == 255 and g == 255 and b == 0:
            return "Yellow"
        elif r == 255 and g == 0 and b == 255:
            return "Magenta"
        elif r == 0 and g == 255 and b == 255:
            return "Cyan"
        elif r == 255 and g == 255 and b == 255:
            return "White"
        else:
            return f"RGB({r}, {g}, {b})"

    def get_status(self) -> dict:
        """
        Get status of all connected Spheros.

        Returns:
            Dictionary with status information
        """
        try:
            response = requests.get(f'{self.base_url}/api/spheros', timeout=5)
            if response.status_code == 200:
                return response.json()
            else:
                return {'success': False, 'spheros': [], 'count': 0}
        except Exception as e:
            print(f"⚠️  Error getting status: {e}")
            return {'success': False, 'spheros': [], 'count': 0}

    def connect_all(self):
        """Connect to all Spheros in the list."""
        print("="*60)
        print("Multi-Sphero Connection Test")
        print("="*60)
        print(f"Connecting to {len(self.sphero_names)} Spheros...")
        print()

        for sphero_name in self.sphero_names:
            success = self.connect_sphero(sphero_name)
            if success:
                time.sleep(10)  # Brief delay between connections
            print()

        print("="*60)
        print(f"Connection Summary: {len(self.connected_spheros)}/{len(self.sphero_names)} Spheros connected")
        print("="*60)
        print()

        if self.connected_spheros:
            print("✓ Connected Spheros:")
            for name in self.connected_spheros:
                print(f"  - {name}")

            # Set unique LED colors for each Sphero
            self.set_all_led_colors()
        else:
            print("✗ No Spheros connected")

        print()

    def disconnect_all(self):
        """Disconnect from all connected Spheros."""
        print()
        print("="*60)
        print("Disconnecting all Spheros...")
        print("="*60)
        print()

        # Disconnect in reverse order
        for sphero_name in reversed(self.connected_spheros[:]):
            self.disconnect_sphero(sphero_name)
            self.connected_spheros.remove(sphero_name)
            time.sleep(0.5)

        print()
        print("="*60)
        print("All Spheros disconnected")
        print("="*60)

    def run(self):
        """Main run loop - connect and wait for interrupt."""
        # Set up signal handler for clean shutdown
        def signal_handler(sig, frame):
            print("\n\nInterrupt received - shutting down...")
            self.running = False

        signal.signal(signal.SIGINT, signal_handler)

        # Connect to all Spheros
        self.connect_all()

        if not self.connected_spheros:
            print("\n⚠️  No Spheros connected. Exiting.")
            return

        # Wait for interrupt
        print("\nPress Ctrl+C to disconnect all Spheros and exit...")
        print()

        # Monitor status while waiting
        try:
            while self.running:
                time.sleep(5)  # Check every 5 seconds

                # Get current status
                status = self.get_status()
                if status.get('success'):
                    current_count = status.get('count', 0)
                    if current_count != len(self.connected_spheros):
                        print(f"⚠️  Status change detected: {current_count} Spheros active (expected {len(self.connected_spheros)})")

        except KeyboardInterrupt:
            print("\n\nKeyboard interrupt - shutting down...")

        # Disconnect all
        self.disconnect_all()


def main():
    """Main entry point."""
    # Check if multi-robot webserver is running
    try:
        response = requests.get('http://localhost:5000/health', timeout=2)
        if response.status_code != 200:
            print("✗ Multi-robot webserver is not responding correctly")
            print("  Start it with: ros2 run multirobot_webserver multirobot_webapp.py")
            sys.exit(1)
    except requests.exceptions.ConnectionError:
        print("✗ Cannot connect to multi-robot webserver at http://localhost:5000")
        print("  Start it with: ros2 run multirobot_webserver multirobot_webapp.py")
        sys.exit(1)

    # Create and run tester
    tester = MultiSpheroTester()
    tester.run()

    print("\nGoodbye!")


if __name__ == '__main__':
    main()
