#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Sphero Agent Node using OpenAI Swarm.

This node provides voice/text command control for the Sphero robot using
the OpenAI Swarm framework for agentic AI.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import os
from typing import Dict, Any
from swarm import Swarm, Agent
import speech_recognition as sr

# Suppress ALSA/JACK warnings
import warnings
warnings.filterwarnings('ignore', category=DeprecationWarning)

# Redirect ALSA error messages to /dev/null
import sys
from contextlib import contextmanager

@contextmanager
def suppress_stderr():
    """Suppress stderr output temporarily."""
    original_stderr = sys.stderr
    try:
        sys.stderr = open(os.devnull, 'w')
        yield
    finally:
        sys.stderr.close()
        sys.stderr = original_stderr


class SpheroAgentNode(Node):
    """
    OpenAI Swarm-based agent node for controlling Sphero via voice/text commands.

    The agent converts natural language input into Sphero control commands
    and publishes them to the sphero_controller_node.
    """

    def __init__(self):
        """Initialize the Sphero agent node."""
        super().__init__('sphero_agent_node')

        # Get OpenAI API key
        self.api_key = os.getenv('OPENAI_API_KEY')
        if not self.api_key:
            self.get_logger().error('OPENAI_API_KEY environment variable not set!')
            raise ValueError('OPENAI_API_KEY required')

        # Initialize Swarm client
        self.client = Swarm()

        # ROS2 Publishers for Sphero commands
        self.sphero_pubs = {
            'led': self.create_publisher(String, 'sphero/led', 10),
            'roll': self.create_publisher(String, 'sphero/roll', 10),
            'spin': self.create_publisher(String, 'sphero/spin', 10),
            'heading': self.create_publisher(String, 'sphero/heading', 10),
            'speed': self.create_publisher(String, 'sphero/speed', 10),
            'stop': self.create_publisher(String, 'sphero/stop', 10),
            'matrix': self.create_publisher(String, 'sphero/matrix', 10),
        }

        # ROS2 Publishers for agent status
        self.response_pub = self.create_publisher(
            String,
            'sphero_agent/response',
            10
        )

        self.status_pub = self.create_publisher(
            String,
            'sphero_agent/status',
            10
        )

        # Create the Swarm agent with Sphero control functions
        self.agent = Agent(
            name="Sphero Controller",
            instructions="""You are a helpful assistant that controls a Sphero robot.

Your role is to:
1. Understand natural language commands from the user
2. Convert them into specific Sphero control actions
3. Execute the appropriate control functions

Available actions:
- LED color control (RGB values)
- Movement (roll, spin, heading, speed)
- Stop movement
- Matrix display (for BOLT)

Be concise and confirm actions clearly.""",
            functions=[
                self.set_led_color,
                self.roll_sphero,
                self.spin_sphero,
                self.set_heading,
                self.set_speed,
                self.stop_sphero,
                self.display_matrix_pattern,
            ]
        )

        # Initialize speech recognizer (won't work without audio, will fallback to text)
        self.recognizer = sr.Recognizer()
        self.audio_available = self._check_audio_available()

        self.get_logger().info('Sphero Agent Node initialized')
        self.get_logger().info(f'Audio input available: {self.audio_available}')
        self.get_logger().info('Agent ready to accept commands')

        # Publish ready status
        status_msg = String()
        status_msg.data = json.dumps({'status': 'ready', 'audio_available': self.audio_available})
        self.status_pub.publish(status_msg)

    def _check_audio_available(self) -> bool:
        """Check if audio input is available."""
        try:
            # Suppress ALSA/JACK errors when checking for microphones
            with suppress_stderr():
                sr.Microphone.list_microphone_names()
            return True
        except Exception as e:
            self.get_logger().warning(f'Audio not available: {str(e)}')
            return False

    # ===== Sphero Control Functions for the Agent =====

    def set_led_color(self, red: int, green: int, blue: int) -> str:
        """
        Set the Sphero's LED color.

        Args:
            red: Red value (0-255)
            green: Green value (0-255)
            blue: Blue value (0-255)

        Returns:
            Confirmation message
        """
        red = max(0, min(255, red))
        green = max(0, min(255, green))
        blue = max(0, min(255, blue))

        msg = String()
        msg.data = json.dumps({'red': red, 'green': green, 'blue': blue})
        self.sphero_pubs['led'].publish(msg)

        self.get_logger().info(f'LED set to RGB({red}, {green}, {blue})')
        return f"LED set to red={red}, green={green}, blue={blue}"

    def roll_sphero(self, heading: int, speed: int, duration: float = 0.0) -> str:
        """
        Make the Sphero roll.

        Args:
            heading: Direction in degrees (0-359)
            speed: Speed (0-255)
            duration: Duration in seconds (0 = continuous)

        Returns:
            Confirmation message
        """
        heading = heading % 360
        speed = max(0, min(255, speed))

        msg = String()
        msg.data = json.dumps({
            'heading': heading,
            'speed': speed,
            'duration': duration
        })
        self.sphero_pubs['roll'].publish(msg)

        self.get_logger().info(f'Rolling at {heading}deg, speed {speed}')
        return f"Rolling at {heading} degrees with speed {speed}"

    def spin_sphero(self, angle: int, duration: float = 1.0) -> str:
        """
        Spin the Sphero.

        Args:
            angle: Degrees to spin (positive = clockwise)
            duration: Duration in seconds

        Returns:
            Confirmation message
        """
        msg = String()
        msg.data = json.dumps({'angle': angle, 'duration': duration})
        self.sphero_pubs['spin'].publish(msg)

        direction = "clockwise" if angle > 0 else "counterclockwise"
        self.get_logger().info(f'Spinning {abs(angle)}deg {direction}')
        return f"Spinning {abs(angle)} degrees {direction}"

    def set_heading(self, heading: int) -> str:
        """
        Set Sphero's heading.

        Args:
            heading: Direction in degrees (0-359)

        Returns:
            Confirmation message
        """
        heading = heading % 360

        msg = String()
        msg.data = json.dumps({'heading': heading})
        self.sphero_pubs['heading'].publish(msg)

        self.get_logger().info(f'Heading set to {heading}deg')
        return f"Heading set to {heading} degrees"

    def set_speed(self, speed: int, duration: float = 0.0) -> str:
        """
        Set Sphero's speed.

        Args:
            speed: Speed (0-255)
            duration: Duration in seconds (0 = continuous)

        Returns:
            Confirmation message
        """
        speed = max(0, min(255, speed))

        msg = String()
        msg.data = json.dumps({'speed': speed, 'duration': duration})
        self.sphero_pubs['speed'].publish(msg)

        self.get_logger().info(f'Speed set to {speed}')
        return f"Speed set to {speed}"

    def stop_sphero(self) -> str:
        """
        Stop the Sphero.

        Returns:
            Confirmation message
        """
        msg = String()
        msg.data = json.dumps({})
        self.sphero_pubs['stop'].publish(msg)

        self.get_logger().info('Sphero stopped')
        return "Sphero stopped"

    def display_matrix_pattern(self, pattern: str, red: int = 255,
                               green: int = 255, blue: int = 255,
                               duration: float = 2.0) -> str:
        """
        Display a pattern on Sphero BOLT's LED matrix.

        Args:
            pattern: Pattern name (smile, cross, arrow_up)
            red: Red value (0-255)
            green: Green value (0-255)
            blue: Blue value (0-255)
            duration: Display duration in seconds

        Returns:
            Confirmation message
        """
        valid_patterns = ['smile', 'cross', 'arrow_up']
        if pattern not in valid_patterns:
            return f"Invalid pattern. Valid: {', '.join(valid_patterns)}"

        msg = String()
        msg.data = json.dumps({
            'pattern': pattern,
            'red': red,
            'green': green,
            'blue': blue,
            'duration': duration
        })
        self.sphero_pubs['matrix'].publish(msg)

        self.get_logger().info(f'Displaying {pattern} pattern')
        return f"Displaying {pattern} pattern"

    # ===== Input Handling =====

    def listen_for_command(self) -> str:
        """
        Listen for voice command (or fall back to text input).

        Returns:
            The transcribed command text
        """
        if self.audio_available:
            try:
                # Suppress ALSA/JACK errors when opening microphone
                with suppress_stderr():
                    with sr.Microphone() as source:
                        self.get_logger().info('Listening...')
                        audio = self.recognizer.listen(source, timeout=5)

                try:
                    command = self.recognizer.recognize_google(audio)
                    self.get_logger().info(f'Heard: {command}')
                    return command
                except sr.UnknownValueError:
                    self.get_logger().warning('Could not understand audio')
                    return ""
                except sr.RequestError as e:
                    self.get_logger().error(f'Speech recognition error: {e}')
                    return ""

            except Exception as e:
                self.get_logger().error(f'Audio error: {e}')
                return ""
        else:
            # Fallback to text input
            return input("Enter command: ")

    def process_command(self, command: str):
        """
        Process a command using the Swarm agent.

        Args:
            command: The natural language command
        """
        if not command or not command.strip():
            return

        self.get_logger().info(f'Processing: {command}')

        try:
            # Run the agent with the command
            response = self.client.run(
                agent=self.agent,
                messages=[{"role": "user", "content": command}]
            )

            # Get the response text
            response_text = ""
            if response.messages:
                last_message = response.messages[-1]
                response_text = last_message.get('content', '')

            self.get_logger().info(f'Agent: {response_text}')

            # Publish response
            response_msg = String()
            response_msg.data = json.dumps({
                'command': command,
                'response': response_text
            })
            self.response_pub.publish(response_msg)

        except Exception as e:
            self.get_logger().error(f'Error processing command: {str(e)}')
            import traceback
            self.get_logger().error(traceback.format_exc())

    def run_interactive_loop(self):
        """Run interactive command loop."""
        self.get_logger().info('Starting interactive mode')
        self.get_logger().info('Say or type commands like:')
        self.get_logger().info('  - "Turn the LED red"')
        self.get_logger().info('  - "Move forward at speed 100"')
        self.get_logger().info('  - "Spin around"')
        self.get_logger().info('  - "Stop"')
        self.get_logger().info('Type "quit" to exit\n')

        while rclpy.ok():
            try:
                command = self.listen_for_command()

                if command.lower() in ['quit', 'exit', 'q']:
                    self.get_logger().info('Exiting...')
                    break

                if command:
                    self.process_command(command)

            except KeyboardInterrupt:
                break
            except Exception as e:
                self.get_logger().error(f'Error: {str(e)}')


def main(args=None):
    """Main entry point for the Sphero agent node."""
    rclpy.init(args=args)

    try:
        node = SpheroAgentNode()

        # Run interactive loop in a separate thread
        import threading
        interactive_thread = threading.Thread(
            target=node.run_interactive_loop,
            daemon=True
        )
        interactive_thread.start()

        # Spin the ROS node
        rclpy.spin(node)

    except KeyboardInterrupt:
        print("\nShutting down...")
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
