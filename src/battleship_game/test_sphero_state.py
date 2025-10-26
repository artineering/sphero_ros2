#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Test script for SpheroState get/set API.

This demonstrates the usage of the get() and set() methods for accessing
and modifying Sphero state properties.
"""

from battleship_game.core.sphero.state import SpheroState, SpheroConnectionState


def main():
    # Create a new SpheroState instance
    state = SpheroState(toy_name="SB-TEST")

    print("=" * 60)
    print("SpheroState Get/Set API Test")
    print("=" * 60)

    # Test getting top-level properties
    print("\n1. Getting top-level properties:")
    print(f"   toy_name: {state.get('toy_name')}")
    print(f"   connection_state: {state.get('connection_state')}")
    print(f"   orientation object: {state.get('orientation')}")

    # Test getting sub-properties
    print("\n2. Getting sub-properties:")
    print(f"   orientation.pitch: {state.get('orientation', 'pitch')}")
    print(f"   orientation.roll: {state.get('orientation', 'roll')}")
    print(f"   orientation.yaw: {state.get('orientation', 'yaw')}")
    print(f"   motion.heading: {state.get('motion', 'heading')}")
    print(f"   motion.speed: {state.get('motion', 'speed')}")
    print(f"   led.red: {state.get('led', 'red')}")
    print(f"   led.green: {state.get('led', 'green')}")
    print(f"   led.blue: {state.get('led', 'blue')}")

    # Test setting sub-properties
    print("\n3. Setting sub-properties:")
    state.set("orientation", 45.5, "pitch")
    state.set("orientation", 30.0, "roll")
    state.set("orientation", 90.0, "yaw")
    state.set("motion", 180, "heading")
    state.set("motion", 128, "speed")
    state.set("led", 255, "red")
    state.set("led", 128, "green")
    state.set("led", 0, "blue")

    print(f"   Set orientation.pitch to 45.5: {state.get('orientation', 'pitch')}")
    print(f"   Set orientation.roll to 30.0: {state.get('orientation', 'roll')}")
    print(f"   Set orientation.yaw to 90.0: {state.get('orientation', 'yaw')}")
    print(f"   Set motion.heading to 180: {state.get('motion', 'heading')}")
    print(f"   Set motion.speed to 128: {state.get('motion', 'speed')}")
    print(f"   Set led.red to 255: {state.get('led', 'red')}")
    print(f"   Set led.green to 128: {state.get('led', 'green')}")
    print(f"   Set led.blue to 0: {state.get('led', 'blue')}")

    # Test setting top-level properties
    print("\n4. Setting top-level properties:")
    state.set("toy_name", "SB-1234")
    state.set("connection_state", SpheroConnectionState.CONNECTED)
    print(f"   Set toy_name to 'SB-1234': {state.get('toy_name')}")
    print(f"   Set connection_state: {state.get('connection_state')}")

    # Test position and velocity
    print("\n5. Position and velocity:")
    state.set("position", 10.5, "x")
    state.set("position", 20.3, "y")
    state.set("velocity", 5.2, "x")
    state.set("velocity", 3.1, "y")
    print(f"   position.x: {state.get('position', 'x')}")
    print(f"   position.y: {state.get('position', 'y')}")
    print(f"   velocity.x: {state.get('velocity', 'x')}")
    print(f"   velocity.y: {state.get('velocity', 'y')}")

    # Test accelerometer and gyroscope
    print("\n6. Accelerometer and gyroscope:")
    state.set("accelerometer", 0.5, "x")
    state.set("accelerometer", -0.3, "y")
    state.set("accelerometer", 1.0, "z")
    state.set("gyroscope", 10.0, "x")
    state.set("gyroscope", -5.0, "y")
    state.set("gyroscope", 2.5, "z")
    print(f"   accelerometer.x: {state.get('accelerometer', 'x')}")
    print(f"   accelerometer.y: {state.get('accelerometer', 'y')}")
    print(f"   accelerometer.z: {state.get('accelerometer', 'z')}")
    print(f"   gyroscope.x: {state.get('gyroscope', 'x')}")
    print(f"   gyroscope.y: {state.get('gyroscope', 'y')}")
    print(f"   gyroscope.z: {state.get('gyroscope', 'z')}")

    # Test battery
    print("\n7. Battery:")
    state.set("battery", 85, "percentage")
    print(f"   battery.percentage: {state.get('battery', 'percentage')}")

    # Test default values
    print("\n8. Testing default values for invalid properties:")
    print(f"   invalid property: {state.get('invalid_prop', default='NOT_FOUND')}")
    print(f"   invalid subprop: {state.get('orientation', 'invalid', default=-999)}")

    # Test full state
    print("\n9. Complete state:")
    print(f"   {state}")

    print("\n" + "=" * 60)
    print("Test completed successfully!")
    print("=" * 60)


if __name__ == '__main__':
    main()
