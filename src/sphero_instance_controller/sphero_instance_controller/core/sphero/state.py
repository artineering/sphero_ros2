#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Sphero State Data Class.

This module defines the data classes that represent the current state
of a Sphero robot, including sensor data, motion state, LED state, and more.
"""

from dataclasses import dataclass, field
from typing import Optional, Tuple
from enum import Enum
import time

try:
    from sensor_msgs.msg import BatteryState as BatteryStateMsg
    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False
    BatteryStateMsg = None


class SpheroConnectionState(Enum):
    """Enumeration of possible Sphero connection states."""
    DISCONNECTED = "disconnected"
    CONNECTING = "connecting"
    CONNECTED = "connected"
    ERROR = "error"


@dataclass
class SpheroOrientation:
    """Represents the orientation of the Sphero in 3D space."""
    pitch: float = 0.0  # Rotation around X-axis (degrees)
    roll: float = 0.0   # Rotation around Y-axis (degrees)
    yaw: float = 0.0    # Rotation around Z-axis (degrees)

    def to_dict(self) -> dict:
        """Convert to dictionary representation."""
        return {
            'pitch': self.pitch,
            'roll': self.roll,
            'yaw': self.yaw
        }


@dataclass
class SpheroAccelerometer:
    """Represents accelerometer data from the Sphero."""
    x: float = 0.0  # Acceleration in X direction (Gs)
    y: float = 0.0  # Acceleration in Y direction (Gs)
    z: float = 0.0  # Acceleration in Z direction (Gs)

    def to_dict(self) -> dict:
        """Convert to dictionary representation."""
        return {
            'x': self.x,
            'y': self.y,
            'z': self.z
        }


@dataclass
class SpheroGyroscope:
    """Represents gyroscope data from the Sphero."""
    x: float = 0.0  # Angular velocity around X-axis (degrees/sec)
    y: float = 0.0  # Angular velocity around Y-axis (degrees/sec)
    z: float = 0.0  # Angular velocity around Z-axis (degrees/sec)

    def to_dict(self) -> dict:
        """Convert to dictionary representation."""
        return {
            'x': self.x,
            'y': self.y,
            'z': self.z
        }


@dataclass
class SpheroPosition:
    """Represents the position of the Sphero in 2D space."""
    x: float = 0.0  # X coordinate (meters or arbitrary units)
    y: float = 0.0  # Y coordinate (meters or arbitrary units)

    def to_dict(self) -> dict:
        """Convert to dictionary representation."""
        return {
            'x': self.x,
            'y': self.y
        }


@dataclass
class SpheroVelocity:
    """Represents the velocity of the Sphero."""
    x: float = 0.0  # Velocity in X direction (units/sec)
    y: float = 0.0  # Velocity in Y direction (units/sec)

    def to_dict(self) -> dict:
        """Convert to dictionary representation."""
        return {
            'x': self.x,
            'y': self.y
        }


@dataclass
class SpheroLEDState:
    """Represents the current LED color state."""
    red: int = 0    # Red channel (0-255)
    green: int = 0  # Green channel (0-255)
    blue: int = 0   # Blue channel (0-255)

    def __post_init__(self):
        """Validate RGB values are in range 0-255."""
        self.red = max(0, min(255, self.red))
        self.green = max(0, min(255, self.green))
        self.blue = max(0, min(255, self.blue))

    def to_tuple(self) -> Tuple[int, int, int]:
        """Convert to RGB tuple."""
        return (self.red, self.green, self.blue)

    def to_dict(self) -> dict:
        """Convert to dictionary representation."""
        return {
            'red': self.red,
            'green': self.green,
            'blue': self.blue
        }


@dataclass
class SpheroMotionState:
    """Represents the current motion state of the Sphero."""
    heading: int = 0         # Current heading in degrees (0-359)
    speed: int = 0           # Current speed (0-255)
    is_moving: bool = False  # Whether the Sphero is currently moving

    def __post_init__(self):
        """Validate heading and speed values."""
        self.heading = self.heading % 360
        self.speed = max(0, min(255, self.speed))

    def to_dict(self) -> dict:
        """Convert to dictionary representation."""
        return {
            'heading': self.heading,
            'speed': self.speed,
            'is_moving': self.is_moving
        }


@dataclass
class SpheroBatteryState:
    """Represents the battery state of the Sphero."""
    percentage: int = 100  # Battery percentage (0-100)
    voltage: Optional[float] = None  # Battery voltage if available
    current: Optional[float] = None  # Battery current in Amperes if available
    charge: Optional[float] = None  # Battery charge in Ah if available
    capacity: Optional[float] = None  # Battery capacity in Ah if available
    temperature: Optional[float] = None  # Battery temperature in Celsius if available

    def __post_init__(self):
        """Validate battery percentage."""
        self.percentage = max(0, min(100, self.percentage))

    def to_dict(self) -> dict:
        """Convert to dictionary representation."""
        result = {'percentage': self.percentage}
        if self.voltage is not None:
            result['voltage'] = self.voltage
        if self.current is not None:
            result['current'] = self.current
        if self.charge is not None:
            result['charge'] = self.charge
        if self.capacity is not None:
            result['capacity'] = self.capacity
        if self.temperature is not None:
            result['temperature'] = self.temperature
        return result

    def to_battery_state_msg(self) -> Optional['BatteryStateMsg']:
        """
        Convert to ROS2 sensor_msgs/BatteryState message.

        Returns:
            BatteryState message or None if ROS2 is not available
        """
        if not ROS2_AVAILABLE or BatteryStateMsg is None:
            return None

        msg = BatteryStateMsg()

        # Set voltage (float) - use NaN if not available
        msg.voltage = self.voltage if self.voltage is not None else float('nan')

        # Set temperature (float) - use NaN if not available
        msg.temperature = self.temperature if self.temperature is not None else float('nan')

        # Set current (float) - use NaN if not available
        msg.current = self.current if self.current is not None else float('nan')

        # Set charge (float) - use NaN if not available
        msg.charge = self.charge if self.charge is not None else float('nan')

        # Set capacity (float) - use NaN if not available
        msg.capacity = self.capacity if self.capacity is not None else float('nan')

        # Set design capacity (float) - use NaN if not available
        msg.design_capacity = float('nan')

        # Set percentage (float 0.0-1.0)
        msg.percentage = self.percentage / 100.0

        # Set power supply status
        # POWER_SUPPLY_STATUS_UNKNOWN = 0
        # POWER_SUPPLY_STATUS_CHARGING = 1
        # POWER_SUPPLY_STATUS_DISCHARGING = 2
        # POWER_SUPPLY_STATUS_NOT_CHARGING = 3
        # POWER_SUPPLY_STATUS_FULL = 4
        if self.percentage >= 100:
            msg.power_supply_status = BatteryStateMsg.POWER_SUPPLY_STATUS_FULL
        else:
            msg.power_supply_status = BatteryStateMsg.POWER_SUPPLY_STATUS_DISCHARGING

        # Set power supply health
        # POWER_SUPPLY_HEALTH_UNKNOWN = 0
        # POWER_SUPPLY_HEALTH_GOOD = 1
        # POWER_SUPPLY_HEALTH_OVERHEAT = 2
        # POWER_SUPPLY_HEALTH_DEAD = 3
        # POWER_SUPPLY_HEALTH_OVERVOLTAGE = 4
        # POWER_SUPPLY_HEALTH_UNSPEC_FAILURE = 5
        # POWER_SUPPLY_HEALTH_COLD = 6
        # POWER_SUPPLY_HEALTH_WATCHDOG_TIMER_EXPIRE = 7
        # POWER_SUPPLY_HEALTH_SAFETY_TIMER_EXPIRE = 8
        if self.percentage < 10:
            msg.power_supply_health = BatteryStateMsg.POWER_SUPPLY_HEALTH_DEAD
        else:
            msg.power_supply_health = BatteryStateMsg.POWER_SUPPLY_HEALTH_GOOD

        # Set power supply technology
        # POWER_SUPPLY_TECHNOLOGY_UNKNOWN = 0
        # POWER_SUPPLY_TECHNOLOGY_NIMH = 1
        # POWER_SUPPLY_TECHNOLOGY_LION = 2
        # POWER_SUPPLY_TECHNOLOGY_LIPO = 3
        # POWER_SUPPLY_TECHNOLOGY_LIFE = 4
        # POWER_SUPPLY_TECHNOLOGY_NICD = 5
        # POWER_SUPPLY_TECHNOLOGY_LIMN = 6
        msg.power_supply_technology = BatteryStateMsg.POWER_SUPPLY_TECHNOLOGY_LIPO

        # Set present flag
        msg.present = True

        # Cell voltages - leave empty for Sphero
        msg.cell_voltage = []

        # Cell temperatures - leave empty for Sphero
        msg.cell_temperature = []

        # Location - empty string for Sphero
        msg.location = "internal"

        # Serial number - empty string
        msg.serial_number = ""

        return msg


@dataclass
class SpheroMatrixState:
    """Represents the LED matrix state (for Sphero BOLT)."""
    pattern: Optional[str] = None  # Current pattern name
    matrix_data: Optional[list] = None  # 64-element list for 8x8 matrix
    color: SpheroLEDState = field(default_factory=SpheroLEDState)
    is_active: bool = False

    def to_dict(self) -> dict:
        """Convert to dictionary representation."""
        return {
            'pattern': self.pattern,
            'matrix_data': self.matrix_data,
            'color': self.color.to_dict(),
            'is_active': self.is_active
        }


@dataclass
class SpheroState:
    """
    Complete state representation of a Sphero robot.

    This data class encapsulates all sensor data, motion state, LED state,
    battery information, and connection status for a Sphero toy.
    """

    # Identification
    toy_name: str = "Unknown"
    toy_type: Optional[str] = None  # e.g., "Sphero BOLT", "Sphero Mini"

    # Connection
    connection_state: SpheroConnectionState = SpheroConnectionState.DISCONNECTED

    # External localization mode
    external_localization: bool = False

    # Motion and Position
    motion: SpheroMotionState = field(default_factory=SpheroMotionState)
    position: SpheroPosition = field(default_factory=SpheroPosition)
    velocity: SpheroVelocity = field(default_factory=SpheroVelocity)

    # Sensors
    orientation: SpheroOrientation = field(default_factory=SpheroOrientation)
    accelerometer: SpheroAccelerometer = field(default_factory=SpheroAccelerometer)
    gyroscope: SpheroGyroscope = field(default_factory=SpheroGyroscope)

    # LED State
    led: SpheroLEDState = field(default_factory=SpheroLEDState)
    matrix: Optional[SpheroMatrixState] = None  # Only for BOLT

    # Battery
    battery: SpheroBatteryState = field(default_factory=SpheroBatteryState)

    # Timing
    timestamp: float = field(default_factory=time.time)
    last_update: float = field(default_factory=time.time)

    # API handle (optional, for updating from device)
    _api: Optional[object] = field(default=None, repr=False)

    # Toy object (optional, for direct command access)
    _toy: Optional[object] = field(default=None, repr=False)

    # Position change tracking (for noise reduction)
    _last_published_position: Optional[Tuple[float, float]] = field(default=None, repr=False)
    _position_change_threshold: float = field(default=1.0, repr=False)  # cm

    # Position offset (for reset_aim functionality)
    _position_offset_x: float = field(default=0.0, repr=False)
    _position_offset_y: float = field(default=0.0, repr=False)

    def set_api(self, api):
        """Set the Sphero API handle for device queries."""
        self._api = api

    def set_toy(self, toy):
        """Set the Sphero toy object for direct command access."""
        self._toy = toy

    def update_timestamp(self):
        """Update the last_update timestamp to current time."""
        self.last_update = time.time()

    def set_external_location(self, x: float, y: float):
        # Apply position offset (for reset_aim functionality)
        # This makes position relative to our coordinate system origin
        new_x = x - self._position_offset_x
        new_y = y - self._position_offset_y

        # Only update position if change exceeds threshold (reduces noise)
        if self._should_update_position(new_x, new_y):
            self.set('position', new_x, 'x')
            self.set('position', new_y, 'y')
            self._last_published_position = (new_x, new_y)

    def _should_update_position(self, new_x: float, new_y: float) -> bool:
        """
        Check if position should be updated based on change threshold.

        Args:
            new_x: New X position
            new_y: New Y position

        Returns:
            bool: True if position should be updated, False otherwise
        """
        # Always update if no previous position
        if self._last_published_position is None:
            return True

        # Calculate distance from last published position
        last_x, last_y = self._last_published_position
        distance = ((new_x - last_x) ** 2 + (new_y - last_y) ** 2) ** 0.5

        # Update if distance exceeds threshold
        return distance >= self._position_change_threshold

    def _get_orientation(self, subproperty: Optional[str] = None):
        """Get orientation property or sub-property."""
        if subproperty is None:
            return self.orientation
        return getattr(self.orientation, subproperty, None)

    def _set_orientation(self, subproperty: str, value) -> bool:
        """Set orientation sub-property."""
        if hasattr(self.orientation, subproperty):
            setattr(self.orientation, subproperty, value)
            self.update_timestamp()
            return True
        return False

    def _get_accelerometer(self, subproperty: Optional[str] = None):
        """Get accelerometer property or sub-property."""
        if subproperty is None:
            return self.accelerometer
        return getattr(self.accelerometer, subproperty, None)

    def _set_accelerometer(self, subproperty: str, value) -> bool:
        """Set accelerometer sub-property."""
        if hasattr(self.accelerometer, subproperty):
            setattr(self.accelerometer, subproperty, value)
            self.update_timestamp()
            return True
        return False

    def _get_gyroscope(self, subproperty: Optional[str] = None):
        """Get gyroscope property or sub-property."""
        if subproperty is None:
            return self.gyroscope
        return getattr(self.gyroscope, subproperty, None)

    def _set_gyroscope(self, subproperty: str, value) -> bool:
        """Set gyroscope sub-property."""
        if hasattr(self.gyroscope, subproperty):
            setattr(self.gyroscope, subproperty, value)
            self.update_timestamp()
            return True
        return False

    def _get_position(self, subproperty: Optional[str] = None):
        """Get position property or sub-property."""
        if subproperty is None:
            return self.position
        return getattr(self.position, subproperty, None)

    def _set_position(self, subproperty: str, value) -> bool:
        """Set position sub-property."""
        if hasattr(self.position, subproperty):
            setattr(self.position, subproperty, value)
            self.update_timestamp()
            return True
        return False

    def _get_velocity(self, subproperty: Optional[str] = None):
        """Get velocity property or sub-property."""
        if subproperty is None:
            return self.velocity
        return getattr(self.velocity, subproperty, None)

    def _set_velocity(self, subproperty: str, value) -> bool:
        """Set velocity sub-property."""
        if hasattr(self.velocity, subproperty):
            setattr(self.velocity, subproperty, value)
            self.update_timestamp()
            return True
        return False

    def _get_motion(self, subproperty: Optional[str] = None):
        """Get motion property or sub-property."""
        if subproperty is None:
            return self.motion
        return getattr(self.motion, subproperty, None)

    def _set_motion(self, subproperty: str, value) -> bool:
        """Set motion sub-property."""
        if hasattr(self.motion, subproperty):
            setattr(self.motion, subproperty, value)
            self.update_timestamp()
            return True
        return False

    def _get_led(self, subproperty: Optional[str] = None):
        """Get LED property or sub-property."""
        if subproperty is None:
            return self.led
        return getattr(self.led, subproperty, None)

    def _set_led(self, subproperty: str, value) -> bool:
        """Set LED sub-property."""
        if hasattr(self.led, subproperty):
            setattr(self.led, subproperty, value)
            self.update_timestamp()
            return True
        return False

    def _get_battery(self, subproperty: Optional[str] = None):
        """Get battery property or sub-property."""
        if subproperty is None:
            return self.battery
        return getattr(self.battery, subproperty, None)

    def _set_battery(self, subproperty: str, value) -> bool:
        """Set battery sub-property."""
        if hasattr(self.battery, subproperty):
            setattr(self.battery, subproperty, value)
            self.update_timestamp()
            return True
        return False

    def _get_matrix(self, subproperty: Optional[str] = None):
        """Get matrix property or sub-property."""
        if self.matrix is None:
            return None
        if subproperty is None:
            return self.matrix
        return getattr(self.matrix, subproperty, None)

    def _set_matrix(self, subproperty: str, value) -> bool:
        """Set matrix sub-property."""
        if self.matrix is None:
            self.matrix = SpheroMatrixState()
        if hasattr(self.matrix, subproperty):
            setattr(self.matrix, subproperty, value)
            self.update_timestamp()
            return True
        return False

    def get(self, property_name: str, subproperty: Optional[str] = None, default=None):
        """
        Get a top-level property or its sub-property.

        Args:
            property_name: Top-level property name (e.g., "orientation", "motion", "led")
            subproperty: Optional sub-property name (e.g., "pitch", "heading", "red")
            default: Default value to return if property not found

        Returns:
            The property value or default if not found

        Examples:
            >>> state.get("orientation")  # Returns SpheroOrientation object
            >>> state.get("orientation", "pitch")  # Returns pitch value
            >>> state.get("motion", "heading")  # Returns heading value
            >>> state.get("led", "red")  # Returns LED red value
            >>> state.get("toy_name")  # Returns toy name string
        """
        # Check if there's a private getter method
        getter_method = f'_get_{property_name}'
        if hasattr(self, getter_method):
            result = getattr(self, getter_method)(subproperty)
            return result if result is not None else default

        # For simple properties (toy_name, connection_state, etc.)
        if hasattr(self, property_name) and subproperty is None:
            return getattr(self, property_name)

        return default

    def set(self, property_name: str, value, subproperty: Optional[str] = None) -> bool:
        """
        Set a top-level property or its sub-property.

        Args:
            property_name: Top-level property name (e.g., "orientation", "motion", "led")
            value: Value to set
            subproperty: Optional sub-property name (e.g., "pitch", "heading", "red")

        Returns:
            bool: True if successful, False otherwise

        Examples:
            >>> state.set("orientation", 45.0, "pitch")  # Sets pitch to 45.0
            >>> state.set("motion", 90, "heading")  # Sets heading to 90
            >>> state.set("led", 255, "red")  # Sets LED red to 255
            >>> state.set("toy_name", "SB-1234")  # Sets toy name
        """
        # Check if there's a private setter method for nested properties
        if subproperty is not None:
            setter_method = f'_set_{property_name}'
            if hasattr(self, setter_method):
                return getattr(self, setter_method)(subproperty, value)
            return False

        # For simple properties (toy_name, connection_state, etc.)
        if hasattr(self, property_name):
            setattr(self, property_name, value)
            self.update_timestamp()
            return True

        return False

    def to_dict(self) -> dict:
        """
        Convert the complete state to a dictionary representation.

        Returns:
            dict: Complete state as nested dictionary
        """
        result = {
            'toy_name': self.toy_name,
            'toy_type': self.toy_type,
            'connection_state': self.connection_state.value,
            'motion': self.motion.to_dict(),
            'position': self.position.to_dict(),
            'velocity': self.velocity.to_dict(),
            'orientation': self.orientation.to_dict(),
            'accelerometer': self.accelerometer.to_dict(),
            'gyroscope': self.gyroscope.to_dict(),
            'led': self.led.to_dict(),
            'battery': self.battery.to_dict(),
            'timestamp': self.timestamp,
            'last_update': self.last_update
        }

        if self.matrix is not None:
            result['matrix'] = self.matrix.to_dict()

        return result

    def update_from_device(self):
        """
        Update the state by querying the Sphero device.

        This method queries all available sensors from the Sphero API
        and updates the internal state accordingly.

        Returns:
            bool: True if update was successful, False if no API available
        """
        if self._api is None:
            return False

        # Import PacketDecodingException for better error handling
        try:
            from spherov2.controls import PacketDecodingException
        except ImportError:
            PacketDecodingException = Exception

        # Query orientation from device
        try:
            orientation = self._api.get_orientation()
            self.set('orientation', orientation.get('pitch', 0.0), 'pitch')
            self.set('orientation', orientation.get('roll', 0.0), 'roll')
            self.set('orientation', orientation.get('yaw', 0.0), 'yaw')
        except PacketDecodingException:
            pass  # Bluetooth packet collision - skip this update
        except Exception:
            pass  # Silent fail - sensor may not be available

        # Query accelerometer from device
        try:
            accel = self._api.get_acceleration()
            self.set('accelerometer', accel.get('x', 0.0), 'x')
            self.set('accelerometer', accel.get('y', 0.0), 'y')
            self.set('accelerometer', accel.get('z', 0.0), 'z')
        except PacketDecodingException:
            pass  # Bluetooth packet collision - skip this update
        except Exception:
            pass

        # Query gyroscope from device
        try:
            gyro = self._api.get_gyroscope()
            self.set('gyroscope', gyro.get('x', 0.0), 'x')
            self.set('gyroscope', gyro.get('y', 0.0), 'y')
            self.set('gyroscope', gyro.get('z', 0.0), 'z')
        except PacketDecodingException:
            pass  # Bluetooth packet collision - skip this update
        except Exception:
            pass

        # Query location from device if we are using the sphero's internal localization
        if not self.external_localization:
            try:
                location = self._api.get_location()
                device_x = location.get('x', 0.0)
                device_y = location.get('y', 0.0)

                # Apply position offset (for reset_aim functionality)
                # This makes position relative to our coordinate system origin
                new_x = device_x - self._position_offset_x
                new_y = device_y - self._position_offset_y

                # Only update position if change exceeds threshold (reduces noise)
                if self._should_update_position(new_x, new_y):
                    self.set('position', new_x, 'x')
                    self.set('position', new_y, 'y')
                    self._last_published_position = (new_x, new_y)
            except PacketDecodingException:
                pass  # Bluetooth packet collision - skip this update
            except Exception:
                pass

        # Query velocity from device
        try:
            velocity = self._api.get_velocity()
            self.set('velocity', velocity.get('x', 0.0), 'x')
            self.set('velocity', velocity.get('y', 0.0), 'y')
        except PacketDecodingException:
            pass  # Bluetooth packet collision - skip this update
        except Exception:
            pass

        # Query heading from device
        try:
            heading = self._api.get_heading()
            self.set('motion', int(heading) % 360, 'heading')
        except PacketDecodingException:
            pass  # Bluetooth packet collision - skip this update
        except Exception:
            pass

        # Query speed from device
        try:
            speed = self._api.get_speed()
            self.set('motion', abs(int(speed)), 'speed')
            self.set('motion', abs(speed) > 0, 'is_moving')
        except PacketDecodingException:
            pass  # Bluetooth packet collision - skip this update
        except Exception:
            pass

        # Query LED state from device
        try:
            led = self._api.get_main_led()
            self.set('led', led.get('red', 0), 'red')
            self.set('led', led.get('green', 0), 'green')
            self.set('led', led.get('blue', 0), 'blue')
        except Exception:
            pass

        # Query battery state from device if available
        try:
            if self._toy is not None:
                # Use the Power command directly on the toy object
                from spherov2.commands.power import Power

                battery_voltage = None
                battery_pct = None

                # Try to get voltage first (more reliable)
                try:
                    battery_voltage = Power.get_battery_voltage(self._toy)
                    # Only update if we got a valid voltage
                    if battery_voltage is not None and battery_voltage > 0:
                        self.set('battery', float(battery_voltage), 'voltage')
                    else:
                        battery_voltage = None  # Treat invalid voltage as None
                except Exception:
                    # Voltage query may not be supported on all models
                    battery_voltage = None

                # Try to get percentage directly
                try:
                    battery_pct = Power.get_battery_percentage(self._toy)
                    # Only update if we got a valid percentage
                    if battery_pct is not None and 0 <= battery_pct <= 100:
                        self.set('battery', int(battery_pct), 'percentage')
                except Exception:
                    # Battery percentage query not supported on some models - calculate from voltage
                    if battery_voltage is not None and battery_voltage > 0:
                        # Calculate percentage from voltage
                        # Typical Sphero battery: 3.6V nominal
                        # Conservative range: 3.4V = 0%, 4.1V = 100%
                        min_voltage = 3.4
                        max_voltage = 4.1
                        calculated_pct = ((battery_voltage - min_voltage) / (max_voltage - min_voltage)) * 100
                        calculated_pct = max(0, min(100, int(calculated_pct)))
                        self.set('battery', calculated_pct, 'percentage')
        except Exception:
            pass

        # Update timestamp is called automatically by set() method
        return True

    def query_property(self, property_name: str, subproperty: Optional[str] = None):
        """
        Query a specific property from the Sphero device and update the state.

        This method queries a single property from the device rather than updating
        the entire state. It's useful for targeted updates without overhead.

        Args:
            property_name: The property to query (e.g., 'orientation', 'battery', 'motion')
            subproperty: Optional sub-property (e.g., 'pitch', 'heading', 'percentage')

        Returns:
            The queried value, or None if query failed or API not available

        Examples:
            >>> state.query_property('orientation', 'pitch')  # Query only pitch
            45.0
            >>> state.query_property('battery', 'percentage')  # Query battery %
            85
            >>> state.query_property('motion', 'heading')  # Query heading
            180

        Raises:
            ValueError: If property_name is not supported
        """
        if self._api is None:
            return None

        # Map properties to their API methods
        property_queries = {
            'orientation': {
                'method': 'get_orientation',
                'subproperties': ['pitch', 'roll', 'yaw']
            },
            'accelerometer': {
                'method': 'get_acceleration',
                'subproperties': ['x', 'y', 'z']
            },
            'gyroscope': {
                'method': 'get_gyroscope',
                'subproperties': ['x', 'y', 'z']
            },
            'position': {
                'method': 'get_location',
                'subproperties': ['x', 'y']
            },
            'velocity': {
                'method': 'get_velocity',
                'subproperties': ['x', 'y']
            },
            'motion': {
                'method': 'get_heading',  # Default to heading
                'subproperties': ['heading', 'speed', 'is_moving']
            },
            'led': {
                'method': 'get_main_led',
                'subproperties': ['red', 'green', 'blue']
            },
            'battery': {
                'method': 'get_battery_percentage',
                'subproperties': ['percentage', 'voltage']
            }
        }

        if property_name not in property_queries:
            raise ValueError(
                f"Unsupported property: {property_name}. "
                f"Supported properties: {', '.join(property_queries.keys())}"
            )

        query_info = property_queries[property_name]

        try:
            # Special handling for different property types
            if property_name == 'motion':
                if subproperty == 'heading' or subproperty is None:
                    heading = self._api.get_heading()
                    value = int(heading) % 360
                    self.set('motion', value, 'heading')
                    if subproperty == 'heading':
                        return value
                elif subproperty == 'speed':
                    speed = self._api.get_speed()
                    value = abs(int(speed))
                    self.set('motion', value, 'speed')
                    self.set('motion', abs(speed) > 0, 'is_moving')
                    return value
                elif subproperty == 'is_moving':
                    speed = self._api.get_speed()
                    value = abs(speed) > 0
                    self.set('motion', abs(int(speed)), 'speed')
                    self.set('motion', value, 'is_moving')
                    return value
                else:
                    # Return entire motion state if no subproperty
                    return self.motion

            elif property_name == 'battery':
                if self._toy is not None:
                    from spherov2.commands.power import Power
                    if subproperty == 'voltage':
                        try:
                            voltage = Power.get_battery_voltage(self._toy)
                            value = float(voltage)
                            self.set('battery', value, 'voltage')
                            return value
                        except Exception:
                            return None
                    else:  # percentage or None
                        battery_voltage = None
                        # Try to get voltage for fallback calculation
                        try:
                            battery_voltage = Power.get_battery_voltage(self._toy)
                            self.set('battery', float(battery_voltage), 'voltage')
                        except Exception:
                            pass

                        # Try to get percentage directly
                        try:
                            battery_pct = Power.get_battery_percentage(self._toy)
                            value = int(battery_pct)
                            self.set('battery', value, 'percentage')
                            if subproperty == 'percentage':
                                return value
                            return self.battery
                        except Exception:
                            # Calculate from voltage if available
                            if battery_voltage is not None:
                                min_voltage = 3.4
                                max_voltage = 4.1
                                calculated_pct = ((battery_voltage - min_voltage) / (max_voltage - min_voltage)) * 100
                                calculated_pct = max(0, min(100, int(calculated_pct)))
                                self.set('battery', calculated_pct, 'percentage')
                                if subproperty == 'percentage':
                                    return calculated_pct
                                return self.battery
                            return None
                return None

            else:
                # Standard property query
                api_method = getattr(self._api, query_info['method'])
                data = api_method()

                if isinstance(data, dict):
                    # Update all subproperties
                    for subprop in query_info['subproperties']:
                        if subprop in data:
                            self.set(property_name, data[subprop], subprop)

                    # Return requested subproperty or entire object
                    if subproperty:
                        return data.get(subproperty)
                    else:
                        return self.get(property_name)
                else:
                    # Single value returned
                    if subproperty:
                        self.set(property_name, data, subproperty)
                        return data
                    else:
                        return self.get(property_name)

        except AttributeError as e:
            # API method doesn't exist
            return None
        except Exception as e:
            # Query failed
            return None

    def to_sphero_sensor_msg(self):
        """
        Convert the state to a SpheroSensor message.

        Returns:
            SpheroSensor message or None if ROS2 is not available

        Note: This method requires the sphero_instance_controller.msg module to be available.
        """
        try:
            from sphero_instance_controller.msg import SpheroSensor
        except ImportError:
            return None

        sensor_msg = SpheroSensor()

        # Orientation
        sensor_msg.pitch = float(self.orientation.pitch)
        sensor_msg.roll = float(self.orientation.roll)
        sensor_msg.yaw = float(self.orientation.yaw)

        # Accelerometer
        sensor_msg.accel_x = float(self.accelerometer.x)
        sensor_msg.accel_y = float(self.accelerometer.y)
        sensor_msg.accel_z = float(self.accelerometer.z)

        # Gyroscope
        sensor_msg.gyro_x = float(self.gyroscope.x)
        sensor_msg.gyro_y = float(self.gyroscope.y)
        sensor_msg.gyro_z = float(self.gyroscope.z)

        # Location
        sensor_msg.x = float(self.position.x)
        sensor_msg.y = float(self.position.y)

        # Velocity
        sensor_msg.velocity_x = float(self.velocity.x)
        sensor_msg.velocity_y = float(self.velocity.y)

        # Battery
        sensor_msg.battery_percentage = int(self.battery.percentage)

        # Timestamp (will be set by caller if needed)
        # sensor_msg.timestamp is left unset here

        return sensor_msg

    def is_healthy(self) -> bool:
        """
        Check if the Sphero is in a healthy state.

        Returns:
            bool: True if connected and battery is sufficient
        """
        return (
            self.connection_state == SpheroConnectionState.CONNECTED and
            self.battery.percentage > 10
        )

    def __str__(self) -> str:
        """String representation of the Sphero state."""
        return (
            f"SpheroState(toy='{self.toy_name}', "
            f"connected={self.connection_state.value}, "
            f"heading={self.motion.heading}�, "
            f"speed={self.motion.speed}, "
            f"battery={self.battery.percentage}%)"
        )

    def __repr__(self) -> str:
        """Detailed representation of the Sphero state."""
        return self.__str__()
