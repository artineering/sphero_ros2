"""
Monkey patch for spherov2 collision detection parsing bug.

The spherov2 library expects 18 bytes in collision responses but some Sphero models
(like BOLT) only send 16 bytes, causing parsing errors. This patch handles both cases.
"""

import struct
from spherov2.commands.sensor import Sensor, CollisionDetected


def apply_collision_patch():
    """Apply monkey patch to fix collision detection parsing."""
    def patched_collision_helper(listener, packet):
        """Handle both 16 and 18 byte collision responses"""
        import sys
        # print(f"[spherov2-patch] Collision packet received! Size: {len(packet.data)} bytes", file=sys.stderr)
        # print(f"[spherov2-patch] Packet hex: {packet.data.hex()}", file=sys.stderr)

        try:
            data = packet.data

            # Pad data if it's shorter than 18 bytes
            if len(data) == 16:
                # print(f"[spherov2-patch] Padding 16-byte packet to 18 bytes", file=sys.stderr)
                data = data + b'\x00\x00'
            elif len(data) < 18:
                # print(f"[spherov2-patch] Padding {len(data)}-byte packet to 18 bytes", file=sys.stderr)
                data = data + b'\x00' * (18 - len(data))

            # Parse the collision data
            unpacked = struct.unpack('>3hB3hBL', data)
            collision_data = CollisionDetected(
                acceleration_x=unpacked[0] / 4096,
                acceleration_y=unpacked[1] / 4096,
                acceleration_z=unpacked[2] / 4096,
                x_axis=bool(unpacked[3] & 1),
                y_axis=bool(unpacked[3] & 2),
                power_x=unpacked[4],
                power_y=unpacked[5],
                power_z=unpacked[6],
                speed=unpacked[7],
                time=unpacked[8] / 1000
            )
            # print(f"[spherov2-patch] Successfully parsed collision data", file=sys.stderr)
            # print(f"[spherov2-patch] Calling listener with collision_data", file=sys.stderr)
            listener(collision_data)
            # print(f"[spherov2-patch] Listener called successfully", file=sys.stderr)

        except struct.error as e:
            # Log malformed collision packets with debug info
            import sys
            packet_size = len(packet.data) if hasattr(packet, 'data') else 'unknown'
            packet_hex = packet.data.hex() if hasattr(packet, 'data') else 'N/A'
            print(f"[spherov2-patch] Collision packet parse error: {e}", file=sys.stderr)
            print(f"[spherov2-patch] Expected 18 bytes, got {packet_size} bytes", file=sys.stderr)
            print(f"[spherov2-patch] Packet data (hex): {packet_hex}", file=sys.stderr)
            # Silently continue
            pass

    # Apply the patch to both the helper method AND the tuple
    # The tuple collision_detected_notify contains (command_info, helper_function)
    # We need to replace the helper in the tuple since it was captured at class definition time
    Sensor._Sensor__collision_detected_notify_helper = staticmethod(patched_collision_helper)

    # Also patch the tuple itself
    original_tuple = Sensor.collision_detected_notify
    Sensor.collision_detected_notify = (original_tuple[0], patched_collision_helper)

    # IMPORTANT: We also need to patch any Toy classes that have already imported the old tuple
    # The BOLT class uses partialmethod with Sensor.collision_detected_notify
    # We need to recreate those partialmethods with the patched tuple
    try:
        from functools import partialmethod
        from spherov2.toy import Toy
        from spherov2.toy.bolt import BOLT

        # Recreate the partialmethod with the patched tuple
        BOLT.add_collision_detected_notify_listener = partialmethod(
            Toy._add_listener,
            Sensor.collision_detected_notify
        )
        BOLT.remove_collision_detected_notify_listener = partialmethod(
            Toy._remove_listener,
            Sensor.collision_detected_notify
        )
        print("Applied spherov2 collision detection patch (16->18 byte fix) + BOLT partialmethods")
    except ImportError as e:
        # BOLT might not be imported yet, which is fine
        print(f"Applied spherov2 collision detection patch (16->18 byte fix) - BOLT not yet loaded: {e}")
