#!/usr/bin/env python3
"""
udp_range_reader.py
===================
Threaded UDP reader for UWB anchor boards.
Reads JSON packets from WiFi-enabled anchors (Portenta C33 + UWB Shield) via UDP,
parses range measurements, and deposits them into a thread-safe queue.

Expected JSON format from anchors:
{"anchor_id": N, "tag_id": M, "distance_cm": X, "timestamp_ms": T}
"""

import json
import socket
import threading
import time
from collections import deque
from dataclasses import dataclass
from typing import Optional


@dataclass
class RangeMeasurement:
    """A single TWR distance measurement."""
    anchor_id: int
    tag_id: int
    distance_m: float      # Converted from cm to meters
    timestamp: float        # Python time.time() when received
    device_ts_ms: int       # Timestamp from the Arduino millis()


class UDPRangeReader(threading.Thread):
    """
    Reads JSON range data from multiple anchors via UDP in a background thread.
    All anchors send to the same UDP port.
    Parsed measurements are placed in a shared deque for the main node to consume.
    """

    def __init__(
        self,
        port: int,
        output_queue: deque,
        max_queue_size: int = 2000,
        logger=None,
    ):
        super().__init__(daemon=True)
        self.port = port
        self.output_queue = output_queue
        self.max_queue_size = max_queue_size
        self.logger = logger
        self._stop_event = threading.Event()
        self._socket: Optional[socket.socket] = None
        self.connected = False
        self.rx_count = 0
        self.error_count = 0
        # Track per-anchor statistics
        self.anchor_stats = {}  # anchor_id -> {'rx_count': int, 'last_seen': float}

    def stop(self):
        """Signal the thread to stop."""
        self._stop_event.set()

    def run(self):
        """Main thread loop: open socket, receive packets, parse JSON."""
        while not self._stop_event.is_set():
            try:
                self._connect()
                self._read_loop()
            except OSError as e:
                if self.logger:
                    self.logger.warn(
                        f"UDP socket error on port {self.port}: {e}. "
                        f"Reconnecting in 2s..."
                    )
                self.connected = False
                time.sleep(2.0)
            except Exception as e:
                if self.logger:
                    self.logger.error(
                        f"UDP reader unexpected error: {e}"
                    )
                self.error_count += 1
                time.sleep(1.0)

    def _connect(self):
        """Open UDP socket and bind to port."""
        if self._socket:
            try:
                self._socket.close()
            except:
                pass

        if self.logger:
            self.logger.info(
                f"Binding UDP socket to 0.0.0.0:{self.port}"
            )

        self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._socket.settimeout(1.0)  # 1 second timeout for recvfrom
        self._socket.bind(('0.0.0.0', self.port))

        self.connected = True

        if self.logger:
            self.logger.info(f"UDP socket bound to port {self.port}")

    def _read_loop(self):
        """Main receive loop: read packets, parse JSON, enqueue measurements."""
        while not self._stop_event.is_set():
            try:
                # Receive UDP packet (max 1024 bytes)
                data, addr = self._socket.recvfrom(1024)

                if not data:
                    continue

                # Decode and parse JSON
                line = data.decode("utf-8", errors="replace").strip()

                # Skip empty lines or comments
                if not line or line.startswith("#"):
                    continue

                # Parse JSON
                packet = json.loads(line)

                # Extract fields
                anchor_id = packet["anchor_id"]
                tag_id = packet["tag_id"]
                distance_cm = packet["distance_cm"]
                device_ts_ms = packet.get("timestamp_ms", 0)

                # Create measurement
                measurement = RangeMeasurement(
                    anchor_id=anchor_id,
                    tag_id=tag_id,
                    distance_m=distance_cm / 100.0,
                    timestamp=time.time(),
                    device_ts_ms=device_ts_ms,
                )

                # Update anchor statistics
                if anchor_id not in self.anchor_stats:
                    self.anchor_stats[anchor_id] = {'rx_count': 0, 'last_seen': 0.0}
                self.anchor_stats[anchor_id]['rx_count'] += 1
                self.anchor_stats[anchor_id]['last_seen'] = measurement.timestamp

                # Deposit into shared queue
                if len(self.output_queue) >= self.max_queue_size:
                    self.output_queue.popleft()  # Drop oldest
                self.output_queue.append(measurement)
                self.rx_count += 1

            except socket.timeout:
                # Normal timeout, continue
                continue
            except json.JSONDecodeError:
                self.error_count += 1
                if self.logger:
                    self.logger.debug(f"UDP packet JSON decode error: {line[:100]}")
            except KeyError as e:
                self.error_count += 1
                if self.logger:
                    self.logger.debug(
                        f"UDP packet missing required key: {e}"
                    )
            except OSError:
                raise  # Let outer handler reconnect
            except Exception as e:
                self.error_count += 1
                if self.logger:
                    self.logger.debug(f"UDP packet parse error: {e}")

    def close(self):
        """Stop the thread and close the socket."""
        self.stop()
        if self._socket:
            try:
                self._socket.close()
            except:
                pass
