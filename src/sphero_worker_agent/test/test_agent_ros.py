#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
In-process ROS test of the connect-queue node: a late-joining transient_local
subscriber gets the latest queue, and a /status 'connected' removes the robot.
"""

import json
import threading
import time

import pytest

rclpy = pytest.importorskip('rclpy')
from rclpy.executors import SingleThreadedExecutor  # noqa: E402,I100,I201
from sphero_instance_controller.ble_connect_queue import (  # noqa: E402,I100,I201
    connect_queue_qos, connect_queue_topic)
from sphero_worker_agent.connect_queue import make_node  # noqa: E402,I100,I201
from std_msgs.msg import String  # noqa: E402,I100,I201


def _wait(pred, timeout=10.0):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if pred():
            return True
        time.sleep(0.05)
    return False


def test_late_joiner_gets_queue_and_connected_removes():
    host = f'test-host-{int(time.time() * 1000) % 100000}'
    ctx = rclpy.Context()
    rclpy.init(context=ctx)
    executor = SingleThreadedExecutor(context=ctx)
    try:
        # make_node uses the default context, so give it ours via a shim.
        from rclpy.node import Node
        orig_init = Node.__init__

        def init_with_ctx(self, *a, **k):
            k.setdefault('context', ctx)
            orig_init(self, *a, **k)
        Node.__init__ = init_with_ctx
        try:
            agent = make_node(host, lambda name: True)
            agent.on_spawn('SB-ROS', 'sid1')
            listener = rclpy.create_node('queue_listener', context=ctx)
        finally:
            Node.__init__ = orig_init
        got = []
        listener.create_subscription(
            String, connect_queue_topic(host),
            lambda m: got.append(json.loads(m.data)), connect_queue_qos())
        status_pub = listener.create_publisher(String, '/sphero/SB_ROS/status', 10)
        executor.add_node(agent)
        executor.add_node(listener)
        threading.Thread(target=executor.spin, daemon=True).start()

        assert _wait(lambda: got and [p['name'] for p in got[-1]['pending']] == ['SB-ROS'])
        # The agent subscribes to the robot's /status on its next tick.
        assert _wait(lambda: status_pub.get_subscription_count() > 0)
        status_pub.publish(String(data=json.dumps(
            {'connection_state': 'connected', 'spawn_id': 'sid1'})))
        assert _wait(lambda: got[-1]['pending'] == [])
    finally:
        executor.shutdown(timeout_sec=2.0)
        rclpy.try_shutdown(context=ctx)
