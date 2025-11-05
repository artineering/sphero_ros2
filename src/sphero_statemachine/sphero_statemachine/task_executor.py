#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Task Executor Node for State Machine.

This node subscribes to state machine task commands and executes them
by publishing to the appropriate Sphero control topics.
"""

import json
import time
from typing import Dict, Any, List, Optional
from queue import Queue
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class TaskExecutorNode(Node):
    """
    Executes state machine tasks by translating them to Sphero commands.

    Subscribes to:
    - /state_machine/task_command: Task commands from state machine

    Publishes to:
    - /sphero/led: LED color commands
    - /sphero/roll: Roll/movement commands
    - /sphero/heading: Heading commands
    - /sphero/speed: Speed commands
    - /sphero/stop: Stop commands
    - /sphero/matrix: LED matrix commands
    """

    def __init__(self):
        """Initialize the task executor node."""
        super().__init__('task_executor')

        # Task queue for sequential execution
        self.task_queue: List[Dict[str, Any]] = []
        self.current_task: Optional[Dict[str, Any]] = None
        self.all_tasks: List[Dict[str, Any]] = []  # Store all tasks for a state
        self.waiting_for_completion = False
        self.task_lock = threading.Lock()

        # Subscribe to state machine task commands
        self.task_sub = self.create_subscription(
            String,
            '/state_machine/task_command',
            self.task_callback,
            10
        )

        # Subscribe to sphero task status
        self.status_sub = self.create_subscription(
            String,
            '/sphero/task/status',
            self.status_callback,
            10
        )

        # Publisher for sphero task controller
        self.sphero_task_pub = self.create_publisher(String, '/sphero/task', 10)

        # Publisher for task execution feedback
        self.feedback_pub = self.create_publisher(
            String,
            '/state_machine/task_feedback',
            10
        )

        self.get_logger().info('Task Executor Node initialized')
        self.get_logger().info('Ready to execute state machine tasks')

    def task_callback(self, msg: String):
        """
        Handle incoming task commands from state machine.
        Queue tasks for sequential execution.

        Expected JSON format:
        {
            "state": "state_name",
            "task_type": "task_type",
            "params": {...},
            "task_index": 0,
            "total_tasks": 1,
            "timestamp": 123.456
        }
        """
        try:
            task = json.loads(msg.data)
            task_index = task.get('task_index', 0)
            total_tasks = task.get('total_tasks', 1)
            state = task.get('state', 'unknown')
            task_type = task.get('task_type', 'none')

            self.get_logger().info(
                f'Received task {task_index + 1}/{total_tasks} from state "{state}": {task_type}'
            )

            with self.task_lock:
                # If this is the first task (index 0), clear previous queue
                if task_index == 0:
                    self.task_queue.clear()
                    self.all_tasks.clear()

                # Add task to queue
                self.task_queue.append(task)
                self.all_tasks.append(task)

                # If no task is currently executing, start the first one
                if not self.waiting_for_completion and self.current_task is None:
                    self.process_next_task()

        except json.JSONDecodeError as e:
            self.get_logger().error(f'Failed to parse task JSON: {e}')
        except Exception as e:
            self.get_logger().error(f'Error queuing task: {e}')
            import traceback
            self.get_logger().error(traceback.format_exc())

    def status_callback(self, msg: String):
        """
        Handle status updates from sphero task controller.
        When a task completes, publish the next task in queue.
        """
        try:
            status = json.loads(msg.data)
            task_status = status.get('status')
            task_id = status.get('task_id', 'unknown')

            if task_status == 'completed':
                self.get_logger().info(f'Sphero task controller completed task: {task_id}')

                with self.task_lock:
                    if self.current_task:
                        current_index = self.current_task.get('task_index', 0)
                        total_tasks = self.current_task.get('total_tasks', 1)
                        state = self.current_task.get('state', 'unknown')

                        self.get_logger().info(
                            f'Task {current_index + 1}/{total_tasks} acknowledged as completed. '
                            f'Proceeding to next task (if any)...'
                        )

                        self.waiting_for_completion = False
                        self.current_task = None

                        # Process next task if available
                        if len(self.task_queue) > 0:
                            self.process_next_task()
                        else:
                            # All tasks completed successfully
                            self.get_logger().info(f'All tasks for state "{state}" completed successfully!')
                            self.publish_feedback(state, 'all_tasks', True)

            elif task_status == 'failed':
                error_msg = status.get('error_message', 'Unknown error')
                self.get_logger().error(f'Sphero task controller failed task: {task_id} - {error_msg}')

                with self.task_lock:
                    if self.current_task:
                        state = self.current_task.get('state', 'unknown')
                        self.get_logger().error(f'Aborting remaining tasks for state "{state}"')

                        # Clear queue and publish failure feedback
                        self.task_queue.clear()
                        self.waiting_for_completion = False
                        self.current_task = None
                        self.publish_feedback(state, 'task_execution', False)

        except json.JSONDecodeError as e:
            self.get_logger().error(f'Failed to parse status JSON: {e}')
        except Exception as e:
            self.get_logger().error(f'Error processing status: {e}')

    def process_next_task(self):
        """
        Process the next task in the queue.
        Must be called while holding task_lock.
        """
        if len(self.task_queue) == 0:
            return

        # Get next task
        self.current_task = self.task_queue.pop(0)
        task_type = self.current_task.get('task_type', 'none')
        params = self.current_task.get('params', {})
        state = self.current_task.get('state', 'unknown')
        task_index = self.current_task.get('task_index', 0)
        total_tasks = self.current_task.get('total_tasks', 1)

        self.get_logger().info(
            f'Executing task {task_index + 1}/{total_tasks} from state "{state}": '
            f'{task_type} with params: {params}'
        )

        # Execute the task based on type
        success = self.execute_task(task_type, params)

        if not success:
            self.get_logger().error(f'Failed to execute task {task_index + 1}/{total_tasks}: {task_type}')
            # Clear remaining tasks and publish failure
            self.task_queue.clear()
            self.current_task = None
            self.waiting_for_completion = False
            self.publish_feedback(state, task_type, False)
        else:
            # Mark that we're waiting for completion from sphero task controller
            self.waiting_for_completion = True
            self.get_logger().info(
                f'Task {task_index + 1}/{total_tasks} published. '
                f'Waiting for completion status from sphero task controller...'
            )

    def execute_task(self, task_type: str, params: Dict[str, Any]) -> bool:
        """
        Send task to sphero task controller for execution.

        Args:
            task_type: Type of task to execute
            params: Task parameters

        Returns:
            True if task was successfully sent, False otherwise
        """
        try:
            # Generate unique task ID
            task_id = f"sm_task_{int(time.time() * 1000)}"

            # Create task message for sphero task controller
            task_message = {
                'task_id': task_id,
                'task_type': task_type,
                'parameters': params
            }

            # Publish to sphero task controller
            msg = String()
            msg.data = json.dumps(task_message)
            self.sphero_task_pub.publish(msg)

            self.get_logger().info(f'Sent task to sphero controller: {task_type} (ID: {task_id})')
            return True

        except Exception as e:
            self.get_logger().error(f'Error sending task {task_type} to sphero controller: {e}')
            return False

    def publish_feedback(self, state: str, task_type: str, success: bool):
        """Publish task execution feedback."""
        feedback = {
            'state': state,
            'task_type': task_type,
            'success': success,
            'timestamp': time.time()
        }

        msg = String()
        msg.data = json.dumps(feedback)
        self.feedback_pub.publish(msg)


def main(args=None):
    """Main entry point for the task executor node."""
    rclpy.init(args=args)

    node = TaskExecutorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down task executor...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
