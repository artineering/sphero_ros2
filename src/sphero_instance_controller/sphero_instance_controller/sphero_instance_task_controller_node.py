#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Sphero Instance Task Controller Node.

This node accepts high-level tasks from a dedicated topic and executes them
using the TaskExecutor class. Designed for multi-robot setups with namespaced topics.

All topics are namespaced under 'sphero/<sphero_name>/' to allow multiple
instances to run simultaneously without cross-talk.
"""

import json
import time
import signal

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import QoSProfile, DurabilityPolicy

from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from multirobot_msgs.msg import FleetPolicy, SimilarityCue, ProximityCue

from sphero_instance_controller.core.common.task import TaskDescriptor, TaskStatus
from sphero_instance_controller.core.sphero.topic_task_executor import TopicTaskExecutor

# Note: Task controller does NOT import scanner, SpheroEduAPI, or Sphero
# It only communicates through ROS topics


class SpheroInstanceTaskController(Node):
    """
    ROS2 node for high-level task control of a single Sphero instance.

    Uses namespaced topics for multi-robot support.
    All topics are prefixed with 'sphero/<sphero_name>/'
    """

    def __init__(self, sphero_name: str):
        """
        Initialize the task controller node.

        Args:
            sphero_name: Name of the Sphero (used for topic namespacing)
        """
        # Sanitize name: replace hyphens with underscores (ROS2 naming rules)
        name_safe = sphero_name.replace("-", "_")
        # Create unique node name with sphero name suffix
        node_name = f'sphero_task_controller_{name_safe}'
        super().__init__(node_name)

        self.sphero_name = sphero_name
        # Sanitize topic name: replace hyphens with underscores (ROS2 topic naming rules)
        topic_name_safe = name_safe
        self.name_safe = name_safe
        self.topic_prefix = f'sphero/{topic_name_safe}'

        # Fleet-policy (Similarity cue) state. The cue is applied THROUGH the
        # task / state-machine stack, never by touching the device directly:
        # steady IDENTITY enqueues a one-shot render task; BEHAVIOR (blink) is
        # handed to the state machine controller as a two-state timer loop.
        self._active_policy_id = None
        self._policy_surface = None  # 'led' | 'matrix' | None
        self._policy_mode = None     # 'steady' | 'blink' | None
        # Spin is a Behavioral Similarity sub-behavior: a finite spin task on the
        # DRIVE lane, composable with the blink SM (LED/matrix lane).
        self._policy_spin_active = False

        # Proximity cue (phase 2). Per-unit clustering driven by a two-state
        # timer SM on the DRIVE lane; the step handler reads live group positions
        # from the executor. Member localization subscriptions are held here and
        # exposed to the executor via `get_member_positions`.
        self._policy_proximity_active = False
        self._member_positions = {}       # name_safe -> {'x','y','t'}
        self._member_position_subs = {}   # name_safe -> Subscription

        # Note: Task controller does NOT connect to hardware
        # It only communicates through ROS topics with the device controller

        # State tracking
        self.current_state = {}
        self.current_position = {'x': 0.0, 'y': 0.0}
        self.current_heading = 0

        # Callback group for reentrant callbacks
        self.callback_group = ReentrantCallbackGroup()

        # Create publishers first (needed for command_publisher callback)
        self._create_publishers()

        # Create subscribers
        self._create_subscribers()

        # Initialize task executor (topic-based, no direct Sphero access)
        self.task_executor = TopicTaskExecutor(
            command_publisher=self.publish_command,
            position_callback=self.get_current_position,
            heading_callback=self.get_current_heading,
            member_positions_callback=self.get_member_positions
        )

        # Create timer for task execution (10 Hz)
        self.task_timer = self.create_timer(
            0.01,
            self.task_execution_loop,
            callback_group=self.callback_group
        )

        # Log initialization
        self._log_initialization()

        # Note: LED control is done through ROS topics, not direct hardware access

    def _create_subscribers(self):
        """Create all ROS subscribers."""
        # Task command subscriber
        self.task_sub = self.create_subscription(
            String,
            f'{self.topic_prefix}/task',
            self.task_callback,
            10,
            callback_group=self.callback_group
        )

        # State feedback subscriber
        self.state_sub = self.create_subscription(
            String,
            f'{self.topic_prefix}/state',
            self.state_callback,
            10,
            callback_group=self.callback_group
        )

        # Reset aim subscriber
        self.reset_aim_sub = self.create_subscription(
            String,
            f'{self.topic_prefix}/reset_aim',
            self.reset_aim_callback,
            10,
            callback_group=self.callback_group
        )

        # Fleet-wide policy broadcast. Single latched topic shared by every unit;
        # transient-local QoS so a (re)started controller immediately receives
        # the current policy. Typed multirobot_msgs/FleetPolicy, not JSON String.
        policy_qos = QoSProfile(depth=1)
        policy_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.policy_sub = self.create_subscription(
            FleetPolicy,
            '/fleet/policy',
            self.policy_callback,
            policy_qos,
            callback_group=self.callback_group
        )

    def _create_publishers(self):
        """Create all ROS publishers."""
        # Task status publisher
        self.task_status_pub = self.create_publisher(
            String,
            f'{self.topic_prefix}/task/status',
            10
        )

        # Command publishers for task execution
        self.raw_motor_pub = self.create_publisher(
            String,
            f'{self.topic_prefix}/raw_motor',
            10
        )

        self.motion_pub = self.create_publisher(
            String,
            f'{self.topic_prefix}/roll',
            10
        )

        self.led_pub = self.create_publisher(
            String,
            f'{self.topic_prefix}/led',
            10
        )

        self.heading_pub = self.create_publisher(
            String,
            f'{self.topic_prefix}/heading',
            10
        )

        self.speed_pub = self.create_publisher(
            String,
            f'{self.topic_prefix}/speed',
            10
        )

        self.spin_pub = self.create_publisher(
            String,
            f'{self.topic_prefix}/spin',
            10
        )

        self.calibrate_compass_pub = self.create_publisher(
            String,
            f'{self.topic_prefix}/calibrate_compass',
            10
        )

        self.matrix_pub = self.create_publisher(
            String,
            f'{self.topic_prefix}/matrix',
            10
        )

        self.stop_pub = self.create_publisher(
            String,
            f'{self.topic_prefix}/stop',
            10
        )

        self.stabilization_pub = self.create_publisher(
            String,
            f'{self.topic_prefix}/stabilization',
            10
        )

        self.collision_pub = self.create_publisher(
            String,
            f'{self.topic_prefix}/collision',
            10
        )

        self.ir_pub = self.create_publisher(
            String,
            f'{self.topic_prefix}/ir',
            10
        )

        # State-machine hand-off for ticking cues (BEHAVIOR blink). The task
        # controller builds a state machine config and drives the state machine
        # controller through its config / control topics.
        self.sm_config_pub = self.create_publisher(
            String,
            f'{self.topic_prefix}/state_machine/config',
            10
        )

        self.sm_control_pub = self.create_publisher(
            String,
            f'{self.topic_prefix}/state_machine/control',
            10
        )

    def _log_initialization(self):
        """Log initialization information."""
        self.get_logger().info('='*70)
        self.get_logger().info('🎯 TASK CONTROLLER ACTIVATED')
        self.get_logger().info('='*70)
        self.get_logger().info(f'Sphero Name: {self.sphero_name}')
        self.get_logger().info(f'Topic Prefix: {self.topic_prefix}')
        self.get_logger().info('Subscribed Topics:')
        self.get_logger().info(f'  - {self.topic_prefix}/task')
        self.get_logger().info(f'  - {self.topic_prefix}/state')
        self.get_logger().info(f'  - {self.topic_prefix}/reset_aim')
        self.get_logger().info('Publishing Topics:')
        self.get_logger().info(f'  - {self.topic_prefix}/task/status')
        self.get_logger().info(f'  - {self.topic_prefix}/raw_motor')
        self.get_logger().info(f'  - {self.topic_prefix}/roll')
        self.get_logger().info(f'  - {self.topic_prefix}/led')
        self.get_logger().info(f'  - {self.topic_prefix}/heading')
        self.get_logger().info(f'  - {self.topic_prefix}/speed')
        self.get_logger().info(f'  - {self.topic_prefix}/spin')
        self.get_logger().info(f'  - {self.topic_prefix}/calibrate_compass')
        self.get_logger().info(f'  - {self.topic_prefix}/matrix')
        self.get_logger().info(f'  - {self.topic_prefix}/stop')
        self.get_logger().info(f'  - {self.topic_prefix}/stabilization')
        self.get_logger().info(f'  - {self.topic_prefix}/collision')
        self.get_logger().info(f'  - {self.topic_prefix}/ir')
        self.get_logger().info('='*70)
        self.get_logger().info('✅ Task Controller READY (Topic-Based Executor)')
        self.get_logger().info('='*70)

    def get_current_position(self):
        """Get current position for task executor."""
        return self.current_position.copy()

    def get_current_heading(self):
        """Get current heading for task executor."""
        return self.current_heading

    def get_member_positions(self):
        """Group localization snapshot for the executor's Proximity handler."""
        return self._member_positions

    def publish_command(self, topic_name: str, params: dict):
        """
        Publish a command to the appropriate ROS topic.

        Args:
            topic_name: The command topic name (e.g., 'raw_motor', 'led', 'motion')
            params: Dictionary of parameters for the command
        """
        msg = String()
        msg.data = json.dumps(params)

        # Route to appropriate publisher based on topic name
        if topic_name == 'raw_motor':
            self.raw_motor_pub.publish(msg)
            self.get_logger().debug(f'Published raw_motor: {params}')
        elif topic_name == 'motion':
            # Motion commands can be roll or stop
            action = params.get('action', 'roll')
            if action == 'stop':
                self.stop_pub.publish(msg)
                self.get_logger().debug('Published stop command')
            else:
                self.motion_pub.publish(msg)
                self.get_logger().debug(f'Published motion: {params}')
        elif topic_name == 'led':
            self.led_pub.publish(msg)
            self.get_logger().debug(f'Published LED: {params}')
        elif topic_name == 'heading':
            self.heading_pub.publish(msg)
            self.get_logger().debug(f'Published heading: {params}')
        elif topic_name == 'speed':
            self.speed_pub.publish(msg)
            self.get_logger().debug(f'Published speed: {params}')
        elif topic_name == 'spin':
            self.spin_pub.publish(msg)
            self.get_logger().debug(f'Published spin: {params}')
        elif topic_name == 'calibrate_compass':
            self.calibrate_compass_pub.publish(msg)
            self.get_logger().debug('Published calibrate_compass command')
        elif topic_name == 'matrix':
            self.matrix_pub.publish(msg)
            self.get_logger().debug(f'Published matrix: {params}')
        elif topic_name == 'stabilization':
            self.stabilization_pub.publish(msg)
            self.get_logger().debug(f'Published stabilization: {params}')
        elif topic_name == 'collision':
            self.collision_pub.publish(msg)
            self.get_logger().debug(f'Published collision: {params}')
        elif topic_name == 'ir':
            self.ir_pub.publish(msg)
            self.get_logger().debug(f'Published ir: {params}')
        else:
            self.get_logger().warning(f'Unknown command topic: {topic_name}')

    # ===== Callbacks =====

    def _compute_start_at(self, now, start_offset):
        """Resolve an optional synchronized-start instant to an absolute epoch.

        Anchors to the coordinator's `now` so units that receive late still fire
        at the same absolute target. Returns None for immediate start.
        """
        if now is not None and start_offset is not None:
            target = float(now) + float(start_offset)
            if target > time.time():
                return target
        return None

    def _build_task(self, item, start_at):
        """Build one TaskDescriptor from a single-task / sub-task dict.

        Normalizes the `name` alias to `task_id`, auto-generates a task_id when
        absent, and resolves lanes (honoring an explicit `lane`/`lanes`
        override). Returns the TaskDescriptor (does not enqueue).
        """
        # name alias -> task_id; auto-generate when neither is present.
        task_id = item.get('task_id') or item.get('name')
        if not task_id:
            task_id = f"task_{int(time.time() * 1000)}"

        task = TaskDescriptor(
            task_id=task_id,
            task_type=item['task_type'],
            parameters=item.get('parameters', {}),
        )

        # Lane resolution: explicit override wins, else infer from task_type.
        override = item.get('lanes') or item.get('lane')
        if override is not None:
            override = [override] if isinstance(override, str) else override
            task.lanes = frozenset(override)
        else:
            task.lanes = self.task_executor.lanes_for(task.task_type)

        if start_at is not None:
            task.start_at = start_at
        return task

    def task_callback(self, msg: String):
        """
        Handle incoming task messages.

        Accepts three shapes:
          (a) single task:  {"task_type": "...", "parameters": {...}}
              (+ optional task_id / name, now / start_offset). Lane inferred
              from task_type. Back-compat for every existing caller.
          (b) concurrent bundle:  {"tasks": [ {...}, {...} ], now, start_offset}
              One shared start_at across all sub-tasks. Rejected if two
              sub-tasks share a lane or any sub-task is exclusive.
          (c) targeted stop / halt: a single `stop` task whose parameters carry
              `target` / `scope` (interpreted by the scheduler).
        """
        try:
            task_data = json.loads(msg.data)

            # ----- Shape (b): concurrent bundle -----
            if isinstance(task_data.get('tasks'), list):
                self._handle_bundle(task_data)
                return

            # ----- Shape (a)/(c): single task -----
            if 'task_type' not in task_data:
                self.get_logger().error('Task missing required field: task_type')
                return

            if not self._task_targets_me(self.sphero_name, task_data):
                self.get_logger().debug(
                    f"single task {task_data['task_type']} not targeted at "
                    f"{self.sphero_name}; skipping"
                )
                return

            start_at = self._compute_start_at(
                task_data.get('now'), task_data.get('start_offset'))
            task = self._build_task(task_data, start_at)

            self.task_executor.add_task(task)

            schedule_note = (
                f' scheduled in {task.start_at - time.time():.2f}s'
                if task.start_at is not None else ''
            )
            self.get_logger().info(
                f'Added task {task.task_id} ({task.task_type}) to queue. '
                f'Queue length: {len(self.task_executor.task_queue)}{schedule_note}'
            )
            self.get_logger().info(f'Task parameters: {json.dumps(task.parameters, indent=2)}')

            self.publish_task_status(task)

        except json.JSONDecodeError as e:
            self.get_logger().error(f'Invalid JSON in task message: {e}')
        except Exception as e:
            self.get_logger().error(f'Error processing task: {e}')

    @staticmethod
    def _task_targets_me(sphero_name, item):
        """True if this Sphero should run `item`.

        Omitted/None `targets` -> applies to all units (back-compat).
        A list -> case-insensitive membership test. Empty list -> nobody.
        """
        targets = item.get('targets')
        if targets is None:
            return True
        if not isinstance(targets, list):
            return True   # malformed -> fail open; caller logs a warning
        me = str(sphero_name).strip().upper()
        return any(str(t).strip().upper() == me for t in targets)

    @staticmethod
    def _bundle_lane_conflict(tasks):
        """Find the first OWNER-vs-OWNER same-lane conflict in a bundle.

        Modifiers carry an empty lane set: they reserve nothing, run inline, and
        never conflict, so they are skipped and never counted. Lane disjointness
        is enforced only among owners. Returns ``(task, conflict_lanes)`` for the
        first conflicting owner, or ``(None, frozenset())`` when the bundle is
        valid.
        """
        seen_owner_lanes = set()
        for t in tasks:
            if not t.lanes:  # modifier: occupies no lane, always allowed
                continue
            conflict = seen_owner_lanes & t.lanes
            if conflict:
                return t, frozenset(conflict)
            seen_owner_lanes |= t.lanes
        return None, frozenset()

    def _handle_bundle(self, task_data):
        """Ingest a concurrent `tasks:[...]` bundle.

        The bundle-level `start_offset` anchors all sub-tasks to the same synced
        fleet instant; each sub-task may carry its own additive `start_offset`
        (float seconds) to stagger its lane after that synced start.
        """
        items = task_data['tasks']
        if not items:
            self.get_logger().error('Bundle rejected: empty tasks list')
            return

        # Self-filter BEFORE building tasks and BEFORE the lane-conflict check:
        # _bundle_lane_conflict scans ALL tasks, so two DRIVE owners aimed at
        # different Spheros would be falsely rejected as a lane conflict. Only
        # sub-tasks that target this unit survive (omitted targets -> all units).
        for item in items:
            raw_targets = item.get('targets')
            if raw_targets is not None and not isinstance(raw_targets, list):
                self.get_logger().warning(
                    f'Bundle sub-task has malformed targets {raw_targets!r}; '
                    f'treating as targeting all units'
                )
        items = [
            item for item in items
            if self._task_targets_me(self.sphero_name, item)
        ]
        if not items:
            self.get_logger().info(
                f'bundle: no tasks target {self.sphero_name}'
            )
            return

        now = task_data.get('now')
        bundle_start_offset = float(task_data.get('start_offset', 0.0) or 0.0)

        tasks = []
        for item in items:
            if 'task_type' not in item:
                self.get_logger().error('Bundle rejected: sub-task missing task_type')
                return
            item_start_offset = float(item.get('start_offset', 0.0))
            start_at = self._compute_start_at(
                now, bundle_start_offset + item_start_offset)
            tasks.append(self._build_task(item, start_at))

        # Validate owner lane disjointness (only OWNER-vs-OWNER same-lane is a
        # conflict; modifiers carry an empty lane set and never conflict).
        conflict_task, conflict_lanes = self._bundle_lane_conflict(tasks)
        if conflict_task is not None:
            self.get_logger().error(
                f'Bundle rejected: lane conflict on {sorted(conflict_lanes)} '
                f'(sub-task {conflict_task.task_id} / {conflict_task.task_type})'
            )
            return

        for t in tasks:
            self.task_executor.add_task(t)
            self.publish_task_status(t)

        # Per-task start delays may differ (staggered), so report each one.
        now_wall = time.time()
        starts = [
            f'{t.task_id}+{t.start_at - now_wall:.2f}s'
            if t.start_at is not None else f'{t.task_id} now'
            for t in tasks
        ]
        self.get_logger().info(
            f'Added bundle of {len(tasks)} tasks '
            f'({", ".join(t.task_type for t in tasks)}) to queue. '
            f'Starts: {", ".join(starts)}'
        )

    # ===== Fleet Policy (broadcast cues) =====

    def policy_callback(self, msg: FleetPolicy):
        """Apply a fleet-wide policy broadcast to this unit via the task stack.

        Phases 1-2 implement the Similarity (LED/matrix lanes) and Proximity
        (DRIVE lane) cues, which compose on disjoint lanes. Common Fate is
        accepted and logged but not acted on yet.
        """
        try:
            # Membership: raw callsigns (e.g. 'SB-33E3') normalized to compare
            # against our sanitized name (e.g. 'SB_33E3').
            is_member = any(
                m.replace('-', '_') == self.name_safe for m in msg.members)

            # Revoke (active=false) or not addressed to us -> tear everything down.
            if not msg.active or not is_member:
                self._clear_similarity()
                self._clear_proximity()
                self._active_policy_id = None
                return

            self._active_policy_id = msg.policy_id

            if msg.enable_similarity:
                self._apply_similarity(msg.similarity)
            else:
                self._clear_similarity()

            if msg.enable_proximity:
                # Deterministic member ordering (sort by safe callsign) so every
                # unit's centroid / slot math is identical.
                members = sorted(m.replace('-', '_') for m in msg.members)
                self._apply_proximity(msg.proximity, members)
            else:
                self._clear_proximity()

            if msg.enable_common_fate:
                self.get_logger().warning(
                    'common fate cue not yet implemented (phase 3)',
                    throttle_duration_sec=30.0)

        except Exception as e:
            self.get_logger().error(f'Error in policy callback: {str(e)}')

    def _apply_similarity(self, cue):
        """Route the Similarity cue into the task / state-machine stack.

        IDENTITY: steady visual render (surface chosen by content: RGB-only ->
        LED, pattern/custom-matrix -> matrix).
        BEHAVIOR: composable sub-behaviors -- `behavior_blink` runs the blink
        state machine on the visual surface, and `behavior_spin` enqueues a
        finite spin task on the DRIVE lane. Both may run at once (LED/matrix and
        DRIVE are disjoint lanes).
        """
        red, green, blue = int(cue.red), int(cue.green), int(cue.blue)
        pattern = cue.pattern or None
        matrix = list(cue.matrix) if len(cue.matrix) == 64 else None
        surface = 'matrix' if (pattern or matrix) else 'led'

        if cue.cue_type == SimilarityCue.BEHAVIOR:
            visual_mode = 'blink' if cue.behavior_blink else None
            want_spin = bool(cue.behavior_spin)
        else:  # IDENTITY: steady visual, no behaviors
            visual_mode = 'steady'
            want_spin = False

        # ----- Visual behavior (LED / matrix surface) -----
        # Leaving blink -> stop the blink state machine so it stops ticking. Clear
        # the SM on the surface lane the blink was running on (old surface).
        if self._policy_mode == 'blink' and visual_mode != 'blink':
            self._clear_blink_sm(self._policy_surface or 'led')
        # Blank the previously-lit surface if the visual moved or went away.
        new_surface = surface if visual_mode is not None else None
        if self._policy_surface is not None and self._policy_surface != new_surface:
            self._enqueue_policy_task(
                self._similarity_blank_task(self._policy_surface))

        if visual_mode == 'steady':
            self._enqueue_policy_task(
                self._similarity_render_task(surface, red, green, blue, pattern, matrix))
        elif visual_mode == 'blink':
            self._publish_blink_sm(
                self._similarity_render_task(surface, red, green, blue, pattern, matrix),
                self._similarity_blank_task(surface), cue.blink_hz, surface)

        self._policy_surface = new_surface
        self._policy_mode = visual_mode

        # ----- Spin behavior (DRIVE lane) -----
        if want_spin:
            self._apply_spin(cue)
        else:
            self._clear_spin()

    def _clear_similarity(self):
        """Tear down the Similarity cue deterministically.

        Fix for the revoke bug where the blink kept running on some members
        (their SM status kept changing while one unit terminated cleanly):

          * Stop the blink SM UNCONDITIONALLY -- not gated on the in-memory
            ``_policy_mode`` flag. That gate was the race: on a unit whose cached
            state did not say 'blink', the clear was never sent, so the SM kept
            ticking forever while a lone LED-blank could not stop it. Sending the
            clear on every revoke removes the dependence on cached state.
          * ``state_machine/control`` -> ``clear`` makes the SM controller HALT
            and DROP the running machine (``StateMachine.clear()`` empties the
            config so ``process()`` returns None), driving its status to
            ``configured: false`` -- a terminated machine, verifiable on
            ``sphero/<name>/state_machine/status``. Not just a dark LED.
          * Idempotent: clearing an already-empty SM is a harmless no-op, so a
            repeated revoke is safe.

        Order: stop the SM FIRST, then blank the surface, so the terminated SM
        cannot re-light after the blank. (Clear latency is << the blink half-
        period, so no stray transition fires in the window.)

        With SM lanes, the clear is scoped to the blink's own surface lane
        (led/matrix), so a user SM on another lane survives.
        """
        self._clear_blink_sm(self._policy_surface or 'led')
        # Blank the tracked surface; fall back to LED if state was lost so a
        # desynced unit still goes dark.
        self._enqueue_policy_task(
            self._similarity_blank_task(self._policy_surface or 'led'))
        self._policy_surface = None
        self._policy_mode = None
        self._clear_spin()

    def _similarity_render_task(self, surface, red, green, blue, pattern, matrix):
        """Build the on-surface render task dict for the cue."""
        if surface == 'led':
            return {
                'task_id': 'policy_similarity',
                'task_type': 'set_led',
                'parameters': {'red': red, 'green': green, 'blue': blue},
            }
        params = {'red': red, 'green': green, 'blue': blue}
        if matrix:
            params['matrix'] = matrix
        else:
            params['pattern'] = pattern or ''
        return {
            'task_id': 'policy_similarity',
            'task_type': 'matrix',
            'parameters': params,
        }

    @staticmethod
    def _similarity_blank_task(surface):
        """Build a task that blanks the given surface (LED off / matrix clear)."""
        if surface == 'led':
            return {
                'task_type': 'set_led',
                'parameters': {'red': 0, 'green': 0, 'blue': 0},
            }
        # Empty pattern + no custom matrix is the device's blank convention.
        return {'task_type': 'matrix', 'parameters': {'pattern': ''}}

    def _enqueue_policy_task(self, item):
        """Enqueue a policy-generated task onto our own executor."""
        task = self._build_task(item, None)
        self.task_executor.add_task(task)
        self.publish_task_status(task)

    def _publish_blink_sm(self, render, blank, blink_hz, lane):
        """Hand a blink cue to the state machine controller as a timer loop.

        Two states, ``on`` (render) and ``off`` (blank), alternate every
        half-period. One full on+off cycle completes at blink_hz; the rate is
        clamped to avoid divide-by-zero / absurd timer periods. The SM is tagged
        with its surface ``lane`` (led/matrix) so it runs concurrently with a
        Proximity SM on the DRIVE lane instead of clobbering a single slot.
        """
        half_period = 1.0 / (2.0 * max(float(blink_hz), 0.1))
        config = {
            'name': 'policy_similarity_blink',
            'lane': lane,
            'initial_state': 'on',
            'states': [
                {
                    'name': 'on',
                    'tasks': [render],
                    'exits': [{
                        'condition': {'type': 'timer', 'duration': half_period},
                        'destination': 'off',
                    }],
                },
                {
                    'name': 'off',
                    'tasks': [blank],
                    'exits': [{
                        'condition': {'type': 'timer', 'duration': half_period},
                        'destination': 'on',
                    }],
                },
            ],
        }
        msg = String()
        msg.data = json.dumps(config)
        self.sm_config_pub.publish(msg)

    def _clear_blink_sm(self, lane):
        """Clear the blink state machine on its surface ``lane`` (led/matrix)."""
        msg = String()
        msg.data = json.dumps({'action': 'clear', 'lane': lane})
        self.sm_control_pub.publish(msg)

    def _apply_spin(self, cue):
        """Apply the spin Behavioral Similarity sub-behavior as a finite DRIVE task.

        Path A: `spin` is a FINITE task (spin_duration_s takes precedence, else
        spin_rotations, else a long default). It owns the DRIVE lane, so it
        composes with the blink SM (LED/matrix lane). A persistent spin that
        lasts as long as the cue is active awaits the Phase-2 SM-lanes work.
        """
        speed = int(cue.spin_speed) if cue.spin_speed > 0 else 100
        duration = float(cue.spin_duration_s)
        rotations = int(cue.spin_rotations)

        params = {'speed': speed}
        if duration > 0:
            params['duration'] = duration
        elif rotations > 0:
            params['rotations'] = rotations
        else:
            params['duration'] = 30.0  # long default

        self._enqueue_policy_task({
            'task_id': 'policy_spin',
            'task_type': 'spin',
            'parameters': params,
        })
        self._policy_spin_active = True

    def _clear_spin(self):
        """Stop the spin behavior on the DRIVE lane if one was applied."""
        if self._policy_spin_active:
            self._enqueue_policy_task({'task_type': 'stop', 'parameters': {}})
            self._policy_spin_active = False

    # ===== Fleet Policy: Proximity cue (DRIVE lane) =====

    def _apply_proximity(self, cue: ProximityCue, members):
        """Route the Proximity cue into the state-machine / DRIVE stack.

        Subscribes this unit to every member's localization stream (so the step
        handler can read the live group snapshot) and hands the state machine
        controller a two-state timer SM on the ``drive`` lane whose every entry
        fires one ``proximity_step`` task. The per-unit computation (anchor,
        repulsion, settle) lives in ``execute_proximity_step``; the SM only
        provides the cadence.
        """
        self._subscribe_member_positions(members)

        anchor = int(cue.anchor)
        target_spacing = float(cue.target_spacing_cm) if cue.target_spacing_cm > 0 else 30.0
        tick_hz = float(cue.tick_hz) if cue.tick_hz > 0 else 3.0
        min_sep = float(cue.min_separation_cm) if cue.min_separation_cm > 0 else 15.0
        max_speed = int(cue.max_speed) if cue.max_speed > 0 else 60

        step_task = {
            'task_id': 'policy_proximity_step',
            'task_type': 'proximity_step',
            'parameters': {
                'me': self.name_safe,
                'members': members,
                'anchor': anchor,
                'anchor_x': float(cue.anchor_x),
                'anchor_y': float(cue.anchor_y),
                'target_spacing_cm': target_spacing,
                'min_separation_cm': min_sep,
                'max_speed': max_speed,
            },
        }
        self._publish_proximity_sm(step_task, tick_hz)
        self._policy_proximity_active = True

    def _clear_proximity(self):
        """Tear down the Proximity cue: halt the DRIVE SM, stop, unsubscribe."""
        if not self._policy_proximity_active and not self._member_position_subs:
            return
        self._clear_proximity_sm()
        # Stop any in-flight proximity roll on the DRIVE lane.
        self._enqueue_policy_task({'task_type': 'stop', 'parameters': {}})
        self._unsubscribe_member_positions()
        self._policy_proximity_active = False

    def _publish_proximity_sm(self, step_task, tick_hz):
        """Publish the two-state timer ping-pong SM (DRIVE lane) for proximity.

        Each state fires ``step_task`` on entry, then waits ``1/tick_hz`` before
        entering the other, so a step fires every ``1/tick_hz`` seconds.
        """
        period = 1.0 / max(float(tick_hz), 0.1)
        config = {
            'name': 'policy_proximity',
            'lane': 'drive',
            'initial_state': 'step_a',
            'states': [
                {
                    'name': 'step_a',
                    'tasks': [step_task],
                    'exits': [{
                        'condition': {'type': 'timer', 'duration': period},
                        'destination': 'step_b',
                    }],
                },
                {
                    'name': 'step_b',
                    'tasks': [step_task],
                    'exits': [{
                        'condition': {'type': 'timer', 'duration': period},
                        'destination': 'step_a',
                    }],
                },
            ],
        }
        msg = String()
        msg.data = json.dumps(config)
        self.sm_config_pub.publish(msg)

    def _clear_proximity_sm(self):
        """Clear the proximity state machine on the ``drive`` lane."""
        msg = String()
        msg.data = json.dumps({'action': 'clear', 'lane': 'drive'})
        self.sm_control_pub.publish(msg)

    def _subscribe_member_positions(self, members):
        """(Re)subscribe to ``/localization/<m>/position`` for each member.

        Drops subscriptions for members no longer in the set and adds new ones,
        so a re-broadcast with a changed roster reconciles cleanly.
        """
        wanted = set(members)
        for name_safe in list(self._member_position_subs):
            if name_safe not in wanted:
                self._destroy_member_sub(name_safe)

        for name_safe in members:
            if name_safe in self._member_position_subs:
                continue
            topic = f'/localization/{name_safe}/position'
            sub = self.create_subscription(
                PoseStamped, topic,
                lambda msg, n=name_safe: self._member_position_callback(n, msg),
                10,
                callback_group=self.callback_group,
            )
            self._member_position_subs[name_safe] = sub
            self.get_logger().info(f'Proximity: subscribed member position {topic}')

    def _unsubscribe_member_positions(self):
        """Drop every member localization subscription and clear the snapshot."""
        for name_safe in list(self._member_position_subs):
            self._destroy_member_sub(name_safe)
        self._member_positions = {}

    def _destroy_member_sub(self, name_safe):
        """Destroy one member subscription and forget its cached fix."""
        sub = self._member_position_subs.pop(name_safe, None)
        if sub is not None:
            self.destroy_subscription(sub)
        self._member_positions.pop(name_safe, None)

    def _member_position_callback(self, name_safe, msg: PoseStamped):
        """Cache a member's latest localization fix for the proximity handler."""
        self._member_positions[name_safe] = {
            'x': msg.pose.position.x,
            'y': msg.pose.position.y,
            't': time.time(),
        }

    def state_callback(self, msg: String):
        """Handle Sphero state updates."""
        try:
            self.current_state = json.loads(msg.data)

            # Update position
            if 'position' in self.current_state:
                self.current_position = self.current_state['position'].copy()

            # Update heading
            if 'motion' in self.current_state and 'heading' in self.current_state['motion']:
                self.current_heading = self.current_state['motion']['heading']

        except json.JSONDecodeError:
            pass

    def reset_aim_callback(self, msg: String):
        """Reset position and heading to origin."""
        self.current_position = {'x': 0.0, 'y': 0.0}
        self.current_heading = 0
        self.get_logger().info('Task controller reset to origin: heading=0°, position=(0, 0)')

    # ===== Task Execution =====

    def task_execution_loop(self):
        """Main task execution loop - called periodically."""
        # Snapshot the running SET (by identity) before processing so we can
        # report per-lane starts/finishes independently.
        previous = {id(t): t for t in self.task_executor.running_tasks()}

        # Process tasks (returns the list of running tasks, or None if idle).
        running = self.task_executor.process_tasks() or []
        current = {id(t): t for t in running}

        # Tasks that left the running set: finished or cancelled.
        for tid, task in previous.items():
            if tid not in current:
                self.publish_task_status(task)
                duration = (task.completed_at or task.started_at) - task.started_at
                self.get_logger().info(
                    f'Task {task.task_id} {task.status.value} in {duration:.2f}s'
                )

        # Tasks newly in the running set: started.
        for tid, task in current.items():
            if tid not in previous:
                self.publish_task_status(task)

                # Update position from state before starting.
                if 'position' in self.current_state:
                    self.current_position = self.current_state['position'].copy()
                    self.get_logger().info(
                        f'Starting task {task.task_id} at position: '
                        f'x={self.current_position["x"]:.2f}, '
                        f'y={self.current_position["y"]:.2f}'
                    )
                else:
                    self.get_logger().info(f'Starting task {task.task_id}')

    def publish_task_status(self, task: TaskDescriptor):
        """Publish task status update."""
        # Include queue + per-lane information.
        status_dict = task.to_dict()
        running = self.task_executor.running_tasks()
        status_dict['queue_length'] = len(self.task_executor.task_queue)
        status_dict['has_current_task'] = bool(running)
        status_dict['total_pending'] = len(self.task_executor.task_queue) + len(running)
        # Which lane slot holds which task_id (or None) so the dashboard can
        # show per-lane state.
        status_dict['running_lanes'] = {
            ln: (t.task_id if t else None)
            for ln, t in self.task_executor.current_tasks.items()
        }

        msg = String()
        msg.data = json.dumps(status_dict)
        self.task_status_pub.publish(msg)

    def cleanup(self):
        """Clean up resources before shutdown."""
        self.get_logger().info('Cleaning up Sphero instance task controller...')
        # Task controller has no hardware to clean up
        # Hardware cleanup is handled by the device controller
        self.get_logger().info('Task controller cleanup complete')


def main(args=None):
    """Main entry point for the Sphero instance task controller node."""
    rclpy.init(args=args)
    node = None
    temp_node = None
    shutdown_requested = False

    def signal_handler(_sig, _frame):
        nonlocal shutdown_requested
        print("\nKeyboard interrupt detected. Shutting down...")
        shutdown_requested = True

    # Register signal handler for SIGINT (Ctrl+C)
    signal.signal(signal.SIGINT, signal_handler)

    try:
        # Create a temporary node to read the sphero_name parameter
        temp_node = rclpy.create_node('temp_param_node')
        temp_node.declare_parameter('sphero_name', '')  # No default - REQUIRED parameter
        sphero_name = temp_node.get_parameter('sphero_name').value

        if not sphero_name:
            raise ValueError(
                "The 'sphero_name' parameter is required but was not provided. "
                "Please launch with: ros2 run sphero_instance_controller sphero_instance_task_controller_node.py "
                "--ros-args -p sphero_name:=<YOUR_SPHERO_NAME>"
            )

        print(f"Sphero name from parameter: {sphero_name}")

        # Destroy temporary node before creating controller node
        temp_node.destroy_node()
        temp_node = None

        # Create the task controller node (no hardware connection needed)
        print(f"Initializing task controller for {sphero_name}...")
        node = SpheroInstanceTaskController(sphero_name)
        print(f"Task controller initialized for {sphero_name}")

        # Spin until shutdown requested
        while rclpy.ok() and not shutdown_requested:
            try:
                rclpy.spin_once(node, timeout_sec=0.1)
            except rclpy.executors.ExternalShutdownException:
                # Expected during shutdown - ignore
                break

    except KeyboardInterrupt:
        print("\nShutting down...")
    except rclpy.executors.ExternalShutdownException:
        # Expected during external shutdown - ignore
        pass
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()

    finally:
        # Clean up temporary node if it exists
        if temp_node:
            temp_node.destroy_node()

        # Clean up the controller node
        if node:
            node.cleanup()
            node.destroy_node()

        # Shutdown ROS 2
        if rclpy.ok():
            rclpy.shutdown()

        print("Goodbye!")


if __name__ == '__main__':
    main()
