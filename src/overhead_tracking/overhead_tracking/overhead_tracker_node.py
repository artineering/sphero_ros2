#!/usr/bin/env python3
"""Overhead global-shutter multi-Sphero tracker.

Operator-staged workflow, one service per stage:

    ~/detect_arena    find the arena corners -> homography (px -> field cm)
    ~/detect_spheros  snapshot the robot blobs currently on the arena
    ~/link_spheros    binds each blob to a callsign: serial LED probe
                      (link_method:=probe) or solo nudges (link_method:=nudge)
    (tracking starts automatically once at least one callsign is linked)

Steady state, driven by the camera's frames on a processing thread (every frame
at track_rate_hz 60, every second one at 30): detect blobs ONCE for the whole
gray frame -- brighter than floor_ratio x the local floor (`blobs.py`), against a
threshold map kept by `floor_map.py` and rebuilt when the room brightness drifts
-- read the LED heading of each uncrowded blob from the SAME frame's half-res
colour, decoded in parallel by `color_decode.ColorWorker`, and hand them to one
`TrackManager`
(`track_manager.py`): per robot a heading EKF driven by its motion_cmd echoes
(`ekf.py`), a covariance-scaled gate, merge/crossing handling and a miss ladder
in seconds of frame time.

Output is `/localization/<name_safe>/position` in field CENTIMETRES -- the
existing contract the device and task controllers already consume, orientation
= EKF yaw once the LEDs gave a heading -- `/localization/<name_safe>/heading_offset`
(the robot's aim offset), plus an annotated JPEG for operators and rosbag.

Why one node and not the usual capture/algorithm split: 1280x720 mono at 200 fps
is 184 MB/s, which is not something to push through DDS on a Pi 5. The camera is
owned directly through a duck-typed pull source so the sim swap still works.
"""

import csv
import json
import math
import os
import queue
import threading
import time
from collections import deque

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from multirobot_msgs.msg import FleetState
from multirobot_msgs.srv import Register
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, \
    qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Bool, Float32, String
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker, MarkerArray

from . import annotate as ann
from . import arena as arena_mod
from . import blobs as bl
from . import color_decode as cdec
from . import heading as hdg
from . import homography as hg
from . import identify as ident
from . import nudge as ndg
from .arena_store import ArenaCalibration, load_arena, save_arena
from .camera import build_source
from .ekf import HeadingOffsetEstimator
from .floor_map import FloorMap
from .markers import build_arena_marker, build_track_markers
from .track_manager import (COASTING, HEALTHY, LOST, MERGED, SUSPECT,  # noqa: F401
                            UNRESOLVED, Blob, TrackManager)

# states
INIT = 'INIT'
IDLE = 'IDLE'
ARENA_READY = 'ARENA_READY'
BLOBS_READY = 'BLOBS_READY'
LINKING = 'LINKING'
TRACKING = 'TRACKING'

# per-track status: HEALTHY, COASTING, MERGED, LOST, UNRESOLVED, SUSPECT --
# owned by track_manager, re-exported above

# processing rates (track_rate_hz) and how many camera frames each one steps
FRAME_STEP = {60.0: 1, 30.0: 2}

# link_seed_csv: a robot's first track.csv row must be this close to a blob
SEED_TOL_CM = 15.0
# /localization/<ns>/heading_offset: publish on a change of more than this many
# degrees, at most once per period (frame time)
OFFSET_PUBLISH_DEG = 1.0
OFFSET_PUBLISH_PERIOD_S = 1.0


def name_safe(name):
    """SB-33E3 -> SB_33E3 (topic-safe)."""
    return str(name).replace('-', '_')


def latched_qos():
    return QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                      history=QoSHistoryPolicy.KEEP_LAST)


class OverheadTrackerNode(Node):

    def __init__(self, **kwargs):
        # kwargs forwarded to rclpy Node -- notably parameter_overrides, which is
        # how the tests configure a sim source without a YAML file
        super().__init__('overhead_tracker_node', **kwargs)
        self._declare_params()
        gp = self.get_parameter

        # ---- camera / robot geometry / detection
        self._source_kind = gp('source').value
        self._width = int(gp('frame_width').value)
        self._height = int(gp('frame_height').value)
        self._ball_d = int(gp('ball_diameter_px').value)
        self._pt_sep = int(gp('point_sep_px').value)
        self._led_search = int(gp('led_search_px').value)
        self._led_sat_min = int(gp('led_sat_min').value)
        self._led_val_min = int(gp('led_val_min').value)
        self._led_color_space = str(gp('led_color_space').value)
        if self._led_color_space not in cdec.MODES:
            self.get_logger().warning(
                f'led_color_space {self._led_color_space!r} not one of {cdec.MODES}; '
                'using bgr')
            self._led_color_space = 'bgr'
        self._blob_min_area = int(gp('blob_min_area_px').value)
        self._blob_link_px = int(gp('blob_link_px').value)
        self._blob_margin = int(gp('blob_roi_margin_px').value)
        self._floor_ratio = float(gp('floor_ratio').value)
        self._floor_drift = float(gp('floor_drift_frac').value)
        self._floor_hold = float(gp('floor_drift_hold_sec').value)
        self._floor_frames = int(gp('floor_rebuild_frames').value)
        self._floor = None            # FloorMap; replaced (never mutated) when the ROI changes
        self._max_robots = int(gp('max_robots').value)

        # ---- rates. Processing is driven by the camera's frames: 60 = every
        # frame, 30 = every second one (by frame seq, so each processed frame
        # keeps its own capture stamp). Camera_fps is the same either way.
        self._track_rate = float(gp('track_rate_hz').value)
        if self._track_rate not in FRAME_STEP:
            self.get_logger().warning(
                f'track_rate_hz {self._track_rate:g} not supported (60 or 30); using 60')
            self._track_rate = 60.0
        self._frame_step = FRAME_STEP[self._track_rate]
        self._publish_rate = float(gp('publish_rate_hz').value)
        self._annotate_rate = float(gp('annotate_rate_hz').value)
        self._jpeg_quality = int(gp('jpeg_quality').value)
        self._jpeg_min_quality = int(gp('annotate_min_quality').value)
        self._annotate_scale = float(gp('annotate_scale').value)

        # ---- tracking
        self._extrapolate = bool(gp('extrapolate_to_now').value)
        self._arena_margin_cm = float(gp('arena_margin_cm').value)

        # ---- arena
        self._arena_yaml = gp('arena_yaml_path').value or os.path.expanduser(
            '~/overhead_field/overhead_arena.yaml')
        self._arena_source = gp('arena_source').value
        self._long_cm = float(gp('arena_long_edge_cm').value)
        self._short_cm = float(gp('arena_short_edge_cm').value)
        self._swap_axes = bool(gp('arena_swap_axes').value)
        self._residual_tol = float(gp('homography_residual_tol_cm').value)
        self._arena_frames = int(gp('arena_detect_frames').value)

        # ---- linking
        self._probe_rgb = (int(gp('probe_red').value), int(gp('probe_green').value),
                           int(gp('probe_blue').value))
        self._lit_delta = float(gp('lit_delta_thresh').value)
        self._lit_min_area = int(gp('lit_min_area_px').value)
        self._link_tol_px = float(gp('link_tol_px').value)
        self._render_timeout = float(gp('render_timeout_seconds').value)
        self._off_timeout = float(gp('off_timeout_seconds').value)
        self._probe_poll = float(gp('probe_poll_seconds').value)
        self._reset_settle = float(gp('reset_settle_seconds').value)
        self._heartbeat_fresh = float(gp('heartbeat_fresh_sec').value)
        self._aim_tol = float(gp('aim_tolerance_deg').value)
        self._aim_settle = float(gp('aim_settle_seconds').value)
        self._sphere_radius_mm = float(gp('sphere_radius_mm').value)
        self._link_method = str(gp('link_method').value)
        # user rule: never command a Sphero below speed 50 (dead band below ~36)
        self._nudge_speed = max(50, int(gp('nudge_speed').value))
        self._nudge_duration = float(gp('nudge_duration_sec').value)
        self._nudge_settle = float(gp('nudge_settle_sec').value)
        self._nudge_min_move = float(gp('nudge_min_move_px').value)
        self._nudge_timeout = float(gp('nudge_timeout_sec').value)
        # (stamp, name, speed) motion_cmd echoes, recorded only while nudge
        # linking is running (None otherwise, so it cannot grow unbounded)
        self._nudge_log = None
        self._nudge_lock = threading.Lock()

        self._cmd_speed_to_cms = float(gp('cmd_speed_to_cms').value)
        self._seed_csv = str(gp('link_seed_csv').value or '')
        self._mgr_kwargs = dict(
            gate_sigma=float(gp('gate_sigma').value),
            gate_min_cm=float(gp('gate_min_cm').value),
            gate_max_cm=float(gp('gate_max_cm').value),
            lost_after_s=float(gp('lost_after_s').value),
            unresolved_after_s=float(gp('unresolved_after_s').value),
            ambiguity_d2=float(gp('ambiguity_d2').value),
            reacquire_ratio=float(gp('reacquire_ratio').value),
            merge_area_frac=float(gp('merge_area_frac').value),
            ball_d_cm=2.0 * float(gp('sphere_radius_mm').value) / 10.0,
            min_sep_cm=float(gp('min_sep_cm').value),
            first_led_n=int(gp('first_led_n').value),
            first_led_window_s=float(gp('first_led_window_s').value),
            first_led_tol_deg=float(gp('first_led_tol_deg').value),
            ekf_kwargs=dict(q_pos=float(gp('q_pos').value),
                            q_th=float(gp('q_th').value),
                            q_pos_no_heading=float(gp('q_pos_no_heading').value),
                            r_pos=float(gp('r_pos').value),
                            r_th=float(gp('r_th').value)),
            cmd_kwargs=dict(lag_v_sec=float(gp('lag_v_sec').value),
                            turn_rate_dps=float(gp('turn_rate_dps').value),
                            speed_to_cms=self._cmd_speed_to_cms))

        # OpenCV's internal parallel-for would oversubscribe the cores against our
        # own threads and make the threading a net loss. Must be set before any work.
        cv2.setNumThreads(int(gp('opencv_num_threads').value))

        # ---- state
        self._state = INIT
        self._state_lock = threading.Lock()
        self._arena = None                 # ArenaCalibration
        self._arena_H = None
        self._arena_Hinv = None
        self._arena_poly_cm = None
        self._arena_lock = threading.Lock()
        self._blobs = []                   # staged snapshot from detect_spheros
        self._blobs_lock = threading.Lock()
        # Every robot's track lives in ONE TrackManager. _tracks_lock guards it
        # and the per-robot dicts below: the processing loop steps it, services add tracks,
        # motion_cmd callbacks (another thread) append commands.
        self._mgr = TrackManager(**self._mgr_kwargs)
        self._tracks_lock = threading.Lock()
        self._last_blob = {}               # name -> u, v, area, split of its last blob
        self._miss = {}                    # name -> processed frames since that blob
        self._offset = {}                  # name -> HeadingOffsetEstimator
        self._offset_pub_t = {}            # name -> frame stamp of the last offset publish
        self._conn_state = {}              # name -> last sphero/<ns>/status connection_state
        self._links = {}                   # name -> blob index
        self._registered, self._failed = [], []
        self._marker_ids = {}
        self._compass_done = {}
        self._fleet_last_seen = {}
        self._fleet_name_safe = {}
        self._pose_pubs, self._led_pubs = {}, {}
        self._matrix_pubs, self._compass_pubs = {}, {}
        self._offset_pubs = {}
        self._roll_pubs = {}
        self._heading_pubs, self._aim_pubs = {}, {}
        self._compass_subs, self._motion_subs = {}, {}
        self._aim_subs, self._status_subs = {}, {}

        self._last_stamp = 0.0
        self._last_pub = 0.0
        self._last_proc_seq = None         # seq of the last processed frame
        self._proc_ms = deque(maxlen=600)  # process time of recent frames
        self._proc_n, self._proc_t0, self._proc_fps = 0, time.time(), 0.0
        self._diag = {'process_ms': 0.0, 'detect_ms': 0.0, 'track_ms': 0.0,
                      'n_comps': 0, 'color_late': 0, 'led_crowded': 0,
                      'annotate_drops': 0, 'annotate_q': 0}

        # ---- camera source
        controls = {'auto_exposure': int(gp('v4l2_auto_exposure').value),
                    'gain': int(gp('v4l2_gain').value),
                    'white_balance_automatic':
                        int(gp('v4l2_white_balance_automatic').value)}
        exp = int(gp('v4l2_exposure_time_absolute').value)
        if exp > 0:
            # 0 means "leave it": at 200 fps the 5 ms frame period clamps exposure
            # anyway, which is why every value from 500 up produced identical
            # frames on the bench. Set it only when overriding deliberately.
            controls['exposure_time_absolute'] = exp
        for kv in str(gp('v4l2_controls_extra').value or '').split(','):
            if '=' in kv:
                k, v = kv.split('=', 1)
                controls[k.strip()] = v.strip()
        self._camera_controls = controls
        self._expected_fps = float(gp('camera_fps').value)
        self._fps_tol = float(gp('fps_warn_tolerance').value)
        self._camera_stale_warn = float(gp('camera_stale_warn_sec').value)
        self._camera_restart = float(gp('camera_restart_sec').value)

        # A replay is offline: nothing may reach /sphero/*. Its robots and their
        # commands come from the recording's cmds.csv instead.
        self._replay = self._source_kind == 'replay'
        try:
            self._source = build_source(
                self._source_kind, device=gp('camera_device').value,
                width=self._width, height=self._height, fps=self._expected_fps,
                fourcc=gp('fourcc').value, controls=controls,
                n=int(gp('sim_spheros').value), seed=int(gp('seed').value),
                ball_d=self._ball_d, replay_path=gp('replay_path').value,
                replay_rate=float(gp('replay_rate').value),
                replay_loop=bool(gp('replay_loop').value),
                replay_lockstep=bool(gp('replay_lockstep').value),
                logger=self.get_logger())
            if self._replay:
                # recorded motion_cmd echoes, through the same handler
                self._source.cmd_handler = self._on_replay_cmd
            self._source.start()
        except Exception as e:                                  # noqa: BLE001
            self.get_logger().error(f'camera source failed to start: {e}')
            self._source = None

        # ---- callback groups
        self._service_group = MutuallyExclusiveCallbackGroup()
        self._sub_group = ReentrantCallbackGroup()
        self._annotate_group = MutuallyExclusiveCallbackGroup()

        # ---- LED colour: half-res decode of each processed frame's own bytes
        self._color = cdec.ColorWorker(self._led_color_space)

        # ---- publishers
        self._annot_pub = self.create_publisher(
            CompressedImage, '~/annotated/compressed', qos_profile_sensor_data)
        # The camera's own MJPEG bytes, every frame, not re-encoded: full video
        # in a rosbag. Off by default (~3-12 MB/s at 60 fps).
        self._raw_pub = None
        if bool(gp('publish_raw_jpeg').value):
            self._raw_pub = self.create_publisher(
                CompressedImage, '~/camera/compressed', qos_profile_sensor_data)
        self._arena_pub = self.create_publisher(Marker, '~/arena_corners',
                                                latched_qos())
        self._track_pub = self.create_publisher(MarkerArray, '~/track_markers', 10)
        self._state_pub = self.create_publisher(String, '~/state', latched_qos())
        # Everything needed to reconstruct an annotated frame offline from a bag:
        # pixel position, field position, heading and blob area per robot.
        self._det_pub = self.create_publisher(String, '~/detections', 10)
        self._link_pub = self.create_publisher(String, '~/link_status',
                                               latched_qos())

        # ---- subscriptions
        self.create_subscription(FleetState, '/sphero_fleet/robots',
                                 self._on_fleet, latched_qos(),
                                 callback_group=self._sub_group)

        # ---- services
        self.create_service(Trigger, '~/detect_arena', self._on_detect_arena,
                            callback_group=self._service_group)
        self.create_service(Trigger, '~/detect_spheros', self._on_detect_spheros,
                            callback_group=self._service_group)
        self.create_service(Register, '~/link_spheros', self._on_link_spheros,
                            callback_group=self._service_group)
        self.create_service(Trigger, '~/reset', self._on_reset,
                            callback_group=self._service_group)
        self.create_service(Trigger, '~/reapply_camera_controls',
                            self._on_reapply_controls,
                            callback_group=self._service_group)
        self.create_service(Trigger, '~/save_annotated',
                            self._on_save_annotated,
                            callback_group=self._service_group)
        self.create_service(Trigger, '~/rebuild_floor', self._on_rebuild_floor,
                            callback_group=self._service_group)

        # ---- annotate worker (bounded FIFO; processing never draws or encodes)
        self._annot_q = queue.Queue(maxsize=int(gp('annotate_queue_max').value))
        self._annot_stop = threading.Event()
        self._annot_thread = threading.Thread(target=self._annotate_loop,
                                              daemon=True, name='annotate')
        self._annot_thread.start()
        self._last_annot_push = 0.0
        self._annot_seq = 0

        # ---- frame processing: its own thread, started by start_processing()
        # (main) so tests can drive _on_frame directly without a racing loop
        self._proc_stop = threading.Event()
        self._proc_thread = None

        # ---- timers
        self.create_timer(1.0 / max(self._annotate_rate, 0.1),
                          self._annotate_push_timer,
                          callback_group=self._annotate_group)
        self.create_timer(1.0, self._publish_state,
                          callback_group=self._annotate_group)
        self.create_timer(1.0 / max(float(gp('marker_republish_hz').value), 0.1),
                          self._republish_arena_marker,
                          callback_group=self._annotate_group)
        delay = float(gp('control_reapply_delay_sec').value)
        self._ctl_timer = self.create_timer(max(delay, 0.05),
                                            self._deferred_controls,
                                            callback_group=self._annotate_group)

        self._startup_load()
        self.get_logger().info(
            f'overhead_tracker_node up: source={self._source_kind} '
            f'{self._width}x{self._height} track={self._track_rate}Hz '
            f'publish={self._publish_rate}Hz state={self._state}')

    # ------------------------------------------------------------------ params
    def _declare_params(self):
        from rcl_interfaces.msg import ParameterDescriptor
        d = self.declare_parameter
        # camera
        d('source', 'v4l2')
        d('camera_device', '/dev/video0')
        d('frame_width', 1920)
        d('frame_height', 1200)
        d('camera_fps', 120.0)
        d('fourcc', 'MJPG')
        d('v4l2_auto_exposure', 1)
        d('v4l2_gain', 100)
        d('v4l2_white_balance_automatic', 0)
        d('v4l2_exposure_time_absolute', 0)
        d('v4l2_controls_extra', '')
        d('replay_path', '')
        d('replay_rate', 1.0)
        d('replay_loop', False)
        d('replay_lockstep', False)
        d('publish_raw_jpeg', False)
        d('control_reapply_delay_sec', 1.5)
        d('fps_warn_tolerance', 0.15)
        d('camera_stale_warn_sec', 1.0)
        d('camera_restart_sec', 5.0)
        d('opencv_num_threads', 1)
        # robot geometry / detection
        d('ball_diameter_px', 34)
        d('point_sep_px', 26)
        d('floor_ratio', 3.2)
        d('blob_roi_margin_px', 25)
        d('blob_min_area_px', 200)
        d('blob_link_px', 25)
        d('floor_drift_frac', 0.20)
        d('floor_drift_hold_sec', 2.0)
        d('floor_rebuild_frames', 5)
        d('led_search_px', 30)
        d('led_sat_min', 90)
        d('led_val_min', 60)
        d('led_color_space', 'bgr')
        d('max_robots', 16)
        # rates / annotation
        d('track_rate_hz', 60.0)
        d('publish_rate_hz', 0.0)
        d('annotate_rate_hz', 15.0)
        d('jpeg_quality', 70)
        d('annotate_min_quality', 40)
        d('annotate_scale', 1.0)
        d('annotate_queue_max', 5)
        d('marker_republish_hz', 1.0)
        # tracking (track_manager.TrackManager defaults)
        d('gate_sigma', 3.0)
        d('gate_min_cm', 15.0)
        d('gate_max_cm', 50.0)
        d('lost_after_s', 0.5)
        d('unresolved_after_s', 2.0)
        d('ambiguity_d2', 4.0)
        d('reacquire_ratio', 0.5)
        d('merge_area_frac', 1.3)
        d('min_sep_cm', 6.0)
        d('first_led_n', 8)
        d('first_led_window_s', 0.5)
        d('first_led_tol_deg', 15.0)
        d('extrapolate_to_now', True)
        # arena
        d('arena_yaml_path', '')
        d('auto_load_arena', True)
        d('arena_source', 'manual')
        d('manual_corners_px', [], ParameterDescriptor(dynamic_typing=True))
        d('arena_long_edge_cm', 0.0)
        d('arena_short_edge_cm', 0.0)
        d('arena_swap_axes', False)
        d('arena_margin_cm', 5.0)
        d('arena_detect_frames', 5)
        d('homography_residual_tol_cm', 0.5)
        d('canny_sigma', 0.33)
        d('morph_close_iters', 3)
        d('border_margin_px', 6)
        d('min_quad_area_frac', 0.05)
        d('max_quad_area_frac', 0.95)
        d('approx_eps_frac', 0.02)
        d('arena_hue_lo', 90)
        d('arena_hue_hi', 130)
        # linking
        d('probe_red', 255)
        d('probe_green', 255)
        d('probe_blue', 255)
        d('lit_delta_thresh', 40.0)
        d('lit_min_area_px', 20)
        d('link_tol_px', 25.0)
        d('render_timeout_seconds', 6.0)
        d('off_timeout_seconds', 4.0)
        d('probe_poll_seconds', 0.2)
        d('reset_settle_seconds', 2.0)
        d('heartbeat_fresh_sec', 15.0)
        d('skip_compass_default', True)
        d('aim_tolerance_deg', 8.0)
        d('aim_settle_seconds', 1.5)
        d('compass_timeout_seconds', 25.0)
        d('link_method', 'probe')
        d('nudge_speed', 50)
        d('nudge_duration_sec', 1.0)
        d('nudge_settle_sec', 1.5)
        d('nudge_min_move_px', 5.0)
        d('nudge_timeout_sec', 8.0)
        d('link_seed_csv', '')
        # motion model (ekf.HeadingEKF / ekf.CommandHistory defaults)
        d('q_pos', 100.0)
        d('q_th', 0.3)
        d('q_pos_no_heading', 500.0)
        d('r_pos', 0.05)
        d('r_th', 0.001)
        d('lag_v_sec', 0.45)
        d('turn_rate_dps', 180.0)
        d('cmd_speed_to_cms', 0.61)
        d('debug_positions', False)
        d('sphere_radius_mm', 36.5)
        # sim
        d('sim_spheros', 5)
        d('seed', 0)

    # ------------------------------------------------------------- state utils
    def _get_state(self):
        with self._state_lock:
            return self._state

    def _set_state(self, s):
        with self._state_lock:
            changed = self._state != s
            self._state = s
        if changed:
            self.get_logger().info(f'state -> {s}')
            self._publish_state()

    def _startup_load(self):
        if not self.get_parameter('auto_load_arena').value:
            return
        if not os.path.exists(self._arena_yaml):
            return
        try:
            cal = load_arena(self._arena_yaml)
            self._apply_arena(cal)
            self.get_logger().info(
                f'loaded arena from {self._arena_yaml}: '
                f'{cal.long_edge_len_cm}x{cal.short_edge_len_cm} cm')
        except Exception as e:                                  # noqa: BLE001
            self.get_logger().warning(f'arena load failed: {e}')

    def _apply_arena(self, cal):
        H = cal.H_np
        # A new FloorMap, built lazily (first processed frame / detect_spheros) or by
        # ~/detect_arena: _startup_load runs before the camera has frames.
        shape = (cal.frame_height or self._height, cal.frame_width or self._width)
        roi = bl.make_roi(cal.corners_px, shape, margin=self._blob_margin)
        fm = FloorMap(roi, ratio=self._floor_ratio, drift_frac=self._floor_drift,
                      drift_hold_sec=self._floor_hold)
        with self._arena_lock:
            self._arena = cal
            self._arena_H = H
            self._arena_Hinv = np.linalg.inv(H)
            self._arena_poly_cm = cal.corners_cm_np
            self._floor = fm               # processing reads self._floor once per frame
        self._republish_arena_marker()
        if self._get_state() in (INIT, IDLE):
            self._set_state(ARENA_READY)

    def _arena_snapshot(self):
        with self._arena_lock:
            return (self._arena, self._arena_H, self._arena_Hinv,
                    self._arena_poly_cm)

    # -------------------------------------------------------------- floor map
    def _rebuild_floor_now(self):
        """Synchronous map build from floor_rebuild_frames fresh frames.
        Service threads only (FloorMap.rebuild blocks); processing never calls it."""
        fm = self._floor
        if fm is None:
            return False, 'no arena'
        frames = [g for g, _s in (self._grab_fresh(1.0)
                                  for _ in range(max(1, self._floor_frames)))
                  if g is not None]
        if not frames:
            return False, 'no camera frame'
        fm.rebuild(frames)
        return True, json.dumps(fm.state())

    def _on_rebuild_floor(self, _req, resp):
        """Rebuild the detection floor map now -- after a lighting, gain or
        exposure change. Robots may stay on the floor: the 61 px opening removes
        them. Allowed while TRACKING; the old map stays in use until the swap."""
        if self._floor is None:
            resp.success, resp.message = False, 'no arena; call ~/detect_arena first'
            return resp
        resp.success, resp.message = self._rebuild_floor_now()
        return resp

    def _observe_floor(self, gray, stamp):
        """Processing thread: track the floor level; build the first map lazily and
        rebuild in the background on a sustained brightness drift."""
        fm = self._floor
        if fm is None or gray is None:
            return
        if fm.tmap is None:
            fm.observe(gray, stamp)
            fm.start_rebuild()                 # first map (startup / auto-load)
        elif fm.observe(gray, stamp) and fm.start_rebuild():
            self.get_logger().info(
                f'floor drift {fm.level:.1f} vs {fm.ref_level:.1f}: rebuilding')

    def _ref_area(self):
        """Merged-blob split reference: median link-time area of the tracks,
        else None (blobs.detect then falls back to the frame median)."""
        with self._tracks_lock:
            return self._mgr.ref_area

    def _detect_blobs(self, gray, H):
        """Robot blobs in one frame, as dicts of u, v (px), area (px), split
        and x_cm, y_cm -- the one px -> cm conversion point. Empty with no
        floor map yet. The detector applies the arena polygon + margin."""
        fm = self._floor
        if fm is None or fm.tmap is None or gray is None or H is None:
            return []
        raw = bl.detect(gray, fm.roi, fm.tmap, self._blob_min_area,
                        self._blob_link_px, ref_area=self._ref_area())
        if not raw:
            return []
        cm = hg.apply_homography_many([b['Centroid'] for b in raw], H)
        return [{'u': b['Centroid'][0], 'v': b['Centroid'][1], 'area': b['Area'],
                 'split': b['Split'], 'x_cm': float(x), 'y_cm': float(y)}
                for b, (x, y) in zip(raw, cm)]

    # ----------------------------------------------------------------- camera
    def _pump_source(self):
        """Render a sim frame for a service waiting on a fresh one. No-op for
        v4l2 and replay (their grab threads run free)."""
        if self._source is not None:
            try:
                self._source.on_tick()
            except Exception as e:                              # noqa: BLE001
                self.get_logger().warning(f'source tick failed: {e}')

    def _deferred_controls(self):
        """Apply camera controls AFTER streaming has begun.

        UVC discards manual exposure at stream start, so setting controls before
        or during start silently does nothing. Fires once, then cancels itself.
        """
        self._ctl_timer.cancel()
        if self._source is None:
            return
        try:
            self._source.apply_controls()
        except Exception as e:                                  # noqa: BLE001
            self.get_logger().warning(f'apply_controls failed: {e}')
        if self._current_frame() is not None and self._get_state() == INIT:
            self._set_state(ARENA_READY if self._arena is not None else IDLE)

    def _latest_frame(self):
        """(gray, stamp) of the latest frame. Lockstep replay: this PULLS the
        next frame; _current_frame() does not."""
        if self._source is None:
            return None, 0.0
        return self._source.latest_gray()

    def _current_frame(self):
        """The current Frame without pulling one, or None."""
        return None if self._source is None else self._source.current_frame()

    def _led_heading(self, color, u, v):
        """0-360 field heading from the LED pair near full-res pixel (u, v) in
        `color` (ColorWorker output for the same frame), or None unless both
        the back (red) and front (green) LEDs are visible -- or there is no
        colour for the frame. The EKF carries theta between fixes."""
        if color is None:
            return None
        return cdec.led_heading(color, u, v, is_hsv=self._color.is_hsv,
                                search_px=self._led_search,
                                min_sep_px=self._pt_sep / 2.0,
                                sat_min=self._led_sat_min, val_min=self._led_val_min)

    def _grab_fresh_frame(self, timeout=1.0):
        """Block until a NEW frame arrives; that Frame, else the current one
        (None if there is none) after `timeout`."""
        t0 = time.time()
        f0 = self._current_frame()
        while time.time() - t0 < timeout:
            self._pump_source()
            f = self._source.latest_frame() if self._source is not None else None
            if f is not None and (f0 is None or f.seq != f0.seq):
                return f
            time.sleep(0.005)
        return self._current_frame()

    def _grab_fresh(self, timeout=1.0):
        """(gray, stamp) of _grab_fresh_frame."""
        f = self._grab_fresh_frame(timeout)
        return (None, 0.0) if f is None else (f.gray, f.stamp)

    def _grab_median(self, n=5, color=False):
        """Median of n distinct frames -- suppresses sensor noise for calibration.

        color=True returns full-res BGR, decoded from each frame's bytes, when
        the source has them. The arena detector ORs Canny over luminance AND
        the two LAB chroma channels, and coloured tape on a neutral floor is a
        strong CHROMA edge but a weak brightness one -- a gray frame silently
        disables the two channels that matter. Sources with no bytes (sim)
        fall back to gray.
        """
        frames = [f for f in (self._grab_fresh_frame(0.5) for _ in range(max(1, n)))
                  if f is not None]
        if not frames:
            return None
        if color and all(f.jpeg is not None for f in frames):
            imgs = [cv2.imdecode(f.jpeg, cv2.IMREAD_COLOR) for f in frames]
        else:
            imgs = [f.gray for f in frames]
        return np.median(np.stack(imgs), axis=0).astype(np.uint8)

    def _on_reapply_controls(self, _req, resp):
        if self._source is None:
            resp.success, resp.message = False, 'no camera source'
            return resp
        try:
            readback = self._source.apply_controls()
            resp.success = True
            resp.message = json.dumps(readback)
        except Exception as e:                                  # noqa: BLE001
            resp.success, resp.message = False, str(e)
        return resp

    # ---------------------------------------------------------------- roster
    def _on_fleet(self, msg):
        now = time.time()
        for r in msg.robots:
            self._fleet_name_safe[r.name] = r.name_safe or name_safe(r.name)
            self._fleet_last_seen[r.name] = r.last_seen or now

    def _ns(self, name):
        return self._fleet_name_safe.get(name, name_safe(name))

    # =====================================================================
    # Stage 1 -- arena
    # =====================================================================
    def _on_detect_arena(self, _req, resp):
        if self._get_state() == TRACKING:
            resp.success = False
            resp.message = ('refusing while TRACKING: a new homography would '
                            'teleport every live track. Call ~/reset first.')
            return resp
        try:
            ok, msg, cal = self._run_arena_detect()
        except Exception as e:                                  # noqa: BLE001
            self.get_logger().error(f'detect_arena raised: {e}')
            ok, msg, cal = False, f'exception: {e}', None
        if ok and cal is not None:
            self._apply_arena(cal)
            try:
                save_arena(self._arena_yaml, cal)
            except Exception as e:                              # noqa: BLE001
                self.get_logger().warning(f'arena save failed: {e}')
            # build the floor map now so ~/detect_spheros works at once; the
            # arena result stands either way
            fok, fmsg = self._rebuild_floor_now()
            if not fok:
                self.get_logger().warning(f'floor map not built: {fmsg}')
        resp.success, resp.message = ok, msg
        return resp

    def _run_arena_detect(self):
        if self._long_cm <= 0 or self._short_cm <= 0:
            return (False, 'arena_long_edge_cm / arena_short_edge_cm must be set '
                           '(measure the arena)', None)
        frame = self._grab_median(self._arena_frames, color=True)
        if frame is None:
            return False, 'no camera frame', None
        gray = (frame if frame.ndim == 2
                else cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY))

        manual = list(self.get_parameter('manual_corners_px').value or [])
        if self._arena_source == 'manual' or len(manual) == 8:
            if len(manual) != 8:
                return (False, 'arena_source=manual but manual_corners_px is not '
                               '8 numbers [u0,v0,u1,v1,u2,v2,u3,v3]', None)
            corners = np.asarray(manual, dtype=float).reshape(4, 2)
            src = 'manual'
        else:
            gp = self.get_parameter
            corners, diag = arena_mod.detect_arena_quad(
                frame, approx_eps_frac=float(gp('approx_eps_frac').value),
                canny_sigma=float(gp('canny_sigma').value),
                close_iters=int(gp('morph_close_iters').value),
                border_margin=int(gp('border_margin_px').value),
                min_area_frac=float(gp('min_quad_area_frac').value),
                max_area_frac=float(gp('max_quad_area_frac').value),
                hue_lo=int(gp('arena_hue_lo').value),
                hue_hi=int(gp('arena_hue_hi').value))
            self._dump_arena_debug(gray, diag)
            if corners is None:
                return (False, f"no arena quad: contours={diag['num_contours']} "
                               f"hull_verts={diag['hull_verts']} "
                               f"{diag['rejected'] or ''}".strip(), None)
            src = 'auto'

        try:
            corners_cm, info = hg.field_corners_from_px(
                corners, self._long_cm, self._short_cm, self._swap_axes)
            H = hg.build_homography(corners, corners_cm)
            mx, mean = hg.residual_cm(corners, corners_cm, H)
        except ValueError as e:
            return False, f'homography failed: {e}', None
        if mx > self._residual_tol:
            return (False, f'homography residual {mx:.3f} cm exceeds tolerance '
                           f'{self._residual_tol} cm', None)

        cal = ArenaCalibration(
            corners_px=[[float(u), float(v)] for u, v in corners],
            corners_field_cm=[[float(x), float(y)] for x, y in corners_cm],
            H=[[float(v) for v in row] for row in H],
            long_edge_len_cm=self._long_cm, short_edge_len_cm=self._short_cm,
            frame_width=int(gray.shape[1]), frame_height=int(gray.shape[0]),
            residual_max_cm=mx, residual_mean_cm=mean,
            camera_settings=dict(self._camera_controls),
            created=time.strftime('%Y-%m-%dT%H:%M:%S'), source=src)
        msg = (f'arena[{src}]: long={self._long_cm}cm short={self._short_cm}cm '
               f'residual_max={mx:.4f}cm origin_corner={info["origin_idx"]} '
               f'long_edge_px={info["long_edge_px"]:.0f} '
               f'short_edge_px={info["short_edge_px"]:.0f} '
               f'corners={[[round(u, 1), round(v, 1)] for u, v in corners]}')
        return True, msg, cal

    def _dump_arena_debug(self, gray, diag):
        """Dump on success AND failure -- a failed calibration is when you most
        need to see what the detector saw."""
        try:
            out = os.path.dirname(os.path.abspath(self._arena_yaml))
            os.makedirs(out, exist_ok=True)
            cv2.imwrite(os.path.join(out, 'arena_gray.png'), gray)
            cv2.imwrite(os.path.join(out, 'arena_edges.png'), diag['edges'])
            cv2.imwrite(os.path.join(out, 'arena_contours.png'),
                        diag['contours_img'])
        except Exception as e:                                  # noqa: BLE001
            self.get_logger().warning(f'arena debug dump failed: {e}')

    def _republish_arena_marker(self):
        cal, _H, _Hi, poly = self._arena_snapshot()
        if cal is None or poly is None:
            return
        # Foxglove subscribes volatile by default, so a one-shot latched publish
        # does not reach a viewer that connects later.
        self._arena_pub.publish(
            build_arena_marker(poly, self.get_clock().now().to_msg()))

    # =====================================================================
    # Stage 2 -- blobs
    # =====================================================================
    def _on_detect_spheros(self, _req, resp):
        state = self._get_state()
        if state in (INIT, IDLE) or self._arena is None:
            resp.success = False
            resp.message = 'no arena yet; call ~/detect_arena first'
            return resp
        try:
            blobs, msg = self._run_blob_detect()
        except Exception as e:                                  # noqa: BLE001
            self.get_logger().error(f'detect_spheros raised: {e}')
            resp.success, resp.message = False, f'exception: {e}'
            return resp
        if not blobs:
            resp.success, resp.message = False, msg
            return resp
        with self._blobs_lock:
            self._blobs = blobs
        if state != TRACKING:
            self._set_state(BLOBS_READY)
        self._snapshot('detect_spheros')
        resp.success = True
        resp.message = json.dumps(blobs)
        return resp

    def _run_blob_detect(self):
        fm = self._floor
        if fm is None:
            return [], 'no arena'
        if fm.tmap is None:
            ok, msg = self._rebuild_floor_now()
            if not ok:
                return [], f'floor map: {msg}'
        gray, _s = self._grab_fresh(1.0)
        if gray is None:
            return [], 'no camera frame'
        _cal, H, _Hi, poly = self._arena_snapshot()
        blobs = []
        # biggest first: the most complete blob gets the lowest id
        for d in sorted(self._detect_blobs(gray, H), key=lambda d: -d['area']):
            if not arena_mod.contains_cm(poly, d['x_cm'], d['y_cm'],
                                         self._arena_margin_cm):
                continue
            blobs.append({'id': len(blobs), 'u': round(d['u'], 1),
                          'v': round(d['v'], 1), 'area': d['area'],
                          'split': d['split'], 'score': None, 'angle': None,
                          'x_cm': round(d['x_cm'], 2), 'y_cm': round(d['y_cm'], 2)})
            if len(blobs) >= self._max_robots:
                break
        if not blobs:
            # brightness stats make a zero-blob result directly actionable: a
            # floor level far from the map's means the lighting or exposure
            # changed (~/rebuild_floor), a low p99 means the robots are not lit.
            return [], (f'no blobs (floor level={fm.level} ref={fm.ref_level} '
                        f'ratio={self._floor_ratio}; frame '
                        f'p50={np.percentile(gray, 50):.0f} '
                        f'p99={np.percentile(gray, 99):.0f} max={gray.max()})')
        return blobs, ''

    # =====================================================================
    # Stage 3 -- linking
    # =====================================================================
    def _ensure_robot_io(self, name):
        """Create every per-robot entity from the SERVICE thread only.

        Concurrent entity creation on one node is not thread-safe in rclpy, so the
        processing loop must never create a publisher. It asserts presence instead.
        """
        ns = self._ns(name)
        if name not in self._pose_pubs:
            self._pose_pubs[name] = self.create_publisher(
                PoseStamped, f'/localization/{ns}/position', 10)
            # latched: a controller that starts later still gets the last value
            self._offset_pubs[name] = self.create_publisher(
                Float32, f'/localization/{ns}/heading_offset', latched_qos())
        if self._replay:
            # offline: no /sphero/* publishers (so every LED/aim/roll write is a
            # no-op) and no live command subscriptions -- cmds.csv is the feed
            return
        if name not in self._led_pubs:
            self._led_pubs[name] = self.create_publisher(
                String, f'sphero/{ns}/led', 10)
            self._matrix_pubs[name] = self.create_publisher(
                String, f'sphero/{ns}/matrix', 10)
            self._compass_pubs[name] = self.create_publisher(
                String, f'sphero/{ns}/calibrate_compass', 10)
            self._heading_pubs[name] = self.create_publisher(
                String, f'sphero/{ns}/heading', 10)
            self._aim_pubs[name] = self.create_publisher(
                String, f'sphero/{ns}/reset_aim', 10)
            self._roll_pubs[name] = self.create_publisher(
                String, f'sphero/{ns}/roll', 10)
        if name not in self._motion_subs:
            # The device controller echoes what it ACTUALLY applied, from every
            # path -- /roll, /stop, and the task stack calling the core
            # in-process for a /fleet/policy broadcast, which never appears on
            # /roll. It is the ONLY command feed: /roll and /stop are not
            # subscribed, or every command would reach the filter twice (and
            # the 0.45 s speed lag was fitted to echo timestamps).
            self._motion_subs[name] = self.create_subscription(
                String, f'sphero/{ns}/motion_cmd',
                lambda m, n=name: self._on_motion_cmd(n, m), 10,
                callback_group=self._sub_group)
            self._compass_subs[name] = self.create_subscription(
                Bool, f'sphero/{ns}/calibrate_compass_done',
                lambda m, n=name: self._compass_done.__setitem__(n, bool(m.data)),
                10, callback_group=self._sub_group)
            # whoever resets the aim (this node or a controller) moves the
            # robot's heading zero: the offset estimate is void
            self._aim_subs[name] = self.create_subscription(
                String, f'sphero/{ns}/reset_aim',
                lambda _m, n=name: self._reset_offset(n, 'reset_aim'), 10,
                callback_group=self._sub_group)
            self._status_subs[name] = self.create_subscription(
                String, f'sphero/{ns}/status',
                lambda m, n=name: self._on_robot_status(n, m), 10,
                callback_group=self._sub_group)

    def _on_motion_cmd(self, name, msg, stamp=None):
        """A sphero/<ns>/motion_cmd echo (live) or a cmds.csv row (replay):
        the robot's EKF command input and heading-offset estimator, plus the
        nudge linker's command log. Live echoes are stamped on arrival, in the
        same wall-clock base as the camera frames."""
        try:
            d = json.loads(msg.data)
            speed = float(d.get('speed', 0))
            heading = float(d.get('heading', 0))
        except (ValueError, TypeError, AttributeError):
            return
        t = time.time() if stamp is None else float(stamp)
        with self._tracks_lock:
            self._mgr.add_cmd(name, t, speed, heading)
            est = self._offset.get(name)
            if est is not None:
                est.on_cmd(t, heading)
        with self._nudge_lock:
            if self._nudge_log is not None:
                self._nudge_log.append((t, name, speed))

    def _on_replay_cmd(self, robot, speed, heading, stamp):
        """ReplaySource feed: injected as if it were the motion_cmd echo."""
        self._on_motion_cmd(robot, String(data=json.dumps(
            {'speed': speed, 'heading': heading})), stamp)

    def _on_robot_status(self, name, msg):
        """sphero/<ns>/status: a BLE reconnect (anything -> 'connected') may
        come back with a different heading zero, so the offset is re-learnt;
        and the device controller's connect path redraws its arrow matrix, so
        a linked robot gets its tracking LEDs back."""
        try:
            state = json.loads(msg.data).get('connection_state')
        except (ValueError, AttributeError):
            return
        prev = self._conn_state.get(name)
        self._conn_state[name] = state
        if state == 'connected' and prev not in (None, 'connected'):
            self._reset_offset(name, 'reconnect')
            with self._tracks_lock:
                linked = name in self._mgr.tracks
            if linked:
                self._tracking_leds(name)
                self.get_logger().info(f'{name}: reconnected, tracking LEDs re-applied')

    def _reset_offset(self, name, why):
        with self._tracks_lock:
            est = self._offset.get(name)
            if est is None:
                return
            had = est.value is not None
            est.reset()
        if had:
            self.get_logger().info(f'{name}: heading offset reset ({why})')

    def _set_led(self, name, r, g, b, led_type='main'):
        pub = self._led_pubs.get(name)
        if pub is not None:
            pub.publish(String(data=json.dumps(
                {'type': led_type, 'red': int(r), 'green': int(g), 'blue': int(b)})))

    def _tracking_leds(self, name):
        """The tracked state: main panel black, front green, back red. The LED
        pair is what the heading is read from; a lit main panel (or the arrow
        matrix the device controller draws at connect) would only swamp it.
        Main black is one set_main_led, which blanks the whole BOLT matrix."""
        self._set_led(name, 0, 0, 0)
        self._set_led(name, 0, 255, 0, 'front')
        self._set_led(name, 255, 0, 0, 'back')

    def _link_leds_on(self, name):
        """The one lit state used for BOTH probing and aiming.

        The MAIN LED is the whole 8x8 panel on a BOLT -- set_led('main') maps
        straight to set_main_led, one command. Do not reach for the matrix API
        here: set_matrix walks all 64 pixels with a set_matrix_pixel BLE write
        each, which is far too slow to re-assert on the probe poll.

        Front green and back red go on with it, so _calibrate_aim can find the
        back LED without a second lighting step.
        """
        r, g, b = self._probe_rgb
        self._set_led(name, r, g, b)
        self._set_led(name, 0, 255, 0, 'front')
        self._set_led(name, 255, 0, 0, 'back')

    def _link_leds_off(self, name):
        """Every emitter dark. Exactly one robot may be lit during probing: a
        robot still lit when its own pre-frame is grabbed contributes nothing to
        lit_blobs_gray's diff and fails as `no_lit` while visibly glowing."""
        self._set_led(name, 0, 0, 0)
        self._set_led(name, 0, 0, 0, 'front')
        self._set_led(name, 0, 0, 0, 'back')

    def _set_robot_heading(self, name, deg):
        pub = self._heading_pubs.get(name)
        if pub is not None:
            pub.publish(String(data=json.dumps({'heading': int(round(deg)) % 360})))

    def _reset_aim(self, name):
        pub = self._aim_pubs.get(name)
        if pub is not None:
            pub.publish(String(data='{}'))
        self._reset_offset(name, 'reset_aim')

    def _measure_heading(self, u, v):
        """Field heading of the robot at pixel (u, v), or None."""
        f = self._grab_fresh_frame(1.0)
        if f is None or f.gray is None:
            return None
        _cal, H, _Hi, _poly = self._arena_snapshot()
        b = min(self._detect_blobs(f.gray, H), default=None,
                key=lambda d: math.hypot(d['u'] - u, d['v'] - v))
        if b is None or math.hypot(b['u'] - u, b['v'] - v) > self._link_tol_px:
            return None
        if f.jpeg is None:
            return None
        # the same half-res colour (and colour space) the tracking path reads
        color = cv2.imdecode(f.jpeg, cv2.IMREAD_REDUCED_COLOR_2)
        if color is not None and self._color.is_hsv:
            color = cv2.cvtColor(color, cv2.COLOR_BGR2HSV)
        return self._led_heading(color, b['u'], b['v'])

    def _calibrate_aim(self, name, blob):
        """Zero the robot's heading on field +x (left-to-right in the image).

        reset_aim FIRST, so the robot's heading 0 is its current orientation and
        the camera reading IS the offset -- no solving, no iteration. Then turn
        by that offset and reset_aim again, leaving heading 0 == field +x.

        Sphero headings are CLOCKWISE-positive (its own docs: 0 forward, 90
        right) while camera/field angles are counter-clockwise-positive, so the
        turn that brings a measured field heading back to 0 is +meas, not -meas.

        Returns the body->field offset in degrees, or None on failure.
        """
        # LEDs are already on from the probe -- _run_linking owns them for the
        # whole probe+aim step and turns them off once, afterwards.
        self._reset_aim(name)
        time.sleep(self._aim_settle)
        meas = self._measure_heading(blob['u'], blob['v'])
        if meas is None:
            self.get_logger().warning(
                f'aim {name}: no heading -- back LED not visible')
            return None
        self._set_robot_heading(name, meas)          # CW turn cancels CCW offset
        time.sleep(self._aim_settle)
        self._reset_aim(name)
        check = self._measure_heading(blob['u'], blob['v'])
        if check is None:
            self.get_logger().warning(f'aim {name}: offset {meas:.1f} deg applied, '
                                      'but could not verify')
        elif hdg.ang_diff(check, 0.0) > self._aim_tol:
            self.get_logger().warning(
                f'aim {name}: offset {meas:.1f} deg applied but robot reads '
                f'{check:.1f} deg, not 0 (tolerance {self._aim_tol:.0f})')
        else:
            self.get_logger().info(
                f'aim {name}: offset {meas:.1f} deg, now reads {check:.1f} deg, '
                'aim zeroed on field +x')
        return meas

    def _on_link_spheros(self, req, resp):
        state = self._get_state()
        if self._arena is None:
            resp.success = False
            resp.message = 'no arena; call ~/detect_arena first'
            return resp
        with self._blobs_lock:
            blobs = list(self._blobs)
        if not blobs:
            resp.success = False
            resp.message = 'no blobs; call ~/detect_spheros first'
            return resp
        if self._link_method not in ('probe', 'nudge'):
            resp.success = False
            resp.message = (f"unknown link_method '{self._link_method}' "
                            "(expected 'probe' or 'nudge')")
            return resp

        self._set_state(LINKING)
        try:
            skip_compass = bool(getattr(req, 'skip_compass',
                                self.get_parameter('skip_compass_default').value))
            registered, failed, links, msg = self._run_linking(
                list(req.callsigns), blobs, skip_compass)
            self._registered, self._failed = ident.merge_link_status(
                self._registered, self._failed,
                links.get('_targets', []), registered, failed)
            links.pop('_targets', None)
            self._links.update(links)
            payload = ident.link_status_payload(self._registered, self._failed,
                                                self._links)
            self._link_pub.publish(String(data=json.dumps(payload)))
            resp.success = bool(registered)
            resp.registered = list(registered)
            resp.failed = list(failed)
            resp.message = msg
        except Exception as e:                                  # noqa: BLE001
            self.get_logger().error(f'link_spheros raised: {e}')
            resp.success, resp.message = False, f'exception: {e}'
        finally:
            # always land in a coherent state, whatever happened above
            with self._tracks_lock:
                have = bool(self._mgr.tracks)
            self._set_state(TRACKING if (have or state == TRACKING)
                            else BLOBS_READY)
            self._snapshot('link_spheros')
        return resp

    def _run_linking(self, requested, blobs, skip_compass=True):
        if not requested and self._replay:
            # no heartbeats offline: the recording's robots are the roster
            requested = list(getattr(self._source, 'robots', []))
        targets = ident.select_targets(requested, self._fleet_last_seen,
                                       time.time(), self._heartbeat_fresh)
        if not targets:
            return [], [], {'_targets': []}, ('no targets: none requested and no '
                                              'fresh heartbeats on /sphero_fleet/robots')
        for name in targets:
            self._ensure_robot_io(name)

        if self._replay and self._seed_csv:
            registered, failed, links = self._seed_link(targets, blobs)
        elif self._link_method == 'nudge':
            if not skip_compass:
                self.get_logger().warning(
                    'link_method=nudge: aim calibration not run (it needs the '
                    'probe LEDs); skip_compass=false ignored')
            registered, failed, links = self._nudge_link(targets, blobs)
        else:
            registered, failed, links = self._probe_link(targets, blobs,
                                                         skip_compass)

        # Status colours go on only once EVERY probe is done. Painting a robot
        # green the moment it links leaves it lit while the next robot is being
        # probed, which breaks the invariant the pre-frame above is built on
        # ("all dark, so the first pre-frame is a true reference"). It is also a
        # race: _set_led is fire-and-forget over BLE, so the green can land
        # between the next robot's pre and post frames and show up in the diff
        # as a freshly-lit blob. Exactly one robot is lit at a time during
        # probing; the operator sees the outcome colours afterwards. (Nudge
        # linking also only lights robots here, after every move is done.)
        for name in registered:
            # Tracking LEDs: the heading comes from the front/back pair; the
            # white panel was only the probe's lit-diff signal. Nothing is being
            # probed now, so several lit at once is fine -- during probing it
            # never is.
            self._tracking_leds(name)
        for name in failed:
            self._set_led(name, 60, 0, 0)          # red = failed

        links['_targets'] = targets
        method = 'seed' if (self._replay and self._seed_csv) else self._link_method
        msg = (f'linked {len(registered)}/{len(targets)} by {method}: '
               f'{ {n: links[n] for n in registered} }')
        if failed:
            msg += f' | failed: {failed}'
        return registered, failed, links, msg

    def _probe_link(self, targets, blobs, skip_compass):
        """Serial LED probe. Returns (registered, failed, links)."""
        # all dark, so the first pre-frame is a true reference
        for name in targets:
            self._link_leds_off(name)
        time.sleep(self._reset_settle)

        centres = [(b['u'], b['v']) for b in blobs]
        used = {idx for n, idx in self._links.items() if n not in targets}
        registered, failed, links = [], [], {}

        for name in targets:
            # One lit window per robot: light it, identify it, aim it, go dark.
            # The LEDs stay on across probe AND aim so each robot is lit exactly
            # once, and the next robot's pre-frame is taken against a dark field.
            ok, blob_idx, pre = self._probe_one(name, centres, used)
            if ok:
                b = blobs[blob_idx]
                if not skip_compass:
                    self._calibrate_aim(name, b)
                self._lock_tracker(name, b['x_cm'], b['y_cm'], b.get('area'))
                links[name] = blob_idx
                used.add(blob_idx)
                registered.append(name)
            else:
                failed.append(name)
            self._link_leds_off(name)
            self._wait_dark(pre)
            time.sleep(self._reset_settle)
        return registered, failed, links

    def _seed_link(self, targets, blobs):
        """REPLAY ONLY (link_seed_csv): link each robot to the detect_spheros
        blob nearest its FIRST row in a reference track.csv (t,robot,x,y,...
        field cm), for recordings without a solo-nudge phase. Closest pairs
        first, one blob per robot, within SEED_TOL_CM. Returns (registered,
        failed, links)."""
        first = {}
        try:
            with open(self._seed_csv) as f:
                for r in csv.DictReader(f):
                    first.setdefault(r['robot'], (float(r['x']), float(r['y'])))
        except (OSError, KeyError, ValueError) as e:
            self.get_logger().error(f'link_seed_csv {self._seed_csv}: {e}')
            return [], list(targets), {}
        used = {idx for n, idx in self._links.items() if n not in targets}
        pairs = sorted((math.hypot(b['x_cm'] - first[n][0], b['y_cm'] - first[n][1]),
                        n, i) for n in targets if n in first
                       for i, b in enumerate(blobs) if i not in used)
        links = {}
        _g, stamp = self._current_frame_stamp()
        for d, name, i in pairs:
            if d > SEED_TOL_CM or name in links or i in used:
                continue
            b = blobs[i]
            self._lock_tracker(name, b['x_cm'], b['y_cm'], b.get('area'), stamp)
            links[name] = i
            used.add(i)
        registered = [n for n in targets if n in links]
        return registered, [n for n in targets if n not in links], links

    # ------------------------------------------------------------ nudge link
    def _nudge_link(self, targets, blobs):
        """Identity by motion. Returns (registered, failed, links).

        One mechanism for live and replay: NudgeWatch reads the motion_cmd
        echoes (live) or the recorded cmds.csv (replay) and, for each window in
        which exactly one robot was commanded to move, the one blob that moved
        is that robot. Live, the node issues those nudges itself, one robot at a
        time; a replay only watches the nudges it recorded. A robot with no
        usable solo window fails -- there is no fallback to the probe.
        """
        centres = [(b['u'], b['v']) for b in blobs]
        used = {idx for n, idx in self._links.items() if n not in targets}
        found = {}                                 # name -> blob index
        with self._nudge_lock:
            self._nudge_log = []
        try:
            if self._replay:
                _g, stamp = self._latest_frame()
                deadline = (stamp or time.time()) + \
                    self._nudge_timeout * len(targets)
                self._watch_nudges(ndg.NudgeWatch(self._nudge_settle), set(targets),
                                   centres, used, found, deadline)
            else:
                for name in targets:
                    self._wait_still()
                    with self._nudge_lock:
                        self._nudge_log = []       # only this robot's nudge
                    watch = ndg.NudgeWatch(self._nudge_settle)
                    self._issue_nudge(name, centres, used)
                    self._watch_nudges(watch, {name}, centres, used, found,
                                       time.time() + self._nudge_timeout)
        finally:
            with self._nudge_lock:
                self._nudge_log = None
        registered = [n for n in targets if n in found]
        failed = [n for n in targets if n not in found]
        return registered, failed, {n: found[n] for n in registered}

    def _watch_nudges(self, watch, pending, centres, used, found, deadline):
        """Pull frames and feed `watch` until every name in `pending` is linked,
        the frame stamp passes `deadline`, or frames stop coming (end of a
        replay). Each solo window of a pending robot is resolved by
        _resolve_nudge; a linked robot is locked at once at its new position.
        """
        last_stamp, stalls = None, 0
        pending = set(pending)
        while pending:
            gray, stamp = self._grab_fresh(1.0)
            if gray is None or stamp == last_stamp:
                stalls += 1
                if stalls >= 3:
                    self.get_logger().warning('nudge link: no new frames; giving up')
                    return
                continue
            stalls, last_stamp = 0, stamp
            if stamp > deadline:
                return
            with self._nudge_lock:
                due = [c for c in self._nudge_log if c[0] <= stamp]
                self._nudge_log = [c for c in self._nudge_log if c[0] > stamp]
            for t, name, speed in due:
                was = watch.name
                watch.command(t, name, speed)
                if was is not None and watch.name != was:
                    self.get_logger().info(
                        f'nudge {was}: window spoiled by {name} moving')
            action = watch.frame(stamp)
            if action is None:
                continue
            if watch.name not in pending:
                # not ours (or already linked): keep the window logic, skip the work
                if action == 'before':
                    watch.set_before([])
                else:
                    watch.take()
                continue
            _cal, H, _Hi, _poly = self._arena_snapshot()
            dets = self._detect_blobs(gray, H)
            if action == 'before':
                watch.set_before([(d['u'], d['v']) for d in dets])
                continue
            name, before = watch.take()
            idx = self._resolve_nudge(name, before, dets, centres, used, stamp)
            if idx is not None:
                found[name] = idx
                used.add(idx)
                pending.discard(name)

    def _resolve_nudge(self, name, before, dets, centres, used, stamp=None):
        """Blob index for `name` from its before points and after detections,
        and lock its tracker where it is NOW. None (logged) if it cannot be told
        apart."""
        after = [(d['u'], d['v']) for d in dets]
        old, new, reason = ndg.moved_blob(before, after, self._nudge_min_move)
        if reason:
            self.get_logger().warning(
                f'link {name}: nudge ambiguous ({reason}, before={len(before)} '
                f'after={len(after)} blobs, min_move={self._nudge_min_move:.0f}px)')
            return None
        m = ident.match_lit_blob(centres, [{'centroid': old}], self._link_tol_px, used)
        if not m['matched']:
            self.get_logger().warning(
                f'link {name}: the blob that moved is not a detect_spheros blob '
                f'(reason={m["reason"]}, nearest={m["distance"]:.1f}px)')
            return None
        d = min(dets, key=lambda d: math.hypot(d['u'] - new[0], d['v'] - new[1]))
        self.get_logger().info(
            f'link {name}: blob {m["blob_index"]} moved ({old[0]:.0f},{old[1]:.0f})'
            f' -> ({new[0]:.0f},{new[1]:.0f}) px')
        self._lock_tracker(name, d['x_cm'], d['y_cm'], d['area'], stamp)
        return m['blob_index']

    def _wait_still(self):
        """Until two detections 0.3 s apart agree within nudge_min_move_px
        (same count, every blob in place), or nudge_timeout_sec."""
        deadline = time.time() + self._nudge_timeout
        prev = None
        while time.time() < deadline:
            gray, _s = self._grab_fresh(1.0)
            if gray is None:
                return
            _cal, H, _Hi, _poly = self._arena_snapshot()
            pts = [(b['u'], b['v']) for b in self._detect_blobs(gray, H)]
            if prev is not None and len(pts) == len(prev) and (not pts or all(
                    min(math.hypot(u - pu, v - pv) for pu, pv in prev)
                    <= self._nudge_min_move for u, v in pts)):
                return
            prev = pts
            time.sleep(0.3)
        self.get_logger().warning('nudge: scene not still; nudging anyway')

    def _issue_nudge(self, name, centres, used):
        """Roll `name` at nudge_speed for nudge_duration_sec, then stop. The
        stop goes out from a timer so the watch keeps reading frames while the
        robot moves. No LED writes.

        Which blob is `name` is not known yet, so the heading is the one with
        the most room for EVERY still-unlinked blob (walls and other blobs).
        Field heading -> Sphero command assumes aim offset 0 (cmd = -field):
        without aim calibration the robot rolls in an unknown direction.
        """
        pub = self._roll_pubs.get(name)
        if pub is None:
            return
        heading = 0
        gray, _s = self._latest_frame()
        _cal, H, _Hi, poly = self._arena_snapshot()
        if gray is not None and H is not None:
            cands = [hg.apply_homography(u, v, H)
                     for i, (u, v) in enumerate(centres) if i not in used]
            others = [(b['x_cm'], b['y_cm']) for b in self._detect_blobs(gray, H)]
            reach = self._nudge_speed * self._cmd_speed_to_cms * self._nudge_duration
            field_deg, room = ndg.free_heading(cands, others, poly, reach)
            heading = int(round(-field_deg)) % 360
            self.get_logger().info(
                f'nudge {name}: field {field_deg:.0f} deg (cmd {heading}), '
                f'worst-case room {room:.0f} cm after {reach:.0f} cm')
        pub.publish(String(data=json.dumps({'heading': heading,
                                            'speed': self._nudge_speed})))
        threading.Timer(self._nudge_duration, lambda: pub.publish(String(
            data=json.dumps({'heading': heading, 'speed': 0})))).start()

    def _probe_one(self, name, centres, used):
        """Light exactly one robot and find which blob got brighter.

        Returns (ok, blob_index, pre_frame). The caller keeps the robot lit for
        the aim step and turns it off once afterwards, so `pre` comes back with
        it for the dark check that follows.
        """
        pre, _s = self._grab_fresh(1.0)
        if pre is None:
            return False, -1, None
        deadline = time.time() + self._render_timeout
        result = None
        last, n_lit = None, 0
        while time.time() < deadline:
            # re-assert every poll: other nodes write LEDs too and will stomp this
            self._link_leds_on(name)
            time.sleep(self._probe_poll)
            post, _s2 = self._grab_fresh(0.5)
            if post is None:
                continue
            lit, _mask = ident.lit_blobs_gray(post, pre, self._lit_delta,
                                              self._lit_min_area)
            m = ident.match_lit_blob(centres, lit, self._link_tol_px, used)
            last, n_lit = m, max(n_lit, len(lit))
            if m['matched']:
                result = m['blob_index']
                break
        if result is None:
            # The reason matters and used to be thrown away. 'no_lit' means the
            # robot never visibly brightened -- an LED/BLE problem, not a vision
            # one. 'too_far' means it DID light but nowhere near a stored blob
            # centre: the detect_spheros list is stale or that blob was never
            # this robot. 'no_blobs' means every candidate is already claimed.
            reason = (last or {}).get('reason', 'no_frame')
            dist = (last or {}).get('distance', float('inf'))
            self.get_logger().warning(
                f'link {name}: no lit blob matched (reason={reason}, '
                f'lit_blobs_seen={n_lit}, nearest={dist:.1f}px, '
                f'tol={self._link_tol_px:.0f}px)')
            return False, -1, pre
        return True, result, pre

    def _wait_dark(self, pre):
        """Block until the field looks like `pre` again, so the next robot's
        pre-frame is taken against a dark arena."""
        if pre is None:
            return
        off_deadline = time.time() + self._off_timeout
        while time.time() < off_deadline:
            post, _s = self._grab_fresh(0.3)
            if post is None:
                return
            lit, _m = ident.lit_blobs_gray(post, pre, self._lit_delta,
                                           self._lit_min_area)
            if not lit:
                return

    def _current_frame_stamp(self):
        """(frame, stamp) of the current frame WITHOUT pulling one: in lockstep
        replay latest_frame() advances the recording, current_frame() does not."""
        f = self._current_frame()
        return f, (f.stamp if f is not None and f.stamp else time.time())

    def _lock_tracker(self, name, x_cm, y_cm, area_px=None, stamp=None):
        """(Re)start `name`'s track where its blob is, at frame time `stamp`
        (default: the current frame). A re-link is the identity check, so it
        also clears SUSPECT, and the heading offset is learnt afresh."""
        if stamp is None:
            _b, stamp = self._current_frame_stamp()
        with self._tracks_lock:
            self._mgr.add_track(name, stamp, (x_cm, y_cm), area_px=area_px)
            self._last_blob.pop(name, None)
            self._miss[name] = 0
            self._offset[name] = HeadingOffsetEstimator()
            self._offset_pub_t.pop(name, None)
        self.get_logger().info(f'locked {name} at ({x_cm:.1f}, {y_cm:.1f}) cm')

    # =====================================================================
    # Stage 4/5 -- reset
    # =====================================================================
    def _on_reset(self, _req, resp):
        with self._tracks_lock:
            n_tracks = len(self._mgr.tracks)
            self._mgr = TrackManager(**self._mgr_kwargs)
            self._last_blob, self._miss = {}, {}
            self._offset, self._offset_pub_t = {}, {}
        with self._blobs_lock:
            n_blobs = len(self._blobs)
            self._blobs = []
        self._links, self._registered, self._failed = {}, [], []
        self._marker_ids = {}
        # publishers/subscriptions are deliberately kept: destroying rclpy
        # entities from a service thread while processing runs is a known hazard.
        self._set_state(ARENA_READY if self._arena is not None else IDLE)
        resp.success = True
        resp.message = f'dropped {n_tracks} tracks, {n_blobs} blobs; arena kept'
        return resp

    # =====================================================================
    # Frame processing -- driven by the source's frames, on its own thread
    # =====================================================================
    def start_processing(self):
        """Start the processing thread (main does; tests call _on_frame)."""
        self._proc_thread = threading.Thread(target=self._process_loop,
                                             daemon=True, name='process')
        self._proc_thread.start()

    def _active(self):
        with self._tracks_lock:
            have_tracks = bool(self._mgr.tracks)
        return self._get_state() in (TRACKING, LINKING) and have_tracks

    def _process_loop(self):
        """Wait for each new frame and hand it to _on_frame. The services and
        subscriptions stay on the executor; what they share with this thread
        is guarded by _tracks_lock (tracks, offsets) and the source's own lock.
        A lockstep replay is advanced by this loop's pulls, so while nothing is
        tracked it does not pull: the frames before linking belong to the
        services (detect_spheros builds the map, the nudge watch reads them)."""
        last = None
        while not self._proc_stop.is_set():
            src = self._source
            if src is None:
                self._proc_stop.wait(0.5)
                continue
            if getattr(src, 'lockstep', False) and not self._active():
                self._proc_stop.wait(0.005)
                continue
            try:
                f = src.wait_frame(last, 0.25)
            except Exception as e:                              # noqa: BLE001
                self.get_logger().warning(f'source failed: {e}',
                                          throttle_duration_sec=5.0)
                self._proc_stop.wait(0.1)
                continue
            if f is None:                    # no frame: a stalled camera?
                cur = self._current_frame()
                self._check_camera_health(cur.stamp if cur else 0.0, time.time())
                continue
            last = f.seq
            try:
                self._on_frame(f)
            except Exception as e:                              # noqa: BLE001
                self.get_logger().error(f'frame processing failed: {e}',
                                        throttle_duration_sec=5.0)

    def _on_frame(self, frame):
        """Every frame the source delivers: publish its raw bytes when asked,
        then process it -- every frame at track_rate_hz 60, every second one
        (by seq) at 30. Returns whether it was processed."""
        if self._raw_pub is not None and frame.jpeg is not None:
            msg = CompressedImage()
            msg.header.stamp = self._stamp_from_wall(frame.stamp)
            msg.header.frame_id = 'overhead_camera'
            msg.format = 'jpeg'
            msg.data = frame.jpeg.tobytes()  # the camera's bytes, not re-encoded
            self._raw_pub.publish(msg)
        last = self._last_proc_seq
        if last is not None and 0 < frame.seq - last < self._frame_step:
            return False
        self._last_proc_seq = frame.seq
        self._process_frame(frame)
        return True

    def _process_frame(self, frame):
        t_start = time.time()
        state = self._get_state()
        with self._tracks_lock:
            have_tracks = bool(self._mgr.tracks)
        active = state in (TRACKING, LINKING) and have_tracks

        # Timebase is the frame's ACQUISITION stamp, not wall-now. The floor
        # level is tracked in every state, so the first map is built (lazily)
        # and drift is caught before tracking starts.
        gray, stamp = frame.gray, frame.stamp
        fresh = gray is not None and stamp != self._last_stamp
        if fresh:
            self._observe_floor(gray, stamp)
        if not active:
            if fresh:
                self._last_stamp = stamp
            return
        _cal, H, _Hinv, _poly = self._arena_snapshot()
        if H is None:
            return
        now = time.time()
        age = max(0.0, now - stamp) if stamp else 0.0
        if getattr(self._source, 'lockstep', False):
            # lockstep stamps drift behind the wall clock by however slow the
            # consumer is; extrapolating over that gap flings poses off the field
            age = 0.0

        # One TrackManager step per NEW frame, at the frame's stamp: the EKFs
        # predict to it from the motion_cmd history, so a repeated frame is
        # not a step (the publish below extrapolates instead). While LINKING,
        # LED probes flash robots and nudges move unlinked ones past linked
        # ones, so no camera step: tracks hold, then catch up (or reacquire)
        # on the first TRACKING frame.
        n_comps = 0
        t_det = t_trk = 0.0
        if state == TRACKING and fresh:
            n_comps, t_det, t_trk = self._camera_update(frame, H)

        # publish_rate_hz <= 0 means every processed frame, as soon as the pose
        # exists: decimating only adds latency, and a positive rate that does
        # not divide the processing rate quantises badly (15 Hz asked of a
        # 30 Hz loop measured 11.5 Hz).
        if self._publish_rate <= 0 or \
                now - self._last_pub >= 1.0 / self._publish_rate:
            self._publish_tracks(self._track_views(), age, stamp)
            self._last_pub = now
        self._publish_offsets(stamp)

        if fresh:
            self._last_stamp = stamp
        ms = (time.time() - t_start) * 1000.0
        self._proc_ms.append(ms)
        self._proc_n += 1
        dt = now - self._proc_t0
        if dt >= 1.0:
            self._proc_fps = self._proc_n / dt
            self._proc_n, self._proc_t0 = 0, now
        self._diag['process_ms'] = round(ms, 2)
        self._diag['detect_ms'] = round(t_det * 1000.0, 2)
        self._diag['track_ms'] = round(t_trk * 1000.0, 2)
        self._diag['n_comps'] = n_comps

    def _camera_update(self, frame, H):
        """Detect, read LED headings, step the manager. Returns (n blobs,
        detect s, track s)."""
        stamp = frame.stamp
        # the colour decode of THIS frame runs on the worker while the blobs
        # are found on the gray one
        has_color = frame.jpeg is not None
        if has_color:
            self._color.submit(frame.seq, frame.jpeg)

        # ONE blob pass for the whole frame (arena polygon + margin applied)
        t_det = time.time()
        comps = self._detect_blobs(frame.gray, H)
        t_det = time.time() - t_det

        t_trk = time.time()
        # the single colour touch point: the LED pair of every UNCROWDED blob,
        # read from the colour of the SAME frame (by seq) -- none ready in time,
        # no LED fix this frame. Crowded = split, or another blob's centre within
        # led_search_px + point_sep_px / 2 on both axes: the LEDs sit half the
        # pair separation from a ball's centre, so only then can the neighbour's
        # LEDs fall inside this blob's square search window.
        # patrol_20261104_163022: unsplit blobs 33-37 px apart (both axes) read
        # each other's LEDs and swapped; parked pairs 41-46 px apart read clean.
        lim = self._led_search + self._pt_sep / 2.0
        crowded = [c['split'] or any(
            max(abs(c['u'] - o['u']), abs(c['v'] - o['v'])) < lim
            for o in comps if o is not c) for c in comps]
        color = None
        if has_color and not all(crowded):
            color = self._color.get(frame.seq)
            if color is None:
                self._diag['color_late'] += 1
        if has_color:
            self._diag['led_crowded'] += sum(
                cr and not c['split'] for c, cr in zip(comps, crowded))
        leds = [None if cr else self._led_heading(color, c['u'], c['v'])
                for c, cr in zip(comps, crowded)]
        blobs = [Blob(c['x_cm'], c['y_cm'], c['area'], c['split'],
                      None if h is None else math.radians(h))
                 for c, h in zip(comps, leds)]
        with self._tracks_lock:
            used = self._mgr.step(stamp, blobs)
            for name, tr in self._mgr.tracks.items():
                j = used.get(name)
                if j is None:
                    self._miss[name] = self._miss.get(name, 0) + 1
                    continue
                c = comps[j]
                self._last_blob[name] = (c['u'], c['v'], c['area'], c['split'])
                self._miss[name] = 0
                # offset samples only from a robot whose LEDs passed the
                # first-fix consistency check (dead LEDs give scattered noise)
                est = self._offset.get(name)
                if est is not None and leds[j] is not None and tr.ekf.theta_known:
                    est.add(stamp, leds[j])
        return len(comps), t_det, time.time() - t_trk

    def _track_views(self):
        """Plain-dict snapshot of every track for the publishers, the overlay
        and ~/state, taken under the lock (processing mutates the EKFs)."""
        with self._tracks_lock:
            out = []
            for name, tr in self._mgr.tracks.items():
                u, v, area, split = self._last_blob.get(name, (0.0, 0.0, 0, False))
                est = self._offset.get(name)
                x, y = tr.pos_cm
                out.append({
                    'name': name, 'x': x, 'y': y, 'status': tr.status,
                    'theta': tr.theta, 'heading_deg': tr.heading_deg,
                    'vel': tr.vel_cms, 'u': u, 'v': v, 'area': area,
                    'split': split, 'miss': self._miss.get(name, 0),
                    'unseen_s': max(0.0, tr.t - tr.last_seen),
                    'offset': None if est is None else est.value})
            return out

    def _publish_offsets(self, stamp):
        """/localization/<ns>/heading_offset: once estimated, then on a change
        of more than OFFSET_PUBLISH_DEG, at most once per OFFSET_PUBLISH_PERIOD_S
        of frame time."""
        due = []
        with self._tracks_lock:
            for name, est in self._offset.items():
                last = self._offset_pub_t.get(name)
                if not est.changed_since_publish(OFFSET_PUBLISH_DEG) or (
                        last is not None and stamp - last < OFFSET_PUBLISH_PERIOD_S):
                    continue
                est.mark_published()
                self._offset_pub_t[name] = stamp
                due.append((name, est.value))
        for name, value in due:
            pub = self._offset_pubs.get(name)
            if pub is not None:
                pub.publish(Float32(data=float(value)))

    def _publish_tracks(self, views, age, frame_stamp=0.0):
        stamp = self.get_clock().now().to_msg()
        positions, statuses, dets = {}, {}, []
        for t in views:
            x, y = t['x'], t['y']
            if self._extrapolate and age > 0:
                # analytically what the EKF would do, WITHOUT mutating it -- the
                # next frame's predict must stay monotonic
                vx, vy = t['vel']
                x += vx * age
                y += vy * age
            self._publish_pose(t['name'], x, y, stamp, t['theta'])
            positions[t['name']] = (x, y)
            statuses[t['name']] = t['status']
            dets.append({
                'name': t['name'],
                'u': round(float(t['u']), 1),         # pixel, last claimed blob
                'v': round(float(t['v']), 1),
                'x_cm': round(float(x), 2),           # field, as published above
                'y_cm': round(float(y), 2),
                'angle': None,                        # no template spine any more
                'heading_deg': (None if t['heading_deg'] is None   # EKF heading
                                else round(float(t['heading_deg']), 1)),
                'score': None,
                'area': int(t['area']),
                'split': bool(t['split']),
                'status': t['status'],
                'miss': t['miss'],                    # processed frames since its blob
            })
        if positions:
            self._track_pub.publish(build_track_markers(
                positions, self._sphere_radius_mm, stamp, self._marker_ids,
                statuses))
        if dets:
            self._det_pub.publish(String(data=json.dumps({
                'stamp': stamp.sec + stamp.nanosec * 1e-9,
                'frame_stamp': float(frame_stamp or 0.0),
                'frame_age_s': round(float(age), 4),
                'robots': dets,
            })))
            # Read live rather than caching in __init__ so the flag can be
            # flipped on a running node with `ros2 param set`. Throttled: frames
            # come at 60 Hz and an unthrottled line per robot per frame buries
            # every other log. ~/detections carries the full-rate data.
            if self.get_parameter('debug_positions').value:
                self.get_logger().info(
                    'pos  ' + '  |  '.join(
                        f"{r['name']} ({r['x_cm']:6.1f}, {r['y_cm']:6.1f}) cm "
                        f"{r['status']}" for r in dets),
                    throttle_duration_sec=1.0)

    def _publish_pose(self, name, x_cm, y_cm, stamp, theta=None):
        """theta: EKF field heading in rad (CCW from +x), or None before the
        robot's first LED heading fix -- then the orientation is identity, which
        a consumer cannot tell from a real heading of 0 (none reads it today)."""
        pub = self._pose_pubs.get(name)
        if pub is None:
            self.get_logger().warning(
                f'no pose publisher for {name} (created only at link time)',
                throttle_duration_sec=10.0)
            return
        msg = PoseStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = 'field'
        msg.pose.position.x = float(x_cm)        # CENTIMETRES -- hard contract
        msg.pose.position.y = float(y_cm)
        msg.pose.position.z = 0.0
        if theta is None:
            msg.pose.orientation.w = 1.0
        else:                                    # yaw about +z
            msg.pose.orientation.z = math.sin(0.5 * theta)
            msg.pose.orientation.w = math.cos(0.5 * theta)
        pub.publish(msg)

    def _check_camera_health(self, stamp, now):
        # a replay holding its last frame, or a lockstep replay behind the wall
        # clock, is not a dead camera: reopening would only repeat every wait
        if not stamp or not getattr(self._source, 'live', True):
            return
        stale = now - stamp
        if stale > self._camera_restart > 0:
            self.get_logger().error(f'camera stale {stale:.1f}s -- reopening')
            if self._source is not None and self._source.reopen():
                # the UVC manual-exposure discard happens on EVERY stream start
                try:
                    self._source.apply_controls()
                except Exception:                               # noqa: BLE001
                    pass
        elif stale > self._camera_stale_warn:
            self.get_logger().warning(f'camera stale {stale:.1f}s',
                                      throttle_duration_sec=5.0)

    # =====================================================================
    # Annotation -- bounded FIFO, never blocks processing
    # =====================================================================
    def _annotate_push_timer(self):
        """Feed the annotate worker in EVERY state, not only while tracking.

        The operator needs the live view to aim the camera and place the arena
        corners long before any robot is linked, so this runs off its own timer
        rather than out of the processing loop. current_frame(): a lockstep
        replay must not be advanced by the overlay.
        """
        f = self._current_frame()
        if f is not None:
            self._push_annotate(f.gray, f.stamp, self._get_state(), self._track_views())

    #: Heading arrow length in FIELD cm, not pixels. Projecting a fixed field
    #: length keeps every robot's arrow the same physical size, so perspective
    #: shrinks the far ones exactly as it shrinks the robots themselves.
    HEADING_ARROW_CM = 12.0

    def _heading_tip_px(self, t, Hinv):
        """Pixel at HEADING_ARROW_CM ahead of `t` along its FIELD heading.

        `t` is a _track_views() entry; the heading is the EKF's.
        Projected rather than drawn at -heading in image space: the field axes
        are rotated in the image (the arena sits at an angle) and perspective is
        not uniform, so an image-space angle would be wrong by the arena's
        rotation and drift across the frame.
        """
        if Hinv is None or t['theta'] is None:
            return None
        th = t['theta']
        x, y = t['x'], t['y']
        tip = np.array([[x + self.HEADING_ARROW_CM * math.cos(th),
                         y + self.HEADING_ARROW_CM * math.sin(th)]])
        u, v = hg.apply_homography_many(tip, Hinv)[0]
        return [float(u), float(v)]

    def _render_dict(self, state, tracks):
        """What the overlay is drawn from. Shared by the JPEG stream and the
        ~/save_annotated snapshot so the two never drift apart."""
        with self._blobs_lock:
            blobs = list(self._blobs) if state in (BLOBS_READY, ARENA_READY) else []
        cal, _H, Hinv, _poly = self._arena_snapshot()
        return {
            'state': state,
            'arena_px': cal.corners_px if cal is not None else None,
            'blobs': [{'id': b['id'], 'u': b['u'], 'v': b['v']} for b in blobs],
            'tracks': [{'name': t['name'], 'u': t['u'], 'v': t['v'],
                        'status': t['status'], 'score': None, 'angle': None,
                        'heading_tip': self._heading_tip_px(t, Hinv)}
                       for t in tracks],
            'banner': f'{state}  process={self._diag["process_ms"]}ms',
        }

    def _on_save_annotated(self, _req, resp):
        """Write ONE annotated PNG and return its path.

        The point is to see the overlay without holding the camera hostage:
        no annotate_rate_hz stream to enable, and no stopping the node so a
        separate process can open /dev/video0. Files are timestamped, so
        repeated calls accumulate instead of overwriting.
        """
        path, err = self._write_annotated('annotated')
        resp.success, resp.message = (path is not None), (err or path)
        return resp

    def _write_annotated(self, tag):
        """Write one annotated PNG, returning (path|None, error|None).

        Never raises: a snapshot is a diagnostic, so a failure to write one must
        not fail the calibration step that asked for it.
        """
        f = self._current_frame()
        if f is None or f.gray is None:
            return None, 'no camera frame'
        gray = f.gray
        tracks = self._track_views()
        out = os.path.dirname(os.path.abspath(self._arena_yaml))
        path = os.path.join(out, time.strftime(f'{tag}_%Y%m%d_%H%M%S.png'))
        try:
            os.makedirs(out, exist_ok=True)
            bgr = ann.draw_overlay(gray, self._render_dict(self._get_state(),
                                                           tracks), self._ball_d)
            if not cv2.imwrite(path, bgr):
                raise IOError(f'imwrite failed: {path}')
        except Exception as e:                                  # noqa: BLE001
            return None, str(e)
        return path, None

    def _snapshot(self, tag):
        """Fire-and-forget snapshot for a calibration step. Logs the path so the
        operator can open it; never touches the caller's response payload."""
        path, err = self._write_annotated(tag)
        if path:
            self.get_logger().info(f'annotated: {path}')
        else:
            self.get_logger().warning(f'annotated snapshot failed: {err}')

    def _push_annotate(self, gray, stamp, state, tracks):
        if gray is None:
            return
        # <= 0 means OFF, not "unlimited". The rate gate below only fires when
        # the rate is positive, so without this an annotate_rate_hz of 0 would
        # push every single frame -- the opposite of what setting it to 0 means.
        # ~/detections carries what the overlay was drawn from, so a bag of that
        # topic can rebuild the annotated frames offline.
        if self._annotate_rate <= 0:
            return
        now = time.time()
        if now - self._last_annot_push < 1.0 / self._annotate_rate:
            return
        self._last_annot_push = now
        render = self._render_dict(state, tracks)
        self._annot_seq += 1
        item = (gray, render, stamp, self._annot_seq)
        try:
            self._annot_q.put_nowait(item)
        except queue.Full:
            # Only reached after the worker has already degraded quality and
            # scale. Drop the OLDEST so latency stays bounded.
            try:
                self._annot_q.get_nowait()
                self._annot_q.put_nowait(item)
            except (queue.Empty, queue.Full):
                pass
            self._diag['annotate_drops'] += 1
        self._diag['annotate_q'] = self._annot_q.qsize()

    def _annotate_loop(self):
        quality = self._jpeg_quality
        scale = self._annotate_scale
        while not self._annot_stop.is_set():
            try:
                gray, render, stamp, _seq = self._annot_q.get(timeout=0.2)
            except queue.Empty:
                quality, scale = self._jpeg_quality, self._annotate_scale
                continue
            # Escalate BEFORE dropping: a cheaper encode is strictly better than
            # losing a frame we already computed the labels for.
            depth = self._annot_q.qsize()
            if depth > self._annot_q.maxsize / 2:
                quality = max(self._jpeg_min_quality, quality - 10)
                if quality <= self._jpeg_min_quality:
                    scale = 0.5
            elif depth == 0:
                quality = self._jpeg_quality
                scale = self._annotate_scale
            try:
                bgr = ann.draw_overlay(gray, render, self._ball_d)
                data = ann.encode_jpeg(bgr, quality, scale)
                if not data:
                    continue
                msg = CompressedImage()
                msg.header.stamp = self._stamp_from_wall(stamp)
                msg.header.frame_id = 'overhead_camera'
                msg.format = 'jpeg'
                msg.data = data
                self._annot_pub.publish(msg)
            except Exception as e:                              # noqa: BLE001
                self.get_logger().warning(f'annotate failed: {e}',
                                          throttle_duration_sec=5.0)

    def _stamp_from_wall(self, wall):
        """Frame ACQUISITION time, so a bag replays truthfully even if the frame
        was published late out of the queue."""
        from builtin_interfaces.msg import Time
        if not wall:
            return self.get_clock().now().to_msg()
        sec = int(wall)
        return Time(sec=sec, nanosec=int((wall - sec) * 1e9))

    # =====================================================================
    def _publish_state(self):
        cal = self._arena
        fm = self._floor
        tracks = self._track_views()
        stats = self._source.stats() if self._source else {}
        # The DRIVER rate is what sets the exposure clamp. The consume rate is
        # how fast the grab thread keeps up (MJPEG decode bound) and is expected
        # to be lower -- comparing that one to camera_fps gives a false alarm.
        driver_fps = float(stats.get('driver_fps', 0.0))
        camera_ok = True
        if driver_fps > 0 and self._expected_fps > 0:
            camera_ok = abs(driver_fps - self._expected_fps) / self._expected_fps \
                <= self._fps_tol
            if not camera_ok:
                self.get_logger().warning(
                    f'camera frame interval is {driver_fps:.0f} fps, expected '
                    f'{self._expected_fps:.0f}; the exposure clamp has moved to '
                    f'{1000.0/driver_fps:.1f} ms and the brightness calibration '
                    'no longer holds',
                    throttle_duration_sec=30.0)
        ms = list(self._proc_ms)
        diag = dict(self._diag)
        diag.update(track_rate_hz=self._track_rate,
                    processed_fps=round(self._proc_fps, 1),
                    process_ms_p50=round(float(np.percentile(ms, 50)), 2) if ms else None,
                    process_ms_p95=round(float(np.percentile(ms, 95)), 2) if ms else None)
        payload = {
            'state': self._get_state(),
            'camera': {'ok': camera_ok, **stats},
            'diag': diag,
            'floor': None if fm is None else {**fm.state(), 'ratio': self._floor_ratio},
            'arena': None if cal is None else {
                'long_cm': cal.long_edge_len_cm, 'short_cm': cal.short_edge_len_cm,
                'residual_max_cm': cal.residual_max_cm, 'source': cal.source},
            'n_blobs': len(self._blobs),
            'links': dict(self._links),
            'tracks': {t['name']: {'status': t['status'], 'miss': t['miss'],
                                   'area': t['area'],
                                   'x_cm': round(t['x'], 1),
                                   'y_cm': round(t['y'], 1),
                                   'heading': (None if t['heading_deg'] is None
                                               else round(t['heading_deg'], 1)),
                                   'unseen_s': round(t['unseen_s'], 2),
                                   'heading_offset': (None if t['offset'] is None
                                                      else round(t['offset'], 1))}
                       for t in tracks},
        }
        self._state_pub.publish(String(data=json.dumps(payload)))

    def destroy_node(self):
        # stop and JOIN the workers before the publishers go away, otherwise
        # they can publish into a destroyed handle on the way out
        self._proc_stop.set()
        if self._proc_thread is not None:
            self._proc_thread.join(timeout=2.0)
        self._color.close()
        self._annot_stop.set()
        if self._annot_thread is not None:
            self._annot_thread.join(timeout=1.0)
        if self._source is not None:
            try:
                self._source.close()
            except Exception:                                   # noqa: BLE001
                pass
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = OverheadTrackerNode()
    node.start_processing()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
