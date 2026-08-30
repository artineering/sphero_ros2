#!/usr/bin/env python3
"""Overhead global-shutter multi-Sphero tracker.

Operator-staged workflow, one service per stage:

    ~/detect_arena    find the arena corners -> homography (px -> field cm)
    ~/detect_spheros  snapshot the robot blobs currently on the arena
    ~/link_spheros    serial LED probe binds each blob to a callsign
    (tracking starts automatically once at least one callsign is linked)

Steady state, per frame: predict each robot with its Kalman filter, extract
bright candidates ONCE for the whole frame, let each robot claim the candidate
inside its own forward-predicted ROI, refine the claim with the rotation-bank
template matcher on a thread pool, gate the result, and fuse it.

Output is `/localization/<name_safe>/position` in field CENTIMETRES at 10 Hz --
the existing contract the device and task controllers already consume -- plus an
annotated JPEG for operators and rosbag.

Why one node and not the usual capture/algorithm split: 1280x720 mono at 200 fps
is 184 MB/s, which is not something to push through DDS on a Pi 5. The camera is
owned directly through a duck-typed pull source so the sim swap still works.
"""

import json
import math
import os
import queue
import threading
import time
from concurrent.futures import ThreadPoolExecutor, wait
from dataclasses import dataclass, field

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
from sphero_instance_controller.msg import SpheroSensor
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker, MarkerArray

from . import annotate as ann
from . import arena as arena_mod
from . import heading as hdg
from . import homography as hg
from . import identify as ident
from . import roi as roi_mod
from . import template as tmpl
from .arena_store import ArenaCalibration, load_arena, save_arena
from .association import associate
from .camera import build_source
from .fusion import VX, VY, FusionKF, FusionParams
from .markers import build_arena_marker, build_track_markers

# states
INIT = 'INIT'
IDLE = 'IDLE'
ARENA_READY = 'ARENA_READY'
BLOBS_READY = 'BLOBS_READY'
LINKING = 'LINKING'
TRACKING = 'TRACKING'

# per-track status
HEALTHY = 'HEALTHY'
COASTING = 'COASTING'
LOST = 'LOST'
UNRESOLVED = 'UNRESOLVED'
OUT_OF_ARENA = 'OUT_OF_ARENA'
SUSPECT = 'SUSPECT'


def name_safe(name):
    """SB-33E3 -> SB_33E3 (topic-safe)."""
    return str(name).replace('-', '_')


def latched_qos():
    return QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                      history=QoSHistoryPolicy.KEEP_LAST)


@dataclass
class TrackState:
    name: str
    kf: FusionKF
    status: str = HEALTHY
    miss_count: int = 0
    lost_since: float = 0.0
    disagreements: int = 0
    last_px: tuple = (0.0, 0.0)
    last_angle: float = 0.0
    last_heading: float = None      # 0-360 field deg, None until the back LED is seen
    last_score: float = 0.0
    roi: tuple = field(default_factory=tuple)


class OverheadTrackerNode(Node):

    def __init__(self, **kwargs):
        # kwargs forwarded to rclpy Node -- notably parameter_overrides, which is
        # how the tests configure a sim source without a YAML file
        super().__init__('overhead_tracker_node', **kwargs)
        self._declare_params()
        gp = self.get_parameter

        # ---- camera / template geometry
        self._source_kind = gp('source').value
        self._width = int(gp('frame_width').value)
        self._height = int(gp('frame_height').value)
        self._ball_d = int(gp('ball_diameter_px').value)
        self._match_thresh = float(gp('match_thresh').value)
        self._min_bright = int(gp('min_bright').value)
        self._min_blob_area = int(gp('min_blob_area').value)
        self._pad_extra = int(gp('candidate_pad_px').value)
        self._dilate_half = int(gp('candidate_dilate_half_px').value) \
            or self._ball_d // 2
        self._pt_sep = int(gp('point_sep_px').value)
        self._led_search = int(gp('led_search_px').value)
        self._led_sat_min = int(gp('led_sat_min').value)
        self._led_val_min = int(gp('led_val_min').value)
        self._detect_scale = int(gp('detect_scale').value)
        self._max_robots = int(gp('max_robots').value)
        self._min_sep_px = float(gp('min_separation_px').value)

        # ---- rates
        self._track_rate = float(gp('track_rate_hz').value)
        self._publish_rate = float(gp('publish_rate_hz').value)
        self._annotate_rate = float(gp('annotate_rate_hz').value)
        self._jpeg_quality = int(gp('jpeg_quality').value)
        self._jpeg_min_quality = int(gp('annotate_min_quality').value)
        self._annotate_scale = float(gp('annotate_scale').value)

        # ---- roi / gating
        self._roi_mode = gp('roi_mode').value
        self._roi_scale = float(gp('roi_scale').value)
        self._roi_growth = float(gp('roi_growth_on_miss').value)
        self._roi_min_half = int(gp('roi_min_half_px').value)
        self._roi_max_half = int(gp('roi_max_half_px').value)
        self._max_misses = int(gp('max_consecutive_misses').value)
        self._lost_timeout = float(gp('lost_timeout_sec').value)
        self._reacquire_gate = float(gp('reacquire_gate_cm').value)
        self._innovation_gate = float(gp('innovation_gate_cm').value)
        self._swap_suspect_n = int(gp('swap_suspect_n').value)
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
        self._sensor_fresh = float(gp('sensor_fresh_sec').value)
        self._fuse_telemetry = bool(gp('fuse_telemetry').value)
        self._sphere_radius_mm = float(gp('sphere_radius_mm').value)
        self._use_tmpl_heading = bool(gp('use_template_heading').value)

        self._fusion_params = FusionParams(
            body_to_field_yaw_offset=float(gp('body_to_field_yaw_offset').value),
            r_cam=float(gp('r_cam').value), r_vel=float(gp('r_vel').value),
            r_ori=float(gp('r_ori').value), r_acc=float(gp('r_acc').value),
            r_gyro=float(gp('r_gyro').value), q_pos=float(gp('q_pos').value),
            q_vel=float(gp('q_vel').value), q_yaw=float(gp('q_yaw').value),
            q_angle=float(gp('q_angle').value), q_acc=float(gp('q_acc').value),
            q_gyro=float(gp('q_gyro').value))

        # OpenCV's internal parallel-for would oversubscribe the cores against our
        # own pool and make the threading a net loss. Must be set before any work.
        cv2.setNumThreads(int(gp('opencv_num_threads').value))

        # ---- template bank
        self._tmpl = tmpl.build_template(
            ball_d=self._ball_d, square=int(gp('matrix_square_px').value),
            pt_sep=int(gp('point_sep_px').value),
            pt_r=int(gp('point_radius_px').value))
        self._bank = tmpl.bank(self._tmpl, int(gp('bank_step_deg').value))
        self._tmpl_size = self._tmpl.shape[0]

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
        self._tracks = {}                  # name -> TrackState
        self._tracks_lock = threading.Lock()
        self._links = {}                   # name -> blob index
        self._registered, self._failed = [], []
        self._marker_ids = {}
        self._latest_sensor = {}           # name -> (SpheroSensor, recv_time)
        self._compass_done = {}
        self._fleet_last_seen = {}
        self._fleet_name_safe = {}
        self._pose_pubs, self._led_pubs = {}, {}
        self._matrix_pubs, self._compass_pubs = {}, {}
        self._heading_pubs, self._aim_pubs = {}, {}
        self._sensor_subs, self._compass_subs = {}, {}

        self._last_stamp = 0.0
        self._last_tick_wall = time.time()
        self._last_pub = 0.0
        self._diag = {'tick_ms': 0.0, 'match_ms': 0.0, 'n_comps': 0,
                      'pool_overrun': 0, 'annotate_drops': 0, 'annotate_q': 0}

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

        try:
            self._source = build_source(
                self._source_kind, device=gp('camera_device').value,
                width=self._width, height=self._height, fps=self._expected_fps,
                fourcc=gp('fourcc').value, controls=controls,
                n=int(gp('sim_spheros').value), seed=int(gp('seed').value),
                ball_d=self._ball_d, logger=self.get_logger())
            self._source.start()
        except Exception as e:                                  # noqa: BLE001
            self.get_logger().error(f'camera source failed to start: {e}')
            self._source = None

        # ---- callback groups
        self._timer_group = MutuallyExclusiveCallbackGroup()
        self._service_group = MutuallyExclusiveCallbackGroup()
        self._sub_group = ReentrantCallbackGroup()
        self._annotate_group = MutuallyExclusiveCallbackGroup()

        # ---- match pool
        workers = int(gp('match_workers').value)
        if workers <= 0:
            workers = min(4, os.cpu_count() or 4)
        self._match_workers = workers
        self._pool = ThreadPoolExecutor(max_workers=workers,
                                        thread_name_prefix='roimatch')
        self._match_timeout = float(gp('match_timeout_sec').value)

        # ---- publishers
        self._annot_pub = self.create_publisher(
            CompressedImage, '~/annotated/compressed', qos_profile_sensor_data)
        self._arena_pub = self.create_publisher(Marker, '~/arena_corners',
                                                latched_qos())
        self._track_pub = self.create_publisher(MarkerArray, '~/track_markers', 10)
        self._state_pub = self.create_publisher(String, '~/state', latched_qos())
        # Everything needed to reconstruct an annotated frame offline from a bag:
        # pixel position, field position, heading and match score per robot.
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

        # ---- annotate worker (bounded FIFO; tick never draws or encodes)
        self._annot_q = queue.Queue(maxsize=int(gp('annotate_queue_max').value))
        self._annot_stop = threading.Event()
        self._annot_thread = threading.Thread(target=self._annotate_loop,
                                              daemon=True, name='annotate')
        self._annot_thread.start()
        self._last_annot_push = 0.0
        self._annot_seq = 0

        # ---- timers
        self.create_timer(1.0 / max(self._track_rate, 1.0), self._tick,
                          callback_group=self._timer_group)
        self.create_timer(1.0 / max(self._track_rate, 1.0), self._pump_source,
                          callback_group=self._annotate_group)
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
            f'publish={self._publish_rate}Hz workers={workers} state={self._state}')

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
        d('control_reapply_delay_sec', 1.5)
        d('fps_warn_tolerance', 0.15)
        d('camera_stale_warn_sec', 1.0)
        d('camera_restart_sec', 5.0)
        d('opencv_num_threads', 1)
        # template / detection
        d('ball_diameter_px', 34)
        d('matrix_square_px', 28)
        d('point_sep_px', 26)
        d('point_radius_px', 3)
        d('bank_step_deg', 15)
        d('match_thresh', 0.45)
        d('min_bright', 60)
        d('min_blob_area', 30)
        d('candidate_pad_px', 8)
        d('candidate_dilate_half_px', 0)
        d('led_search_px', 30)
        d('led_sat_min', 90)
        d('led_val_min', 60)
        d('detect_scale', 2)
        d('max_robots', 16)
        d('min_separation_px', 34.0)
        # rates / annotation
        d('track_rate_hz', 30.0)
        d('publish_rate_hz', 10.0)
        d('annotate_rate_hz', 15.0)
        d('jpeg_quality', 70)
        d('annotate_min_quality', 40)
        d('annotate_scale', 1.0)
        d('annotate_queue_max', 5)
        d('marker_republish_hz', 1.0)
        # roi / gating
        d('roi_mode', 'shared_mask')
        d('roi_scale', 3.0)
        d('roi_growth_on_miss', 1.5)
        d('roi_min_half_px', 30)
        d('roi_max_half_px', 160)
        d('max_consecutive_misses', 10)
        d('lost_timeout_sec', 2.0)
        d('reacquire_gate_cm', 25.0)
        d('innovation_gate_cm', 15.0)
        d('swap_suspect_n', 20)
        d('extrapolate_to_now', True)
        # threading
        d('match_workers', 0)
        d('match_timeout_sec', 0.05)
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
        d('reset_settle_seconds', 0.5)
        d('heartbeat_fresh_sec', 15.0)
        d('skip_compass_default', True)
        d('aim_tolerance_deg', 8.0)
        d('aim_settle_seconds', 1.5)
        d('compass_timeout_seconds', 25.0)
        # fusion
        d('body_to_field_yaw_offset', 0.0)
        d('r_cam', 1.0)
        d('r_vel', 4.0)
        d('r_ori', 2.0)
        d('r_acc', 50.0)
        d('r_gyro', 4.0)
        d('q_pos', 0.04)
        d('q_vel', 1.0)
        d('q_yaw', 1.0)
        d('q_angle', 1.0)
        d('q_acc', 100.0)
        d('q_gyro', 25.0)
        d('sensor_fresh_sec', 1.0)
        d('fuse_telemetry', False)
        d('sphere_radius_mm', 36.5)
        d('use_template_heading', False)
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
        with self._arena_lock:
            self._arena = cal
            self._arena_H = H
            self._arena_Hinv = np.linalg.inv(H)
            self._arena_poly_cm = cal.corners_cm_np
        self._republish_arena_marker()
        if self._get_state() in (INIT, IDLE):
            self._set_state(ARENA_READY)

    def _arena_snapshot(self):
        with self._arena_lock:
            return (self._arena, self._arena_H, self._arena_Hinv,
                    self._arena_poly_cm)

    # ----------------------------------------------------------------- camera
    def _pump_source(self):
        """Drive the source. No-op for v4l2 (its grab thread runs free); for sim
        this is what advances frames."""
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
        gray, _ = self._latest_frame()
        if gray is not None and self._get_state() == INIT:
            self._set_state(ARENA_READY if self._arena is not None else IDLE)

    def _latest_frame(self):
        if self._source is None:
            return None, 0.0
        return self._source.latest_gray()

    def _latest_bgr(self):
        if self._source is None:
            return None
        bgr, _stamp = self._source.latest_bgr()
        return bgr

    def _camera_heading(self, bgr, u, v, spine_deg):
        """0-360 field heading, or None if the back LED is not visible.

        Both LEDs give a continuous bearing; the red one alone only picks which
        end of the spine is the front, so it inherits the 15 deg bank step.
        """
        if bgr is None:
            return None
        red = hdg.find_led(bgr, u, v, hdg.RED_HUE, self._led_search,
                           self._led_sat_min, self._led_val_min)
        if red is None:
            return None
        green = hdg.find_led(bgr, u, v, hdg.GREEN_HUE, self._led_search,
                             self._led_sat_min, self._led_val_min)
        if green is not None:
            paired = hdg.heading_from_pair(red, green, self._pt_sep / 2.0)
            if paired is not None:
                return paired
        return hdg.heading_from_spine(spine_deg, u, v, red[0], red[1])

    def _grab_fresh(self, timeout=1.0):
        """Block until a frame with a NEW stamp arrives (or timeout)."""
        t0 = time.time()
        _g, s0 = self._latest_frame()
        while time.time() - t0 < timeout:
            self._pump_source()
            g, s = self._latest_frame()
            if g is not None and s != s0:
                return g, s
            time.sleep(0.005)
        return self._latest_frame()

    def _grab_median(self, n=5):
        """Median of n distinct frames -- suppresses sensor noise for calibration."""
        frames = []
        for _ in range(max(1, n)):
            g, _s = self._grab_fresh(0.5)
            if g is not None:
                frames.append(g)
        if not frames:
            return None
        return np.median(np.stack(frames), axis=0).astype(np.uint8)

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
        resp.success, resp.message = ok, msg
        return resp

    def _run_arena_detect(self):
        if self._long_cm <= 0 or self._short_cm <= 0:
            return (False, 'arena_long_edge_cm / arena_short_edge_cm must be set '
                           '(measure the arena)', None)
        gray = self._grab_median(self._arena_frames)
        if gray is None:
            return False, 'no camera frame', None

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
                gray, approx_eps_frac=float(gp('approx_eps_frac').value),
                canny_sigma=float(gp('canny_sigma').value),
                close_iters=int(gp('morph_close_iters').value),
                border_margin=int(gp('border_margin_px').value),
                min_area_frac=float(gp('min_quad_area_frac').value),
                max_area_frac=float(gp('max_quad_area_frac').value))
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
        resp.success = True
        resp.message = json.dumps(blobs)
        return resp

    def _run_blob_detect(self):
        gray, _s = self._grab_fresh(1.0)
        if gray is None:
            return [], 'no camera frame'
        _cal, H, _Hi, poly = self._arena_snapshot()
        hits = tmpl.detect_roi(gray, self._bank, self._match_thresh,
                               self._min_bright, self._min_blob_area,
                               self._pad_extra, self._detect_scale,
                               self._dilate_half)
        blobs = []
        for x, y, score, angle in sorted(hits, key=lambda h: -h[2]):
            x_cm, y_cm = hg.apply_homography(x, y, H)
            if not arena_mod.contains_cm(poly, x_cm, y_cm, self._arena_margin_cm):
                continue
            blobs.append({'id': len(blobs), 'u': float(x), 'v': float(y),
                          'score': round(float(score), 3),
                          'angle': float(angle),
                          'x_cm': round(x_cm, 2), 'y_cm': round(y_cm, 2)})
            if len(blobs) >= self._max_robots:
                break
        if not blobs:
            # brightness stats make a zero-blob result directly actionable: the
            # arena should read p50 2-4, so a high p50 means exposure got stomped
            # and a low p99 means the robots are not lit.
            return [], (f'no blobs (frame p50={np.percentile(gray, 50):.0f} '
                        f'p99={np.percentile(gray, 99):.0f} max={gray.max()}; '
                        f'min_bright={self._min_bright})')
        return blobs, ''

    # =====================================================================
    # Stage 3 -- linking
    # =====================================================================
    def _ensure_robot_io(self, name):
        """Create every per-robot entity from the SERVICE thread only.

        Concurrent entity creation on one node is not thread-safe in rclpy, so the
        tick must never create a publisher. It asserts presence instead.
        """
        ns = self._ns(name)
        if name not in self._pose_pubs:
            self._pose_pubs[name] = self.create_publisher(
                PoseStamped, f'/localization/{ns}/position', 10)
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
        if name not in self._sensor_subs:
            self._sensor_subs[name] = self.create_subscription(
                SpheroSensor, f'sphero/{ns}/sensors',
                lambda m, n=name: self._on_sensor(n, m), 10,
                callback_group=self._sub_group)
            self._compass_subs[name] = self.create_subscription(
                Bool, f'sphero/{ns}/calibrate_compass_done',
                lambda m, n=name: self._compass_done.__setitem__(n, bool(m.data)),
                10, callback_group=self._sub_group)

    def _on_sensor(self, name, msg):
        self._latest_sensor[name] = (msg, time.time())

    def _set_led(self, name, r, g, b, led_type='main'):
        pub = self._led_pubs.get(name)
        if pub is not None:
            pub.publish(String(data=json.dumps(
                {'type': led_type, 'red': int(r), 'green': int(g), 'blue': int(b)})))

    def _set_robot_heading(self, name, deg):
        pub = self._heading_pubs.get(name)
        if pub is not None:
            pub.publish(String(data=json.dumps({'heading': int(round(deg)) % 360})))

    def _reset_aim(self, name):
        pub = self._aim_pubs.get(name)
        if pub is not None:
            pub.publish(String(data='{}'))

    def _measure_heading(self, u, v):
        """Field heading of the robot at pixel (u, v), or None."""
        gray, _s = self._grab_fresh(1.0)
        if gray is None:
            return None
        m = tmpl.match_bank_at(gray, self._bank, u, v, self._pad_extra)
        if m is None:
            return None
        return self._camera_heading(self._latest_bgr(), m[0], m[1], m[3])

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
        self._set_led(name, 0, 255, 0, 'front')
        self._set_led(name, 255, 0, 0, 'back')
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
                have = bool(self._tracks)
            self._set_state(TRACKING if (have or state == TRACKING)
                            else BLOBS_READY)
        return resp

    def _run_linking(self, requested, blobs, skip_compass=True):
        targets = ident.select_targets(requested, self._fleet_last_seen,
                                       time.time(), self._heartbeat_fresh)
        if not targets:
            return [], [], {'_targets': []}, ('no targets: none requested and no '
                                              'fresh heartbeats on /sphero_fleet/robots')
        for name in targets:
            self._ensure_robot_io(name)

        # all dark, so the first pre-frame is a true reference
        for name in targets:
            self._set_led(name, 0, 0, 0)
        time.sleep(self._reset_settle)

        centres = [(b['u'], b['v']) for b in blobs]
        used = {idx for n, idx in self._links.items() if n not in targets}
        registered, failed, links = [], [], {}

        for name in targets:
            ok, blob_idx = self._probe_one(name, centres, used)
            if ok:
                b = blobs[blob_idx]
                if not skip_compass:
                    self._calibrate_aim(name, b)
                self._lock_tracker(name, b['x_cm'], b['y_cm'])
                links[name] = blob_idx
                used.add(blob_idx)
                registered.append(name)
            else:
                failed.append(name)

        # Status colours go on only once EVERY probe is done. Painting a robot
        # green the moment it links leaves it lit while the next robot is being
        # probed, which breaks the invariant the pre-frame above is built on
        # ("all dark, so the first pre-frame is a true reference"). It is also a
        # race: _set_led is fire-and-forget over BLE, so the green can land
        # between the next robot's pre and post frames and show up in the diff
        # as a freshly-lit blob. Exactly one robot is lit at a time during
        # probing; the operator sees the outcome colours afterwards.
        for name in registered:
            self._set_led(name, 0, 60, 0)          # green = linked
        for name in failed:
            self._set_led(name, 60, 0, 0)          # red = failed

        links['_targets'] = targets
        msg = (f'linked {len(registered)}/{len(targets)}: '
               f'{ {n: links[n] for n in registered} }')
        if failed:
            msg += f' | failed: {failed}'
        return registered, failed, links, msg

    def _probe_one(self, name, centres, used):
        """Light exactly one robot and find which blob got brighter."""
        pre, _s = self._grab_fresh(1.0)
        if pre is None:
            return False, -1
        r, g, b = self._probe_rgb
        deadline = time.time() + self._render_timeout
        result = None
        last, n_lit = None, 0
        while time.time() < deadline:
            # re-assert every poll: other nodes write LEDs too and will stomp this
            self._set_led(name, r, g, b)
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
        self._set_led(name, 0, 0, 0)
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
            return False, -1
        # wait for it to go dark again so the next probe's pre-frame is clean
        off_deadline = time.time() + self._off_timeout
        while time.time() < off_deadline:
            post, _s = self._grab_fresh(0.3)
            if post is None:
                break
            lit, _m = ident.lit_blobs_gray(post, pre, self._lit_delta,
                                           self._lit_min_area)
            if not lit:
                break
        return True, result

    def _lock_tracker(self, name, x_cm, y_cm):
        yaw0 = self._sensor_yaw_field(name)
        kf = FusionKF(x_cm, y_cm, self._fusion_params, yaw0_deg=yaw0)
        ts = TrackState(name=name, kf=kf)
        with self._tracks_lock:
            self._tracks[name] = ts
        self.get_logger().info(f'locked {name} at ({x_cm:.1f}, {y_cm:.1f}) cm')

    def _sensor_yaw_field(self, name):
        entry = self._latest_sensor.get(name)
        if not entry:
            return 0.0
        msg, _t = entry
        return float(msg.yaw) - self._fusion_params.body_to_field_yaw_offset

    # =====================================================================
    # Stage 4/5 -- reset
    # =====================================================================
    def _on_reset(self, _req, resp):
        with self._tracks_lock:
            n_tracks = len(self._tracks)
            self._tracks = {}
        with self._blobs_lock:
            n_blobs = len(self._blobs)
            self._blobs = []
        self._links, self._registered, self._failed = {}, [], []
        self._marker_ids = {}
        # publishers/subscriptions are deliberately kept: destroying rclpy
        # entities from a service thread while the tick runs is a known hazard.
        self._set_state(ARENA_READY if self._arena is not None else IDLE)
        resp.success = True
        resp.message = f'dropped {n_tracks} tracks, {n_blobs} blobs; arena kept'
        return resp

    # =====================================================================
    # The tick
    # =====================================================================
    def _tick(self):
        t_start = time.time()
        state = self._get_state()
        with self._tracks_lock:
            tracks = list(self._tracks.values())
        if state not in (TRACKING, LINKING) or not tracks:
            return
        cal, H, Hinv, poly = self._arena_snapshot()
        if H is None:
            return

        # T1/T2 -- frame + timebase from the ACQUISITION stamp, not wall-now
        gray, stamp = self._latest_frame()
        fresh = gray is not None and stamp != self._last_stamp
        lo, hi = 0.25 / self._track_rate, 4.0 / self._track_rate
        if fresh:
            dt = stamp - self._last_stamp if self._last_stamp > 0 else 1.0 / self._track_rate
        else:
            dt = t_start - self._last_tick_wall
        dt = roi_mod.clamp(dt, lo, hi)
        now = time.time()
        age = max(0.0, now - stamp) if stamp else 0.0
        self._check_camera_health(stamp, now)

        # T3 -- predict (serial; KFs are tick-thread-only, hence no lock)
        preds = {}
        for t in tracks:
            preds[t.name] = t.kf.predict(dt)

        # While LINKING, LED probes are flashing robots all over the arena.
        # Taking a camera measurement then is a direct route to a baked-in ID
        # swap, so we predict and publish only.
        allow_camera = (state == TRACKING) and fresh

        n_comps = 0
        t_match = 0.0
        if allow_camera:
            n_comps, t_match = self._camera_update(gray, tracks, preds, cal, H,
                                                   Hinv, poly)

        # T11 -- telemetry
        for t in tracks:
            self._apply_telemetry(t, now)

        # T12 -- publish. publish_rate_hz <= 0 means "every tick, as soon as the
        # pose exists": no decimation, so consumers see each estimate at the
        # track rate instead of waiting up to a publish period for it. The gate
        # could only ever fire on a tick boundary anyway, so a configured rate
        # that is not a divisor of track_rate_hz quantises badly -- 15 Hz asked
        # of a 30 Hz tick measured 11.5 Hz, because a tick running long pushes
        # the next eligible publish out a whole tick.
        if self._publish_rate <= 0 or \
                now - self._last_pub >= 1.0 / self._publish_rate:
            self._publish_tracks(tracks, age)
            self._last_pub = now

        if fresh:
            self._last_stamp = stamp
        self._last_tick_wall = t_start
        self._diag['tick_ms'] = round((time.time() - t_start) * 1000.0, 2)
        self._diag['match_ms'] = round(t_match * 1000.0, 2)
        self._diag['n_comps'] = n_comps

    def _camera_update(self, gray, tracks, preds, cal, H, Hinv, poly):
        t0 = time.time()
        h_img, w_img = gray.shape[:2]

        # T4 -- ONE mask + connected-components pass for the whole frame
        comps = tmpl.bright_components_scaled(gray, self._min_bright,
                                              self._min_blob_area,
                                              self._dilate_half,
                                              self._detect_scale)
        comps = [c for c in comps if arena_mod.contains_px(
            cal.corners_px_np, c[0], c[1], self._ball_d)]

        # T5 -- forward-predicted ROI per track
        rects = {}
        for t in tracks:
            if t.status == LOST:
                continue
            px, py = preds[t.name]
            try:
                u_pred, v_pred = hg.field_cm_to_px(px, py, Hinv)
                scale = hg.px_per_cm_at(u_pred, v_pred, H)
            except (ValueError, np.linalg.LinAlgError):
                continue
            vx, vy = t.kf.x[VX], t.kf.x[VY]
            vel_px = math.hypot(vx, vy) * scale
            half = roi_mod.roi_half(self._ball_d, self._roi_scale, vel_px,
                                    1.0 / self._track_rate, t.miss_count,
                                    self._roi_growth, self._roi_min_half,
                                    self._roi_max_half)
            rect = roi_mod.roi_from_center(u_pred, v_pred, half, w_img, h_img)
            t.roi = rect[:4]
            if roi_mod.degenerate(rect, self._tmpl_size, self._pad_extra):
                continue
            rects[t.name] = rect

        # T6 -- claim assignment on CENTROIDS, before any matching. Resolving
        # conflicts here is what stops two tracks ever being handed one blob.
        pairs = []
        for t in tracks:
            rect = rects.get(t.name)
            if rect is None:
                continue
            u_pred = 0.5 * (rect[0] + rect[2])
            v_pred = 0.5 * (rect[1] + rect[3])
            for ci, c in enumerate(comps):
                if roi_mod.contains(rect, c[0], c[1]):
                    pairs.append((math.hypot(c[0] - u_pred, c[1] - v_pred),
                                  t.name, ci))
        pairs.sort()
        claimed, taken = {}, set()
        for _d, tname, ci in pairs:
            if tname in claimed or ci in taken:
                continue
            claimed[tname] = ci
            taken.add(ci)

        # T7 -- refine each claim on the pool (OpenCV releases the GIL)
        futures = {}
        for tname, ci in claimed.items():
            cx, cy, _a, _b = comps[ci]
            futures[tname] = self._pool.submit(
                tmpl.match_bank_at, gray, self._bank, cx, cy, self._pad_extra)
        done, pending = wait(list(futures.values()), timeout=self._match_timeout)
        if pending:
            self._diag['pool_overrun'] += 1
            self.get_logger().warning(
                f'{len(pending)} ROI matches exceeded {self._match_timeout}s',
                throttle_duration_sec=5.0)

        results = {}
        for tname, fut in futures.items():
            if fut in done:
                try:
                    results[tname] = fut.result()
                except Exception:                               # noqa: BLE001
                    results[tname] = None

        # T8 -- accept / reject, serially
        accepted = {}
        by_name = {t.name: t for t in tracks}
        for tname, m in results.items():
            t = by_name[tname]
            if m is None or m[2] < self._match_thresh:
                continue
            u, v, score, angle = m
            try:
                x_cm, y_cm = hg.apply_homography(u, v, H)
            except ValueError:
                continue
            if not arena_mod.contains_cm(poly, x_cm, y_cm, self._arena_margin_cm):
                t.status = OUT_OF_ARENA
                continue
            px, py = preds[tname]
            if math.hypot(x_cm - px, y_cm - py) > self._innovation_gate:
                t.disagreements += 1
                if t.disagreements > self._swap_suspect_n:
                    t.status = SUSPECT
                continue
            accepted[tname] = (u, v, score, angle, x_cm, y_cm)

        # exclusivity on the REFINED positions: keep the stronger match
        for a in list(accepted):
            for b in list(accepted):
                if a >= b or a not in accepted or b not in accepted:
                    continue
                ua, va, sa = accepted[a][0], accepted[a][1], accepted[a][2]
                ub, vb, sb = accepted[b][0], accepted[b][1], accepted[b][2]
                if math.hypot(ua - ub, va - vb) < self._min_sep_px:
                    accepted.pop(a if sa < sb else b)

        bgr = self._latest_bgr()
        for tname, (u, v, score, angle, x_cm, y_cm) in accepted.items():
            t = by_name[tname]
            t.kf.update_camera(x_cm, y_cm)
            t.miss_count = 0
            t.status = HEALTHY
            t.last_px = (u, v)
            t.last_angle = angle
            t.last_score = score
            h = self._camera_heading(bgr, u, v, angle)
            if h is not None:
                t.last_heading = h

        # T9 -- miss ladder
        now = time.time()
        for t in tracks:
            if t.name in accepted or t.status == OUT_OF_ARENA:
                continue
            t.miss_count += 1
            if t.status not in (SUSPECT,):
                if t.miss_count > self._max_misses:
                    if t.status != LOST:
                        t.lost_since = now
                    t.status = LOST
                    if now - t.lost_since > self._lost_timeout:
                        t.status = UNRESOLVED
                else:
                    t.status = COASTING

        # T10 -- re-acquire the lost against whatever nobody claimed
        self._reacquire(tracks, comps, taken, gray, H, poly, accepted)
        return len(comps), time.time() - t0

    def _reacquire(self, tracks, comps, taken, gray, H, poly, accepted):
        lost = [t for t in tracks if t.status in (LOST, UNRESOLVED)]
        free = [ci for ci in range(len(comps)) if ci not in taken]
        if not lost or not free:
            return
        # A global re-bind while two robots overlap is exactly when a swap gets
        # committed permanently, so don't attempt one then.
        healthy_px = [t.last_px for t in tracks if t.status == HEALTHY]
        meas, meas_idx = [], []
        for ci in free:
            cx, cy, _a, _b = comps[ci]
            if any(math.hypot(cx - hx, cy - hy) < self._min_sep_px
                   for hx, hy in healthy_px):
                continue
            m = tmpl.match_bank_at(gray, self._bank, cx, cy, self._pad_extra)
            if m is None or m[2] < self._match_thresh:
                continue
            try:
                x_cm, y_cm = hg.apply_homography(m[0], m[1], H)
            except ValueError:
                continue
            if not arena_mod.contains_cm(poly, x_cm, y_cm, self._arena_margin_cm):
                continue
            meas.append((x_cm, y_cm))
            meas_idx.append((ci, m))
        if not meas:
            return
        pred_xy = [t.kf.pos_cm for t in lost]
        matches, _up, _um = associate(pred_xy, meas, self._reacquire_gate)
        for pi, mi in matches:
            t = lost[pi]
            _ci, m = meas_idx[mi]
            x_cm, y_cm = meas[mi]
            t.kf.update_camera(x_cm, y_cm)
            t.miss_count = 0
            t.status = HEALTHY
            t.last_px = (m[0], m[1])
            t.last_angle = m[3]
            t.last_score = m[2]
            self.get_logger().info(f're-acquired {t.name}')

    def _apply_telemetry(self, t, now):
        if not self._fuse_telemetry:
            return
        entry = self._latest_sensor.get(t.name)
        if not entry:
            return
        msg, recv = entry
        if now - recv > self._sensor_fresh:
            return
        # orientation FIRST so the yaw used to rotate body vectors is current
        t.kf.update_orientation(float(msg.yaw), float(msg.pitch), float(msg.roll))
        t.kf.update_velocity_body(float(msg.velocity_x), float(msg.velocity_y))
        t.kf.update_accel_body_g(float(msg.accel_x), float(msg.accel_y),
                                 float(msg.accel_z))
        t.kf.update_gyro(float(msg.gyro_x), float(msg.gyro_y), float(msg.gyro_z))

    def _publish_tracks(self, tracks, age):
        stamp = self.get_clock().now().to_msg()
        positions, statuses, dets = {}, {}, []
        for t in tracks:
            x, y = t.kf.pos_cm
            if self._extrapolate and age > 0:
                # analytically what F would do, WITHOUT mutating the filter --
                # the next tick's frame-time predict must stay monotonic
                x += t.kf.x[VX] * age
                y += t.kf.x[VY] * age
            self._publish_pose(t.name, x, y, stamp)
            positions[t.name] = (x, y)
            statuses[t.name] = t.status
            dets.append({
                'name': t.name,
                'u': round(float(t.last_px[0]), 1),   # pixel, last accepted match
                'v': round(float(t.last_px[1]), 1),
                'x_cm': round(float(x), 2),           # field, as published above
                'y_cm': round(float(y), 2),
                'angle': round(float(t.last_angle), 1),
                'heading_deg': (None if t.last_heading is None
                                else round(float(t.last_heading), 1)),
                'score': round(float(t.last_score), 3),
                'status': t.status,
                'miss': t.miss_count,
            })
        if positions:
            self._track_pub.publish(build_track_markers(
                positions, self._sphere_radius_mm, stamp, self._marker_ids,
                statuses))
        if dets:
            self._det_pub.publish(String(data=json.dumps({
                'stamp': stamp.sec + stamp.nanosec * 1e-9,
                'frame_age_s': round(float(age), 4),
                'robots': dets,
            })))

    def _publish_pose(self, name, x_cm, y_cm, stamp):
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
        msg.pose.orientation.w = 1.0
        pub.publish(msg)

    def _check_camera_health(self, stamp, now):
        if not stamp:
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
    # Annotation -- bounded FIFO, never blocks the tick
    # =====================================================================
    def _annotate_push_timer(self):
        """Feed the annotate worker in EVERY state, not only while tracking.

        The operator needs the live view to aim the camera and place the arena
        corners long before any robot is linked, so this runs off its own timer
        rather than out of the tick.
        """
        gray, stamp = self._latest_frame()
        with self._tracks_lock:
            tracks = list(self._tracks.values())
        self._push_annotate(gray, stamp, self._get_state(), tracks)

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
        with self._blobs_lock:
            blobs = list(self._blobs) if state in (BLOBS_READY, ARENA_READY) else []
        cal = self._arena
        render = {
            'state': state,
            'arena_px': cal.corners_px if cal is not None else None,
            'blobs': [{'id': b['id'], 'u': b['u'], 'v': b['v']} for b in blobs],
            'tracks': [{'name': t.name, 'u': t.last_px[0], 'v': t.last_px[1],
                        'status': t.status, 'score': t.last_score,
                        'angle': t.last_angle if self._use_tmpl_heading
                        else t.last_angle, 'roi': t.roi} for t in tracks],
            'banner': f'{state}  tick={self._diag["tick_ms"]}ms',
        }
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
        with self._tracks_lock:
            tracks = list(self._tracks.values())
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
        payload = {
            'state': self._get_state(),
            'camera': {'ok': camera_ok, **stats},
            'diag': dict(self._diag),
            'workers': self._match_workers,
            'arena': None if cal is None else {
                'long_cm': cal.long_edge_len_cm, 'short_cm': cal.short_edge_len_cm,
                'residual_max_cm': cal.residual_max_cm, 'source': cal.source},
            'n_blobs': len(self._blobs),
            'links': dict(self._links),
            'tracks': {t.name: {'status': t.status, 'miss': t.miss_count,
                                'score': round(t.last_score, 3),
                                'x_cm': round(t.kf.pos_cm[0], 1),
                                'y_cm': round(t.kf.pos_cm[1], 1),
                                'angle': round(t.last_angle, 1),
                                'heading': (None if t.last_heading is None
                                            else round(t.last_heading, 1)),
                                'disagreements': t.disagreements}
                       for t in tracks},
        }
        self._state_pub.publish(String(data=json.dumps(payload)))

    def destroy_node(self):
        # stop and JOIN the annotate worker before the publisher goes away,
        # otherwise it can try to publish into a destroyed handle on the way out
        self._annot_stop.set()
        if self._annot_thread is not None:
            self._annot_thread.join(timeout=1.0)
        try:
            self._pool.shutdown(wait=False)
        except Exception:                                       # noqa: BLE001
            pass
        if self._source is not None:
            try:
                self._source.close()
            except Exception:                                   # noqa: BLE001
                pass
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = OverheadTrackerNode()
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
