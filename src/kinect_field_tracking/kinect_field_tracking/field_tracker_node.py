#!/usr/bin/env python3
"""Kinect v1 overhead field tracker: calibration, registration, fused tracking.

Three responsibilities on ONE node (state machine
UNCALIBRATED -> CALIBRATED -> REGISTERING -> TRACKING):

1. FIELD CALIBRATION (~/calibrate, std_srvs/Trigger): detect the single field
   rectangle in RGB (color-agnostic Canny/contours), fit the ground plane in
   depth, back-project the 4 corners onto the plane, build the `field` frame
   (origin = bottom-left of the camera view in image space, +x long edge, +y
   short edge), broadcast a STATIC TF field->kinect_overhead, publish a corners
   Marker, and persist to config/kinect_field.yaml. On startup the YAML is loaded
   and the TF re-broadcast WITHOUT re-detecting; ~/calibrate forces a fresh pass.

2. EMPTY-ARENA BASELINE (~/capture_baseline, std_srvs/Trigger): capture the
   per-pixel depth heightmap of the empty arena and persist it as .npy.

3. REGISTRATION (~/register, multirobot_msgs/srv/Register) + TRACKING (10 Hz):
   bind each deployed callsign to a field position by serial LED probing, lock a
   per-callsign Kalman filter, then continuously publish
   /localization/<name_safe>/position (frame_id=field) at 10 Hz, fusing camera
   position with sphero/<name>/sensors telemetry (velocity/orientation/accel/
   gyro -- never telemetry position, which would be circular).

USB 2.0: depth and RGB cannot stream concurrently; every camera operation runs
under a pause that stops the tracking depth grab, and serializes depth XOR video
via source.stop().
"""

import json
import os
import threading
import time

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped, TransformStamped, Point
from sensor_msgs.msg import Image
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.callback_groups import (
    MutuallyExclusiveCallbackGroup,
    ReentrantCallbackGroup,
)
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
)
from std_msgs.msg import String, Bool
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker, MarkerArray
from builtin_interfaces.msg import Duration
import tf2_ros

from multirobot_msgs.msg import FleetState
from multirobot_msgs.srv import Register
from sphero_instance_controller.msg import SpheroSensor

import cv2

from .detection import (
    DetectConfig,
    Intrinsics,
    FreenectSource,
    SimSource,
    detect_blobs,
    masked_blur,
    capture_heightmap,
    average_depth,
    project,
)
from . import geometry as geom
from .calibration import (
    CalibrationResult,
    save_calibration,
    load_calibration,
    save_heightmap,
    load_heightmap,
)
from .fusion import FusionKF, FusionParams
from .tracking import associate
from .rgb_probe import detect_lit_blobs, match_lit_depth_blob

# --- state machine ---
UNCALIBRATED = 'UNCALIBRATED'
CALIBRATED = 'CALIBRATED'
REGISTERING = 'REGISTERING'
TRACKING = 'TRACKING'


def _name_safe(name):
    return name.replace('-', '_')


def _latched_qos():
    return QoSProfile(
        depth=1,
        durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        history=QoSHistoryPolicy.KEEP_LAST,
    )


def detect_field_rectangle(rgb, approx_eps_frac=0.02, canny_sigma=0.33,
                           close_iters=3, border_margin=6, min_area_frac=0.05):
    """Largest convex 4-vertex quad in an RGB image (color-agnostic, legacy API).

    The field boundary is often COLOURED tape on a near-neutral floor: a strong
    COLOUR edge but a weak BRIGHTNESS edge. Grayscale-only Canny misses it (the
    tape vanishes in luminance), so we OR auto-thresholded Canny over luminance
    AND the two LAB chroma channels (a, b) -- catching colour edges of ANY hue
    without thresholding a specific colour. The thin tape loop's gaps are bridged
    by a morphological CLOSE; the image border is suppressed to disconnect walls/
    doors/baseboards that touch the frame edge; then the largest-area contour's
    CONVEX HULL is reduced to 4 vertices (the hull ignores interior wood-grain
    edges and tolerates small gaps in the loop). Uses only the OpenCV 4.6 legacy
    API (no ArucoDetector).

    Returns (corners_px | None, diag) where diag feeds the calibration debug dump:
      {'edges', 'num_contours', 'num_quads', 'largest_quad_area', 'hull_verts',
       'contours_img'}.
    """
    h, w = rgb.shape[:2]
    img_area = float(w * h)

    lab = cv2.cvtColor(rgb, cv2.COLOR_RGB2LAB)
    channels = [cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY), lab[:, :, 1], lab[:, :, 2]]
    edges = np.zeros((h, w), np.uint8)
    for ch in channels:
        chb = cv2.GaussianBlur(ch, (5, 5), 0)
        v = float(np.median(chb))
        lo = int(max(0, (1.0 - canny_sigma) * v))
        hi = int(min(255, (1.0 + canny_sigma) * v))
        edges = cv2.bitwise_or(edges, cv2.Canny(chb, lo, hi))

    k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
    edges = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, k, iterations=int(close_iters))
    if border_margin > 0:
        b = int(border_margin)
        edges[:b, :] = 0
        edges[-b:, :] = 0
        edges[:, :b] = 0
        edges[:, -b:] = 0

    contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    vis = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR).copy()  # BGR for cv2.imwrite
    cv2.drawContours(vis, contours, -1, (0, 255, 255), 1)  # all contours (yellow)

    best = None
    best_area = 0.0
    hull_verts = 0
    if contours:
        big = max(contours, key=cv2.contourArea)
        hull = cv2.convexHull(big)
        cv2.polylines(vis, [hull], True, (255, 0, 0), 2)   # largest hull (blue)
        peri = cv2.arcLength(hull, True)
        for epsf in (approx_eps_frac, 0.03, 0.04, 0.05, 0.06, 0.08):
            approx = cv2.approxPolyDP(hull, epsf * peri, True)
            if epsf == approx_eps_frac:
                hull_verts = len(approx)
            if (len(approx) == 4 and cv2.isContourConvex(approx)
                    and cv2.contourArea(approx) / img_area >= min_area_frac):
                best = approx
                best_area = cv2.contourArea(approx)
                break
    if best is not None:
        cv2.polylines(vis, [best], True, (0, 0, 255), 3)   # chosen quad (red)

    diag = {
        'edges': edges,
        'num_contours': len(contours),
        'num_quads': 1 if best is not None else 0,
        'largest_quad_area': float(best_area),
        'hull_verts': int(hull_verts),
        'contours_img': vis,
    }
    corners = best.reshape(4, 2).astype(float) if best is not None else None
    return corners, diag


def rotation_to_quat(R):
    """3x3 rotation matrix -> (x, y, z, w) quaternion."""
    R = np.asarray(R, dtype=float)
    t = np.trace(R)
    if t > 0:
        s = np.sqrt(t + 1.0) * 2
        w = 0.25 * s
        x = (R[2, 1] - R[1, 2]) / s
        y = (R[0, 2] - R[2, 0]) / s
        z = (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    return float(x), float(y), float(z), float(w)


def build_track_markers(positions_cm, sphere_radius_mm, stamp, id_map,
                        lifetime_s=0.5, frame_id='field'):
    """Build a MarkerArray (METRES) for the tracked Spheros, for Foxglove.

    The /localization pose is in CM (for the device controller); these markers are
    the metric counterpart so they render in the field-frame 3D view. Per callsign:
    a SPHERE resting on the ground (z = radius) and a TEXT label above it. Stable
    ids per callsign (sphere=base, text=base+1) so updates replace; a short
    lifetime auto-expires Spheros that stop being tracked.

    `positions_cm` : {name -> (x_cm, y_cm)}; `id_map` is mutated to keep ids stable.
    """
    r_m = sphere_radius_mm / 1000.0
    sec = int(lifetime_s)
    nsec = int(round((lifetime_s - sec) * 1e9))
    arr = MarkerArray()
    for name, (x_cm, y_cm) in positions_cm.items():
        base = id_map.setdefault(name, 2 * len(id_map))
        x_m, y_m = x_cm / 100.0, y_cm / 100.0

        sph = Marker()
        sph.header.frame_id = frame_id
        sph.header.stamp = stamp
        sph.ns = 'spheros'
        sph.id = int(base)
        sph.type = Marker.SPHERE
        sph.action = Marker.ADD
        sph.pose.position.x = float(x_m)
        sph.pose.position.y = float(y_m)
        sph.pose.position.z = float(r_m)            # rest on the ground plane
        sph.pose.orientation.w = 1.0
        sph.scale.x = sph.scale.y = sph.scale.z = float(2.0 * r_m)
        sph.color.r = 0.0
        sph.color.g = 1.0
        sph.color.b = 0.0
        sph.color.a = 1.0
        sph.lifetime = Duration(sec=sec, nanosec=nsec)
        arr.markers.append(sph)

        txt = Marker()
        txt.header.frame_id = frame_id
        txt.header.stamp = stamp
        txt.ns = 'sphero_labels'
        txt.id = int(base) + 1
        txt.type = Marker.TEXT_VIEW_FACING
        txt.action = Marker.ADD
        txt.pose.position.x = float(x_m)
        txt.pose.position.y = float(y_m)
        txt.pose.position.z = float(2.0 * r_m + 0.03)
        txt.pose.orientation.w = 1.0
        txt.scale.z = 0.04
        txt.color.r = txt.color.g = txt.color.b = 1.0
        txt.color.a = 1.0
        txt.text = str(name)
        txt.lifetime = Duration(sec=sec, nanosec=nsec)
        arr.markers.append(txt)
    return arr


class FieldTrackerNode(Node):

    def __init__(self):
        super().__init__('field_tracker_node')
        self._declare_params()
        gp = self.get_parameter

        self._source_kind = gp('source').value
        self._rate_hz = float(gp('publish_rate_hz').value)
        self._dt = 1.0 / self._rate_hz
        self._marker_republish_hz = float(gp('marker_republish_hz').value)
        self._preview_enabled = bool(gp('preview_enabled').value)
        self._preview_rate_hz = float(gp('preview_rate_hz').value)
        self._heartbeat_fresh_sec = float(gp('heartbeat_fresh_sec').value)
        self._sphere_radius_mm = float(gp('sphere_radius_mm').value)
        self._gate_cm = float(gp('gate_cm').value)
        self._sensor_fresh_sec = float(gp('sensor_fresh_sec').value)

        self._intr = Intrinsics(
            fx=float(gp('fx').value), fy=float(gp('fy').value),
            cx=float(gp('cx').value), cy=float(gp('cy').value))
        # RGB camera intrinsics, used ONLY to back-project the field-rectangle
        # corners (which are detected in the RGB image, a different sensor from
        # the depth cam). Spheros are genuine depth pixels and keep self._intr.
        self._intr_rgb = Intrinsics(
            fx=float(gp('fx_rgb').value), fy=float(gp('fy_rgb').value),
            cx=float(gp('cx_rgb').value), cy=float(gp('cy_rgb').value))
        self._dcfg = DetectConfig(
            fg_threshold_mm=float(gp('fg_threshold_mm').value),
            fg_height_max_mm=float(gp('fg_height_max_mm').value),
            include_holes=bool(gp('include_holes').value),
            min_area=int(gp('min_area').value),
            max_area=int(gp('max_area').value),
            sphere_height_mm=2.0 * self._sphere_radius_mm,
            smooth_ksize=int(gp('smooth_ksize').value),
        )
        self._fparams = FusionParams(
            body_to_field_yaw_offset=float(gp('body_to_field_yaw_offset').value),
            r_cam=float(gp('r_cam').value), r_vel=float(gp('r_vel').value),
            r_ori=float(gp('r_ori').value), r_acc=float(gp('r_acc').value),
            r_gyro=float(gp('r_gyro').value),
            q_pos=float(gp('q_pos').value), q_vel=float(gp('q_vel').value),
            q_yaw=float(gp('q_yaw').value), q_angle=float(gp('q_angle').value),
            q_acc=float(gp('q_acc').value), q_gyro=float(gp('q_gyro').value))

        # probe / registration tunables
        self._probe_rgb = (int(gp('probe_red').value), int(gp('probe_green').value),
                           int(gp('probe_blue').value))
        self._rgb_du = int(gp('rgb_du').value)
        self._rgb_dv = int(gp('rgb_dv').value)
        self._probe_min_area_px = int(gp('probe_min_area_px').value)
        self._overlap_tol_px = float(gp('overlap_tol_px').value)
        self._brightness_delta_thresh = float(gp('brightness_delta_thresh').value)
        self._render_timeout = float(gp('render_timeout_seconds').value)
        self._off_timeout = float(gp('off_timeout_seconds').value)
        self._probe_poll = float(gp('probe_poll_seconds').value)
        self._reset_settle = float(gp('reset_settle_seconds').value)
        self._discovery_settle = float(gp('discovery_settle_seconds').value)
        self._compass_timeout = float(gp('compass_timeout_seconds').value)

        # calibration tunables
        self._calib_depth_frames = int(gp('calib_depth_frames').value)
        self._calib_rgb_frames = int(gp('calib_rgb_frames').value)
        self._ransac_thresh_mm = float(gp('ransac_thresh_mm').value)
        self._canny_sigma = float(gp('canny_sigma').value)
        self._morph_close_iters = int(gp('morph_close_iters').value)
        self._border_margin_px = int(gp('border_margin_px').value)
        self._min_quad_area_frac = float(gp('min_quad_area_frac').value)
        self._approx_eps_frac = float(gp('approx_eps_frac').value)
        self._auto_baseline = bool(gp('auto_baseline').value)

        try:
            share = os.path.join(
                get_package_share_directory('kinect_field_tracking'), 'config')
        except Exception:  # noqa: BLE001 - not installed (e.g. unit test)
            share = os.getcwd()
        self._field_yaml = gp('field_yaml_path').value or os.path.join(
            share, 'kinect_field.yaml')
        self._heightmap_path = gp('heightmap_path').value or os.path.join(
            share, 'kinect_heightmap.npy')

        # --- runtime state ---
        self._state = UNCALIBRATED
        self._paused = False
        # Serializes ALL camera access between the low-rate preview timer and the
        # (long, blocking) camera services so depth and video never grab at once
        # on the USB 2.0 bus. _video_streaming tracks whether the live stream is
        # currently video, so the preview re-issues source.stop() once when it
        # resumes after a depth operation (clean depth->video switch).
        self._cam_lock = threading.Lock()
        self._video_streaming = False
        self._calib = None              # CalibrationResult
        self._T_fc = None               # 4x4 field<-cam
        self._plane_n = None
        self._plane_d = None
        self._heightmap = None
        self._heightmap_smooth = None   # cached masked-blur of the fixed heightmap
        self._trackers = {}             # name -> FusionKF
        self._registered = set()
        self._failed = set()
        self._fleet_last_seen = {}      # name -> last_seen
        self._fleet_name_safe = {}      # name -> name_safe
        self._latest_sensor = {}        # name -> (SpheroSensor, recv_monotonic)
        self._led_pubs = {}
        self._compass_pubs = {}
        self._pose_pubs = {}
        self._sensor_subs = {}
        self._compass_done_subs = {}
        self._compass_done = {}         # name -> bool

        # --- source ---
        self._source = self._build_source()

        # --- callback groups ---
        self._timer_group = MutuallyExclusiveCallbackGroup()
        self._service_group = MutuallyExclusiveCallbackGroup()
        self._sub_group = ReentrantCallbackGroup()

        # --- TF + viz + status publishers ---
        self._static_tf = tf2_ros.StaticTransformBroadcaster(self)
        self._corners_pub = self.create_publisher(
            Marker, '~/field_corners', _latched_qos())
        self._reg_status_pub = self.create_publisher(
            String, '~/registration_status', _latched_qos())
        # metres viz markers for the field-frame 3D view (the /localization pose
        # is in CM for the device controller, which renders ~100x off; these
        # markers are the metric counterpart). Volatile -- republished every tick.
        self._track_markers_pub = self.create_publisher(
            MarkerArray, '~/track_markers', 10)
        self._marker_id_map = {}     # callsign -> stable base marker id

        # --- roster sub (LATCHED to match the webserver publisher) ---
        self.create_subscription(
            FleetState, '/sphero_fleet/robots', self._on_fleet, _latched_qos(),
            callback_group=self._sub_group)

        # --- services ---
        self.create_service(Trigger, '~/calibrate', self._on_calibrate,
                            callback_group=self._service_group)
        self.create_service(Trigger, '~/capture_baseline', self._on_capture_baseline,
                            callback_group=self._service_group)
        self.create_service(Register, '~/register', self._on_register,
                            callback_group=self._service_group)

        # --- RGB preview (Foxglove arena-setup view; idle states only) ---
        self._preview_pub = self.create_publisher(Image, '~/rgb_preview', 10)
        self._preview_group = MutuallyExclusiveCallbackGroup()
        if self._preview_enabled and self._preview_rate_hz > 0:
            self._preview_timer = self.create_timer(
                1.0 / self._preview_rate_hz, self._preview_tick,
                callback_group=self._preview_group)

        # --- field-corners marker republish (persistent-marker pattern) ---
        # The one-shot latched publish on calibrate does not reach volatile/late
        # subscribers (Foxglove subscribes volatile by default), so republish the
        # marker at a low rate. Pure publishing -- no camera access, no USB2 lock
        # -- so it runs in any state and just no-ops while uncalibrated. Own
        # callback group so it never contends with the tracking timer.
        self._marker_group = MutuallyExclusiveCallbackGroup()
        if self._marker_republish_hz > 0:
            self._marker_timer = self.create_timer(
                1.0 / self._marker_republish_hz, self._republish_marker,
                callback_group=self._marker_group)

        # --- tracking timer ---
        self._timer = self.create_timer(
            self._dt, self._tick, callback_group=self._timer_group)

        # --- startup: load calibration + heightmap if present ---
        self._startup_load()

        self.get_logger().info(
            f"field_tracker_node up: source={self._source_kind} "
            f"rate={self._rate_hz:.1f}Hz state={self._state}")

    # ------------------------------------------------------------------ params
    def _declare_params(self):
        self.declare_parameter('source', 'sim')
        self.declare_parameter('auto_calibrate', False)
        self.declare_parameter('auto_baseline', False)
        self.declare_parameter('field_yaml_path', '')
        self.declare_parameter('heightmap_path', '')
        self.declare_parameter('publish_rate_hz', 10.0)
        self.declare_parameter('marker_republish_hz', 1.0)
        self.declare_parameter('preview_enabled', True)
        self.declare_parameter('preview_rate_hz', 5.0)
        self.declare_parameter('heartbeat_fresh_sec', 15.0)
        self.declare_parameter('sensor_fresh_sec', 1.0)
        # intrinsics
        self.declare_parameter('fx', 594.21)
        self.declare_parameter('fy', 591.04)
        self.declare_parameter('cx', 339.31)
        self.declare_parameter('cy', 242.74)
        # RGB camera intrinsics (Kinect v1 canonical) for corner back-projection
        self.declare_parameter('fx_rgb', 525.0)
        self.declare_parameter('fy_rgb', 525.0)
        self.declare_parameter('cx_rgb', 319.5)
        self.declare_parameter('cy_rgb', 239.5)
        # detection
        self.declare_parameter('fg_threshold_mm', 25.0)
        self.declare_parameter('fg_height_max_mm', 300.0)
        self.declare_parameter('include_holes', True)
        self.declare_parameter('min_area', 40)
        self.declare_parameter('max_area', 6000)
        self.declare_parameter('smooth_ksize', 25)   # masked-blur kernel for segmentation
        self.declare_parameter('sphere_radius_mm', 36.5)
        self.declare_parameter('gate_cm', 22.0)
        # calibration
        self.declare_parameter('calib_depth_frames', 30)
        self.declare_parameter('calib_rgb_frames', 5)
        self.declare_parameter('ransac_thresh_mm', 15.0)
        self.declare_parameter('canny_sigma', 0.33)       # auto-Canny band width
        self.declare_parameter('morph_close_iters', 3)    # bridge tape-loop gaps
        self.declare_parameter('border_margin_px', 6)     # disconnect frame-edge clutter
        self.declare_parameter('min_quad_area_frac', 0.05)
        self.declare_parameter('approx_eps_frac', 0.02)
        # registration
        self.declare_parameter('probe_red', 0)
        self.declare_parameter('probe_green', 255)
        self.declare_parameter('probe_blue', 0)
        self.declare_parameter('rgb_du', 0)             # extra const RGB-x offset (px)
        self.declare_parameter('rgb_dv', 0)             # extra const RGB-y offset (px)
        self.declare_parameter('probe_min_area_px', 15)  # min lit-blob area (px)
        self.declare_parameter('overlap_tol_px', 25.0)   # lit<->projected-depth match radius
        self.declare_parameter('brightness_delta_thresh', 50.0)  # post-pre luma jump
        self.declare_parameter('render_timeout_seconds', 6.0)
        self.declare_parameter('off_timeout_seconds', 4.0)
        self.declare_parameter('probe_poll_seconds', 0.2)
        self.declare_parameter('reset_settle_seconds', 0.5)
        self.declare_parameter('discovery_settle_seconds', 1.0)
        self.declare_parameter('compass_timeout_seconds', 25.0)
        # fusion
        self.declare_parameter('body_to_field_yaw_offset', 0.0)
        self.declare_parameter('r_cam', 1.0)
        self.declare_parameter('r_vel', 4.0)
        self.declare_parameter('r_ori', 2.0)
        self.declare_parameter('r_acc', 50.0)
        self.declare_parameter('r_gyro', 4.0)
        self.declare_parameter('q_pos', 0.04)
        self.declare_parameter('q_vel', 1.0)
        self.declare_parameter('q_yaw', 1.0)
        self.declare_parameter('q_angle', 1.0)
        self.declare_parameter('q_acc', 100.0)
        self.declare_parameter('q_gyro', 25.0)
        # sim
        self.declare_parameter('sim_spheres', 4)
        self.declare_parameter('seed', 0)

    # ------------------------------------------------------------------ source
    def _build_source(self):
        kind = self._source_kind
        if kind == 'kinect':
            try:
                return FreenectSource()
            except SystemExit as e:
                self._fatal(f"source=kinect but the Kinect is unavailable: {e}")
        if kind == 'sim':
            n = int(self.get_parameter('sim_spheres').value)
            seed = int(self.get_parameter('seed').value)
            return SimSource(self._intr, self._dcfg, n, seed, probe_rgb=self._probe_rgb)
        self._fatal(f"unknown source '{kind}' (expected 'kinect' or 'sim').")

    def _fatal(self, msg):
        self.get_logger().fatal(msg)
        if rclpy.ok():
            rclpy.shutdown()
        raise SystemExit(msg)

    def _startup_load(self):
        if os.path.exists(self._heightmap_path):
            try:
                self._heightmap = load_heightmap(self._heightmap_path)
                self._heightmap_smooth = None  # invalidate cache
                self.get_logger().info(f"loaded heightmap {self._heightmap_path}")
            except Exception as e:  # noqa: BLE001
                self.get_logger().warning(f"failed to load heightmap: {e}")
        if os.path.exists(self._field_yaml):
            try:
                calib = load_calibration(self._field_yaml)
                self._apply_calibration(calib)
                self._state = CALIBRATED
                self.get_logger().info(
                    f"loaded calibration {self._field_yaml}; re-broadcast static TF")
            except Exception as e:  # noqa: BLE001
                self.get_logger().warning(f"failed to load calibration: {e}")
        elif self.get_parameter('auto_calibrate').value:
            self.get_logger().info("auto_calibrate=true and no YAML; calibrating...")
            ok, msg = self._run_calibration()
            self.get_logger().info(f"auto calibration: {ok} {msg}")

    # -------------------------------------------------------------- roster sub
    def _on_fleet(self, msg: FleetState):
        for r in msg.robots:
            self._fleet_last_seen[r.name] = r.last_seen
            self._fleet_name_safe[r.name] = r.name_safe or _name_safe(r.name)

    # ------------------------------------------------------------ calibration
    def _apply_calibration(self, calib: CalibrationResult):
        self._calib = calib
        self._T_fc = calib.T_field_from_cam_np
        self._plane_n = np.asarray(calib.plane_n, dtype=float)
        self._plane_d = float(calib.plane_d)
        self._broadcast_static_tf()
        self._publish_corners_marker()

    def _broadcast_static_tf(self):
        R_fc = self._T_fc[:3, :3]
        t_fc_mm = self._T_fc[:3, 3]
        qx, qy, qz, qw = rotation_to_quat(R_fc)
        tf = TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = self._calib.frame_parent       # field
        tf.child_frame_id = self._calib.frame_child         # kinect_overhead
        tf.transform.translation.x = float(t_fc_mm[0] / 1000.0)
        tf.transform.translation.y = float(t_fc_mm[1] / 1000.0)
        tf.transform.translation.z = float(t_fc_mm[2] / 1000.0)
        tf.transform.rotation.x = qx
        tf.transform.rotation.y = qy
        tf.transform.rotation.z = qz
        tf.transform.rotation.w = qw
        self._static_tf.sendTransform(tf)

    def _publish_corners_marker(self):
        m = Marker()
        m.header.frame_id = 'field'
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = 'field'
        m.id = 0
        m.type = Marker.LINE_STRIP
        m.action = Marker.ADD
        m.scale.x = 0.02
        m.color.r = 0.0
        m.color.g = 1.0
        m.color.b = 0.0
        m.color.a = 1.0
        corners = list(self._calib.corners_field_cm)
        for (x_cm, y_cm) in corners + corners[:1]:  # close the loop
            p = Point()
            p.x = float(x_cm) / 100.0
            p.y = float(y_cm) / 100.0
            p.z = 0.0
            m.points.append(p)
        self._corners_pub.publish(m)

    def _republish_marker(self):
        """Low-rate persistent republish so volatile/late subscribers see it."""
        if self._calib is None:
            return
        self._publish_corners_marker()

    def _on_calibrate(self, request, response):
        self._paused = True
        self._cam_lock.acquire()
        try:
            ok, msg = self._run_calibration()
            response.success = ok
            response.message = msg
        finally:
            self._video_streaming = False
            self._cam_lock.release()
            self._paused = False
        return response

    def _run_calibration(self):
        """Detect rectangle + fit plane -> field frame; persist + broadcast TF."""
        try:
            # serialize: depth average, then RGB median (depth XOR video)
            self._source.stop()
            depth = average_depth(self._source, self._calib_depth_frames)
            self._source.stop()
            rgb = self._grab_rgb_median(self._calib_rgb_frames)
        except Exception as e:  # noqa: BLE001
            return False, f"capture failed: {e}"

        corners_px, diag = detect_field_rectangle(
            rgb, approx_eps_frac=self._approx_eps_frac,
            canny_sigma=self._canny_sigma, close_iters=self._morph_close_iters,
            border_margin=self._border_margin_px,
            min_area_frac=self._min_quad_area_frac)
        # Always dump debug PNGs (success OR failure) for diagnosis.
        self._dump_calib_debug(rgb, diag)
        if corners_px is None:
            return False, (
                f"no field rectangle detected: {diag['num_contours']} contours, "
                f"largest-hull approx had {diag['hull_verts']} vertices "
                f"(need 4 convex), best quad area={diag['largest_quad_area']:.0f}px2; "
                f"debug PNGs in {os.path.dirname(self._field_yaml) or '.'}")

        pts = self._depth_to_camera_points(depth)
        if len(pts) < 100:
            return False, f"too few valid depth points for plane fit ({len(pts)})"
        n, d, _ = geom.fit_plane_ransac(pts, thresh_mm=self._ransac_thresh_mm)
        n, d = geom.orient_normal_toward_camera(n, d)

        try:
            # INTERIM HACK: the corners are detected in the RGB image, so their
            # rays are formed with the RGB intrinsics (a different sensor from the
            # depth cam, fx_rgb~525 vs fx~594), then intersected with the SAME
            # depth-fitted ground plane (n, d). This corrects the field SCALE to
            # ~5-10cm. It still IGNORES the ~2.5cm RGB-depth baseline and uses
            # canonical (not per-device) RGB intrinsics.
            # TODO(proper-fix): use registered depth (FREENECT_DEPTH_REGISTERED)
            # + RGB intrinsics, or a homography from the RGB corners to the
            # measured field dimensions, for true metric accuracy.
            corners_cam = np.array([
                geom.backproject_corner(u, v, n, d, self._intr_rgb)
                for (u, v) in corners_px])
        except ValueError as e:
            return False, f"corner back-projection failed: {e}"

        res = geom.build_field_frame(corners_cam, corners_px, n)

        calib = CalibrationResult(
            intrinsics={'fx': self._intr.fx, 'fy': self._intr.fy,
                        'cx': self._intr.cx, 'cy': self._intr.cy},
            corners_px=corners_px.tolist(),
            corners_cam_mm=corners_cam.tolist(),
            corners_field_cm=np.asarray(res['corners_field_cm']).tolist(),
            plane_n=np.asarray(n).tolist(),
            plane_d=float(d),
            T_field_from_cam=res['T_field_from_cam'].tolist(),
            origin_corner_index=res['origin_idx'],
            long_edge_len_cm=res['long_edge_len_cm'],
            short_edge_len_cm=res['short_edge_len_cm'])
        try:
            save_calibration(self._field_yaml, calib)
        except Exception as e:  # noqa: BLE001
            return False, f"could not save {self._field_yaml}: {e}"

        self._apply_calibration(calib)
        if self._state == UNCALIBRATED:
            self._state = CALIBRATED

        if self._auto_baseline:
            self._capture_baseline_impl()

        return True, (f"field calibrated: long={res['long_edge_len_cm']:.1f}cm "
                      f"short={res['short_edge_len_cm']:.1f}cm; static TF broadcast")

    def _depth_to_camera_points(self, depth, stride=4):
        """Back-project valid depth pixels (subsampled) to camera-frame 3D mm."""
        sub = depth[::stride, ::stride]
        ys, xs = np.nonzero(sub > 0)
        if xs.size == 0:
            return np.empty((0, 3))
        u = xs * stride
        v = ys * stride
        z = sub[ys, xs]
        X = (u - self._intr.cx) * z / self._intr.fx
        Y = (v - self._intr.cy) * z / self._intr.fy
        return np.column_stack([X, Y, z]).astype(float)

    def _grab_rgb_median(self, n_frames):
        frames = []
        for _ in range(max(1, n_frames)):
            frames.append(self._source.get_video())
            time.sleep(0.03)
        return np.median(np.stack(frames), axis=0).astype(np.uint8)

    def _dump_calib_debug(self, rgb, diag):
        """Save the captured RGB, the Canny edges, and (if any contours) the
        contour/quad overlay as PNGs in the calibration dir, for diagnosis.

        Written on EVERY ~/calibrate attempt. RGB is converted to BGR for
        cv2.imwrite; the overlay (diag['contours_img']) is already BGR.
        """
        out_dir = os.path.dirname(self._field_yaml) or '.'
        try:
            os.makedirs(out_dir, exist_ok=True)
            cv2.imwrite(os.path.join(out_dir, 'calib_rgb.png'),
                        cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))
            cv2.imwrite(os.path.join(out_dir, 'calib_edges.png'), diag['edges'])
            if diag['num_contours'] > 0:
                cv2.imwrite(os.path.join(out_dir, 'calib_contours.png'),
                            diag['contours_img'])
            self.get_logger().info(
                f"calib debug: {diag['num_contours']} contours, "
                f"{diag['num_quads']} quads, "
                f"largest={diag['largest_quad_area']:.0f}px2 -> {out_dir}")
        except Exception as e:  # noqa: BLE001
            self.get_logger().warning(f"calib debug dump failed: {e}")

    # ----------------------------------------------------------- baseline svc
    def _on_capture_baseline(self, request, response):
        self._paused = True
        self._cam_lock.acquire()
        try:
            ok, msg = self._capture_baseline_impl()
            response.success = ok
            response.message = msg
        finally:
            self._video_streaming = False
            self._cam_lock.release()
            self._paused = False
        return response

    def _capture_baseline_impl(self):
        try:
            self._source.stop()
            hm = capture_heightmap(self._source, self._calib_depth_frames)
            save_heightmap(self._heightmap_path, hm)
            self._heightmap = hm
            self._heightmap_smooth = None  # invalidate cache
        except Exception as e:  # noqa: BLE001
            return False, f"baseline capture failed: {e}"
        valid = int((hm > 0).sum())
        return True, f"heightmap captured ({valid} valid px) -> {self._heightmap_path}"

    # ----------------------------------------------------------- led helpers
    def _led_pub(self, name):
        if name not in self._led_pubs:
            ns = self._fleet_name_safe.get(name, _name_safe(name))
            self._led_pubs[name] = self.create_publisher(String, f'sphero/{ns}/led', 10)
        return self._led_pubs[name]

    def _set_led(self, name, led_type, rgb):
        msg = String()
        msg.data = json.dumps({'type': led_type, 'red': int(rgb[0]),
                               'green': int(rgb[1]), 'blue': int(rgb[2])})
        self._led_pub(name).publish(msg)

    def _reset_leds(self, name):
        for t in ('main', 'front', 'back'):
            self._set_led(name, t, (0, 0, 0))

    def _light_probe(self, name):
        self._set_led(name, 'main', self._probe_rgb)
        self._set_led(name, 'front', (0, 0, 0))
        self._set_led(name, 'back', (0, 0, 0))

    def _compass_pub(self, name):
        if name not in self._compass_pubs:
            ns = self._fleet_name_safe.get(name, _name_safe(name))
            self._compass_pubs[name] = self.create_publisher(
                String, f'sphero/{ns}/calibrate_compass', 10)
        return self._compass_pubs[name]

    # --------------------------------------------------------- sensors subs
    def _ensure_sensor_sub(self, name):
        if name in self._sensor_subs:
            return
        ns = self._fleet_name_safe.get(name, _name_safe(name))
        self._sensor_subs[name] = self.create_subscription(
            SpheroSensor, f'sphero/{ns}/sensors',
            lambda msg, n=name: self._on_sensor(n, msg), 10,
            callback_group=self._sub_group)

    def _on_sensor(self, name, msg: SpheroSensor):
        self._latest_sensor[name] = (msg, time.monotonic())

    # --------------------------------------------------- compass-done subs
    def _ensure_compass_done_sub(self, name):
        if name in self._compass_done_subs:
            return
        ns = self._fleet_name_safe.get(name, _name_safe(name))
        self._compass_done_subs[name] = self.create_subscription(
            Bool, f'sphero/{ns}/calibrate_compass_done',
            lambda msg, n=name: self._on_compass_done(n, msg), 10,
            callback_group=self._sub_group)

    def _on_compass_done(self, name, msg: Bool):
        self._compass_done[name] = True

    # -------------------------------------------------------------- register
    def _on_register(self, request, response):
        if self._calib is None:
            response.success = False
            response.message = "not calibrated; call ~/calibrate first"
            return response
        prev_state = self._state
        self._state = REGISTERING
        self._paused = True
        self._cam_lock.acquire()
        try:
            self._run_registration(request, response)
        except Exception as e:  # noqa: BLE001
            response.success = False
            response.message = f"registration error: {e}"
            self.get_logger().error(response.message)
        finally:
            self._video_streaming = False
            self._cam_lock.release()
            self._paused = False
            # always end in TRACKING so locked trackers publish
            self._state = TRACKING if (self._trackers or prev_state == TRACKING) else CALIBRATED
        return response

    def _run_registration(self, request, response):
        from .registration import select_targets, merge_registration_status, \
            registration_status_payload

        now = time.time()
        targets = select_targets(
            list(request.callsigns), dict(self._fleet_last_seen),
            now, self._heartbeat_fresh_sec)
        if not targets:
            response.success = False
            response.message = "no targets (no fresh-heartbeat Spheros and no explicit callsigns)"
            return response

        for name in targets:
            self._led_pub(name)
            self._ensure_sensor_sub(name)
            self._ensure_compass_done_sub(name)
        if self._discovery_settle > 0:
            time.sleep(self._discovery_settle)

        timed_out_compass = []
        if not request.skip_compass:
            timed_out_compass = self._phase0_compass(targets)

        # Phase 1: reset all targets' LEDs
        for name in targets:
            self._reset_leds(name)
        if self._reset_settle > 0:
            time.sleep(self._reset_settle)

        # Phase 2: serial, camera-confirmed probe
        new_registered, new_failed = [], []
        for name in targets:
            ok = self._probe_one(name)
            (new_registered if ok else new_failed).append(name)

        # final LEDs: green = registered, red = failed
        for name in new_registered:
            self._set_led(name, 'main', (0, 255, 0))
        for name in new_failed:
            self._set_led(name, 'main', (255, 0, 0))

        # merge into the full status (subset re-registration preserves the rest)
        self._registered, self._failed = merge_registration_status(
            self._registered, self._failed, set(targets),
            set(new_registered), set(new_failed))

        payload = registration_status_payload(self._registered, self._failed)
        status = String()
        status.data = json.dumps(payload)
        self._reg_status_pub.publish(status)

        msg = (f"registered {len(new_registered)}/{len(targets)}: "
               f"{', '.join(new_registered) or '(none)'}")
        if new_failed:
            msg += f"; failed: {', '.join(new_failed)}"
        if timed_out_compass:
            msg += f"; compass-timeout: {', '.join(timed_out_compass)}"
        response.success = True
        response.registered = new_registered
        response.failed = new_failed
        response.message = msg
        self.get_logger().info(msg)
        return response

    def _phase0_compass(self, targets):
        """Trigger compass cal on all targets in parallel; wait for done/timeout."""
        for name in targets:
            self._compass_done[name] = False
        for name in targets:
            m = String()
            m.data = '{}'
            self._compass_pub(name).publish(m)
        deadline = time.monotonic() + self._compass_timeout
        while time.monotonic() < deadline:
            if all(self._compass_done.get(n, False) for n in targets):
                break
            time.sleep(0.1)
        return [n for n in targets if not self._compass_done.get(n, False)]

    def _probe_one(self, name):
        """Light one Sphero, confirm by GREEN/depth OVERLAP, lock a KF.

        Depth gives detection + position; the lit green only confirms WHICH depth
        blob we lit. The green Sphero appears tens of pixels from the depth pixel
        (RGB vs depth sensor offset), so we match by overlap of a detected green
        RGB blob with the depth blob's centre PROJECTED into RGB (via the RGB
        intrinsics), within overlap_tol_px -- not pixel-exact patches.
        """
        # 1. confirm dark, detect blobs in depth
        self._reset_leds(name)
        try:
            self._source.stop()
            depth = self._source.get_depth()
            depth = self._source.get_depth()
        except Exception as e:  # noqa: BLE001
            self.get_logger().warning(f"{name}: depth grab failed: {e}")
            return False
        baseline, baseline_smooth = self._seg_baseline(depth)
        meas, _ = detect_blobs(depth, baseline, self._intr, self._dcfg,
                               baseline_smooth=baseline_smooth)
        if not meas:
            self.get_logger().warning(f"{name}: no depth blobs detected")
            return False
        # project each depth blob's 3D centre into RGB pixels (interim: RGB
        # intrinsics, ignores the ~2.5cm RGB-depth baseline -- TODO registered depth)
        projected = [self._project_depth_blob_to_rgb(m) for m in meas]

        # 2. pre (dark) RGB, 3. light, 4. poll until a green blob overlaps a depth blob
        try:
            self._source.stop()
            pre_rgb = self._grab_rgb()
        except Exception as e:  # noqa: BLE001
            self.get_logger().warning(f"{name}: pre RGB grab failed: {e}")
            pre_rgb = None
        self._light_probe(name)
        result, post_rgb, lit_blobs = self._wait_until_lit_overlap(projected, pre_rgb)
        ok = result['matched']
        if ok:
            m = meas[int(result['depth_index'])]
            self._lock_tracker(name, m)
            self.get_logger().info(
                f"{name}: confirmed lit (overlap dist={result['distance']:.1f}px)")
        else:
            self.get_logger().warning(
                f"{name}: not confirmed lit ({result['reason']}, "
                f"best dist={result['distance']:.1f}px tol={self._overlap_tol_px:.0f})")

        # debug dump (success OR fail) for live overlap verification
        self._dump_reg_debug(name, post_rgb if post_rgb is not None else pre_rgb,
                             projected, lit_blobs, result)

        # 6. reset + confirm off (no lit blob overlaps any depth blob)
        self._reset_leds(name)
        if not self._wait_until_off_overlap(projected, pre_rgb):
            self.get_logger().warning(f"{name}: still lit after reset (timeout)")
        return ok

    def _project_depth_blob_to_rgb(self, meas):
        """Project a depth blob's 3D camera-frame centre into RGB pixels."""
        X_mm, Y_mm, Z_mm, _u, _v = meas
        pu, pv = project(X_mm, Y_mm, Z_mm, self._intr_rgb)
        return (pu + self._rgb_du, pv + self._rgb_dv)

    def _grab_rgb(self):
        if not hasattr(self._source, 'get_video'):
            return None
        return self._source.get_video()

    def _detect_lit(self, post_rgb, pre_rgb):
        """Newly-bright blobs vs the dark pre-frame (robust to LED blow-out)."""
        if post_rgb is None:
            return []
        blobs, _ = detect_lit_blobs(
            post_rgb, pre_rgb, brightness_delta=self._brightness_delta_thresh,
            min_area=self._probe_min_area_px)
        return blobs

    def _wait_until_lit_overlap(self, projected, pre_rgb):
        """Poll RGB until a newly-bright blob overlaps a depth blob, or timeout.

        Returns (match_result, last_post_rgb, last_lit_blobs).
        """
        deadline = time.monotonic() + self._render_timeout
        post = self._grab_rgb()
        lit = self._detect_lit(post, pre_rgb)
        result = match_lit_depth_blob(projected, lit, self._overlap_tol_px)
        while not result['matched'] and time.monotonic() < deadline:
            time.sleep(self._probe_poll)
            post = self._grab_rgb()
            lit = self._detect_lit(post, pre_rgb)
            result = match_lit_depth_blob(projected, lit, self._overlap_tol_px)
        return result, post, lit

    def _wait_until_off_overlap(self, projected, pre_rgb):
        """Poll RGB until NO newly-bright blob overlaps a depth blob, or timeout."""
        deadline = time.monotonic() + self._off_timeout
        while time.monotonic() < deadline:
            lit = self._detect_lit(self._grab_rgb(), pre_rgb)
            if not match_lit_depth_blob(projected, lit, self._overlap_tol_px)['matched']:
                return True
            time.sleep(self._probe_poll)
        return False

    def _dump_reg_debug(self, name, rgb, projected, lit_blobs, result):
        """Save reg_<name_safe>.png overlaying projected depth centres, detected
        bright-change (lit) blobs, the match line + decision, into the calib dir."""
        if rgb is None:
            return
        out_dir = os.path.dirname(self._field_yaml) or '.'
        ns = self._fleet_name_safe.get(name, _name_safe(name))
        tol = int(self._overlap_tol_px)
        try:
            os.makedirs(out_dir, exist_ok=True)
            vis = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR).copy()
            for di, (pu, pv) in enumerate(projected):
                c = (int(round(pu)), int(round(pv)))
                cv2.circle(vis, c, tol, (0, 255, 255), 1)   # tolerance radius
                cv2.circle(vis, c, 3, (0, 255, 255), -1)    # projected depth centre
                cv2.putText(vis, f"d{di}", (c[0] + 5, c[1] - 5),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 1)
            for lb in lit_blobs:
                x, y, w, hh = lb['bbox']
                cv2.rectangle(vis, (x, y), (x + w, y + hh), (255, 0, 255), 2)  # lit (magenta)
                gu, gv = lb['centroid']
                cv2.circle(vis, (int(round(gu)), int(round(gv))), 3, (255, 0, 255), -1)
            di = result.get('depth_index')
            gj = result.get('green_index')
            if di is not None and gj is not None and gj < len(lit_blobs):
                pu, pv = projected[di]
                gu, gv = lit_blobs[gj]['centroid']
                colour = (0, 0, 255) if result['matched'] else (0, 0, 150)
                cv2.line(vis, (int(round(pu)), int(round(pv))),
                         (int(round(gu)), int(round(gv))), colour, 2)
            label = ("MATCH d%d dist=%.1f" % (di, result['distance'])
                     if result['matched']
                     else "NO-MATCH (%s) dist=%.1f" % (result['reason'], result['distance']))
            cv2.putText(vis, label, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                        (0, 0, 255) if not result['matched'] else (0, 200, 0), 2)
            cv2.imwrite(os.path.join(out_dir, f'reg_{ns}.png'), vis)
            self.get_logger().info(
                f"{name}: reg debug -> {out_dir}/reg_{ns}.png "
                f"({len(lit_blobs)} lit blob(s), dist={result['distance']:.1f}px)")
        except Exception as e:  # noqa: BLE001
            self.get_logger().warning(f"{name}: reg debug dump failed: {e}")

    def _lock_tracker(self, name, meas):
        """Initialize a FusionKF for `name` at the blob's field contact point."""
        X_mm, Y_mm, Z_mm, u, v = meas
        contact_cam = geom.contact_point_cam_mm(
            u, v, Z_mm, self._intr, self._plane_n, self._plane_d, self._sphere_radius_mm)
        x_cm, y_cm = geom.cam_mm_to_field_cm(contact_cam, self._T_fc)
        yaw0 = self._sensor_yaw_field(name)
        self._trackers[name] = FusionKF(x_cm, y_cm, self._fparams, yaw0_deg=yaw0)
        self._pose_pub(name)
        self.get_logger().info(f"{name}: locked at field ({x_cm:.1f},{y_cm:.1f})cm yaw0={yaw0:.0f}")

    def _sensor_yaw_field(self, name):
        entry = self._latest_sensor.get(name)
        if entry is None:
            return 0.0
        msg, _ = entry
        return float(msg.yaw) - self._fparams.body_to_field_yaw_offset

    def _fallback_baseline(self, depth):
        """Scalar-ish baseline if no heightmap yet (median valid depth)."""
        valid = depth[depth > 0]
        floor = float(np.median(valid)) if valid.size else 2000.0
        return np.full_like(depth, floor)

    def _seg_baseline(self, depth):
        """Return (baseline, baseline_smooth) for detect_blobs.

        The fixed heightmap is masked-blurred ONCE and cached (re-blurring it
        every 10Hz call would be wasteful); the per-call fallback baseline is left
        for detect_blobs to blur inline.
        """
        if self._heightmap is not None:
            if self._dcfg.smooth_ksize > 1 and self._heightmap_smooth is None:
                self._heightmap_smooth = masked_blur(
                    self._heightmap, self._dcfg.smooth_ksize)
            return self._heightmap, self._heightmap_smooth
        return self._fallback_baseline(depth), None

    # ------------------------------------------------------------- pose pubs
    def _pose_pub(self, name):
        if name not in self._pose_pubs:
            ns = self._fleet_name_safe.get(name, _name_safe(name))
            self._pose_pubs[name] = self.create_publisher(
                PoseStamped, f'/localization/{ns}/position', 10)
        return self._pose_pubs[name]

    # ------------------------------------------------------------------ timer
    def _tick(self):
        if self._paused or self._state != TRACKING or not self._trackers:
            return
        try:
            depth = self._source.get_depth()
        except Exception as e:  # noqa: BLE001
            self.get_logger().warning(f"depth grab failed in tick: {e}")
            return
        baseline, baseline_smooth = self._seg_baseline(depth)
        meas, _ = detect_blobs(depth, baseline, self._intr, self._dcfg,
                               baseline_smooth=baseline_smooth)
        meas_field = self._blobs_to_field(meas)
        self._fuse_and_publish(meas_field)

    def _blobs_to_field(self, meas):
        out = []
        for (X_mm, Y_mm, Z_mm, u, v) in meas:
            contact = geom.contact_point_cam_mm(
                u, v, Z_mm, self._intr, self._plane_n, self._plane_d,
                self._sphere_radius_mm)
            out.append(geom.cam_mm_to_field_cm(contact, self._T_fc))
        return out

    def _fuse_and_publish(self, meas_field):
        names = list(self._trackers.keys())
        # predict every tracker
        preds = []
        for name in names:
            preds.append(self._trackers[name].predict(self._dt))
        matches, _, _ = associate(preds, meas_field, self._gate_cm)
        matched_idx = {ti: mi for ti, mi in matches}
        # update each tracker: camera (if matched) + telemetry (if fresh)
        stamp = self.get_clock().now().to_msg()
        now = time.monotonic()
        positions_cm = {}
        for ti, name in enumerate(names):
            kf = self._trackers[name]
            if ti in matched_idx:
                mx, my = meas_field[matched_idx[ti]]
                kf.update_camera(mx, my)
            self._apply_telemetry(name, kf, now)
            self._publish_pose(name, kf, stamp)   # /localization PoseStamped (CM)
            positions_cm[name] = kf.pos_cm
        self._publish_track_markers(positions_cm, stamp)

    def _publish_track_markers(self, positions_cm, stamp):
        """Publish metres viz markers (SPHERE + label) for the field-frame 3D view."""
        arr = build_track_markers(
            positions_cm, self._sphere_radius_mm, stamp, self._marker_id_map)
        self._track_markers_pub.publish(arr)

    def _apply_telemetry(self, name, kf, now):
        entry = self._latest_sensor.get(name)
        if entry is None:
            return
        msg, recv = entry
        if (now - recv) > self._sensor_fresh_sec:
            return  # stale telemetry: predict-only this cycle
        # orientation first so the yaw estimate is fresh for vel/accel rotation
        kf.update_orientation(msg.yaw, msg.pitch, msg.roll)
        kf.update_velocity_body(msg.velocity_x, msg.velocity_y)
        kf.update_accel_body_g(msg.accel_x, msg.accel_y, msg.accel_z)
        kf.update_gyro(msg.gyro_x, msg.gyro_y, msg.gyro_z)

    def _publish_pose(self, name, kf, stamp):
        x_cm, y_cm = kf.pos_cm
        pose = PoseStamped()
        pose.header.stamp = stamp
        pose.header.frame_id = 'field'
        pose.pose.position.x = float(x_cm)
        pose.pose.position.y = float(y_cm)
        pose.pose.position.z = 0.0
        pose.pose.orientation.w = 1.0
        self._pose_pub(name).publish(pose)

    # ---------------------------------------------------------- rgb preview
    def _preview_tick(self):
        """Publish an RGB frame for Foxglove -- ONLY when the device is idle.

        Streams in UNCALIBRATED/CALIBRATED, never while paused or during
        REGISTERING/TRACKING (those grab depth; depth XOR video on USB 2.0). The
        camera lock prevents collision with the camera services; _video_streaming
        re-issues source.stop() once on resume for a clean depth->video switch.
        """
        if not self._preview_enabled:
            return
        if self._paused or self._state not in (UNCALIBRATED, CALIBRATED):
            return
        if not self._cam_lock.acquire(blocking=False):
            return
        frame = None
        try:
            if not self._video_streaming:
                self._source.stop()          # clean switch onto the video stream
                self._video_streaming = True
            frame = self._source.get_video()
        except Exception as e:  # noqa: BLE001 - transient grab error: skip tick
            self.get_logger().warning(f"preview frame grab failed: {e}")
        finally:
            self._cam_lock.release()
        if frame is not None:
            self._publish_preview(frame)

    def _publish_preview(self, frame):
        h, w = frame.shape[:2]
        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'kinect_overhead'
        msg.height = int(h)
        msg.width = int(w)
        msg.encoding = 'rgb8'
        msg.is_bigendian = 0
        msg.step = int(w) * 3
        msg.data = np.ascontiguousarray(frame, dtype=np.uint8).tobytes()
        self._preview_pub.publish(msg)

    def destroy_node(self):
        try:
            if getattr(self, '_source', None) is not None:
                self._source.close()
        finally:
            super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    executor = None
    try:
        node = FieldTrackerNode()
        executor = MultiThreadedExecutor()
        executor.add_node(node)
        executor.spin()
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
