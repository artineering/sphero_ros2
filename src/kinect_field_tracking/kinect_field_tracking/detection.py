#!/usr/bin/env python3
"""Kinect v1 depth detection primitives (ported, self-contained).

PORTED BY COPY from the deprecated aruco_slam/kinect_tracking.py. No runtime
dependency on aruco_slam. Contains the validated depth-geometry helpers
(Intrinsics, back-project/project), the empty-arena foreground detector
(detect_blobs against a per-pixel heightmap baseline), the heightmap capture
routine, and the two depth/RGB sources (FreenectSource real Kinect, SimSource
synthetic).

USB 2.0 constraint: the Kinect v1 cannot sustain DEPTH and VIDEO isochronous
streams concurrently (they corrupt with "Invalid magic"). FreenectSource.stop()
tears down whichever sync stream is live so callers can serialize depth XOR
video by stopping one before grabbing the other.
"""

import time
from dataclasses import dataclass

import cv2
import numpy as np

W, H = 640, 480  # Kinect v1 depth resolution


# --------------------------------------------------------------------------- #
# Geometry
# --------------------------------------------------------------------------- #
@dataclass
class Intrinsics:
    fx: float = 594.21
    fy: float = 591.04
    cx: float = 339.31
    cy: float = 242.74


def backproject(u, v, z, intr):
    """Pixel (u,v) at depth z (mm) -> camera-frame (X, Y, Z) in mm."""
    X = (u - intr.cx) * z / intr.fx
    Y = (v - intr.cy) * z / intr.fy
    return X, Y, z


def project(X, Y, z, intr):
    """Camera-frame (X,Y,z) mm -> pixel (u, v)."""
    u = X * intr.fx / z + intr.cx
    v = Y * intr.fy / z + intr.cy
    return u, v


# --------------------------------------------------------------------------- #
# Detection config (foreground segmentation + blob filtering)
# --------------------------------------------------------------------------- #
@dataclass
class DetectConfig:
    # foreground vs. per-pixel heightmap baseline (mm)
    fg_threshold_mm: float = 25.0   # |baseline - live| > this => foreground
    fg_height_max_mm: float = 300.0
    include_holes: bool = True
    sphere_height_mm: float = 73.0  # ball diameter, fallback depth for all-hole blobs
    # depth smoothing for SEGMENTATION ONLY (masked Gaussian; <=1 disables).
    # Collapses the subtraction speckle band; the solid 73mm ball survives, only
    # isolated speckle dies. Blob z is still taken from RAW depth (see detect_blobs).
    smooth_ksize: int = 25
    # blob filtering (pixels)
    min_area: int = 40
    max_area: int = 6000


# --------------------------------------------------------------------------- #
# Masked (normalized) depth smoothing -- ignores invalid (0) pixels
# --------------------------------------------------------------------------- #
def masked_blur(depth, ksize):
    """Normalized Gaussian blur that ignores invalid (0) depth pixels.

    out = GaussianBlur(depth*valid) / GaussianBlur(valid); out[depth<=0] = 0.
    Zeros (invalid returns) do not bleed into valid regions. ksize is forced odd;
    ksize <= 1 returns the input unchanged.
    """
    k = int(ksize)
    if k <= 1:
        return depth
    if k % 2 == 0:
        k += 1
    valid = (depth > 0).astype(np.float32)
    d = depth.astype(np.float32) * valid
    num = cv2.GaussianBlur(d, (k, k), 0)
    den = cv2.GaussianBlur(valid, (k, k), 0)
    out = np.zeros_like(d)
    nz = den > 1e-6
    out[nz] = num[nz] / den[nz]
    out[depth <= 0] = 0.0
    return out


# --------------------------------------------------------------------------- #
# Detection: heightmap subtraction -> blob centroids (camera-frame mm)
# --------------------------------------------------------------------------- #
def detect_blobs(live, baseline, intr, cfg, baseline_smooth=None):
    """Foreground blobs vs. a per-pixel empty-arena depth heightmap.

    The camera is not guaranteed perfectly overhead, so the empty-arena ground
    depth varies per pixel; `baseline` is a per-pixel heightmap (NOT a scalar).
    A pixel is foreground when its depth differs from the heightmap by more than
    fg_threshold_mm (object closer than the floor), or, for glossy shells, when
    a previously-valid floor pixel now returns no depth (a hole).

    Returns (measurements, mask):
      measurements : list of (X_mm, Y_mm, Z_mm, u, v) per accepted blob, where
                     (X,Y,Z) is the back-projected camera-frame centroid and
                     (u,v) its pixel. The pixel + Z are needed downstream for the
                     contact-point radius offset and RGB sampling.
      mask         : uint8 foreground mask (diagnostics).

    Depth smoothing (cfg.smooth_ksize > 1) masked-blurs baseline + live to build
    the SEGMENTATION mask only; blob z is taken from the RAW `live` depth so the
    blur never biases position. Pass `baseline_smooth` (pre-blurred baseline) to
    avoid re-blurring the fixed baseline every call.
    """
    k = int(getattr(cfg, 'smooth_ksize', 1))
    if k > 1:
        seg_live = masked_blur(live, k)
        seg_base = baseline_smooth if baseline_smooth is not None else masked_blur(baseline, k)
    else:
        seg_live = live
        seg_base = baseline

    valid_bg = seg_base > 0
    diff = seg_base - seg_live  # positive => object is closer than the floor
    closer = (valid_bg & (seg_live > 0)
              & (diff > cfg.fg_threshold_mm) & (diff < cfg.fg_height_max_mm))
    if cfg.include_holes:
        holes = valid_bg & (seg_live <= 0)  # glossy ball: floor return disappears
        mask = (closer | holes)
    else:
        mask = closer
    mask = mask.astype(np.uint8) * 255

    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.dilate(mask, kernel, iterations=1)

    n, labels, stats, centroids = cv2.connectedComponentsWithStats(mask, connectivity=8)
    measurements = []
    for i in range(1, n):
        area = stats[i, cv2.CC_STAT_AREA]
        if area < cfg.min_area or area > cfg.max_area:
            continue
        u, v = centroids[i]
        comp = labels == i
        # z from RAW live depth within the component (smoothing must NOT bias position)
        zs = live[comp]
        zs = zs[zs > 0]
        if zs.size:
            z = float(np.median(zs))
        else:  # all-holes blob: fall back to floor depth minus a ball
            z = float(baseline[int(round(v)), int(round(u))]) - cfg.sphere_height_mm
        if z <= 0:
            continue
        X, Y, _ = backproject(u, v, z, intr)
        measurements.append((X, Y, z, float(u), float(v)))
    return measurements, mask


# --------------------------------------------------------------------------- #
# Baseline (empty-arena per-pixel heightmap) capture
# --------------------------------------------------------------------------- #
def capture_heightmap(source, n_frames):
    """Average n_frames of (empty-arena) depth into a per-pixel heightmap.

    Per-pixel mean over valid (>0) returns; pixels that were never valid stay 0
    (treated as no-baseline by detect_blobs).
    """
    acc = np.zeros((H, W), dtype=np.float64)
    cnt = np.zeros((H, W), dtype=np.float64)
    for _ in range(n_frames):
        d = source.get_depth()
        valid = d > 0
        acc[valid] += d[valid]
        cnt[valid] += 1
        time.sleep(0.03)
    base = np.zeros((H, W), dtype=np.float32)
    nz = cnt > 0
    base[nz] = (acc[nz] / cnt[nz]).astype(np.float32)
    return base


def average_depth(source, n_frames):
    """Robust averaged single depth frame (per-pixel mean over valid returns).

    Used by field calibration to fit the ground plane on a clean depth image.
    """
    return capture_heightmap(source, n_frames)


# --------------------------------------------------------------------------- #
# Depth/RGB sources
# --------------------------------------------------------------------------- #
class FreenectSource:
    """Real Kinect v1 via libfreenect's synchronous python API."""

    def __init__(self):
        try:
            import freenect  # lazy import so sim mode needs no hardware deps
        except ImportError as e:
            raise SystemExit(
                "freenect python bindings not found. Build OpenKinect/libfreenect "
                "with -DBUILD_PYTHON3=ON before using source:=kinect."
            ) from e
        self._fn = freenect
        # FREENECT_DEPTH_MM: registered-to-depth-cam millimetres, 0 = invalid.
        self._fmt = getattr(freenect, "DEPTH_MM", getattr(freenect, "DEPTH_REGISTERED", 0))

    def get_depth(self):
        depth, _ = self._fn.sync_get_depth(format=self._fmt)
        return depth.astype(np.float32)

    def get_video(self):
        """One RGB video frame as a contiguous uint8 HxWx3 (row-major R,G,B)."""
        video, _ = self._fn.sync_get_video()
        return np.ascontiguousarray(video, dtype=np.uint8)

    def stop(self):
        """Tear down whichever sync stream is live WITHOUT destroying the wrapper.

        The Kinect v1 over USB 2.0 cannot sustain depth + RGB concurrently. Stop
        one stream before grabbing the other; the next sync_get_* re-inits that
        single stream cleanly.
        """
        try:
            self._fn.sync_stop()
        except Exception:
            pass

    def close(self):
        try:
            self._fn.sync_stop()
        except Exception:
            pass


class SimSource:
    """Synthetic flat arena with N colliding blobs (no hardware).

    Also synthesizes an RGB frame: each blob is rendered as a disc in a fixed
    "probe" colour controlled by set_lit()/clear_lit(), so registration's
    lit-blob detection can be exercised without a Kinect.
    """

    def __init__(self, intr, cfg, n_spheres, seed, plane=2000.0, noise=6.0,
                 sphere_radius=36.5, probe_rgb=(0, 255, 0)):
        self.intr = intr
        self.cfg = cfg
        self.plane = plane
        self.noise = noise
        self.radius = sphere_radius
        self.probe_rgb = probe_rgb
        self.rng = np.random.default_rng(seed)
        self.bx, self.by = 900.0, 650.0
        self.pos = np.column_stack([
            self.rng.uniform(-self.bx, self.bx, n_spheres),
            self.rng.uniform(-self.by, self.by, n_spheres),
        ])
        self.vel = self.rng.uniform(-22.0, 22.0, (n_spheres, 2))  # mm/frame
        self._lit_index = None  # which blob is currently lit (for RGB probe sim)

    # -- registration test hooks ------------------------------------------- #
    def set_lit(self, index):
        self._lit_index = int(index)

    def clear_lit(self):
        self._lit_index = None

    def _resolve_collisions(self):
        min_d = 2.0 * self.radius
        n = len(self.pos)
        for i in range(n):
            for j in range(i + 1, n):
                d = self.pos[j] - self.pos[i]
                dist = float(np.hypot(d[0], d[1]))
                if dist == 0.0 or dist >= min_d:
                    continue
                normal = d / dist
                overlap = min_d - dist
                self.pos[i] -= normal * (overlap / 2.0)
                self.pos[j] += normal * (overlap / 2.0)
                dv = self.vel[i] - self.vel[j]
                along = float(np.dot(dv, normal))
                if along > 0.0:
                    self.vel[i] -= along * normal
                    self.vel[j] += along * normal

    def baseline(self):
        return np.full((H, W), self.plane, dtype=np.float32)

    def step(self):
        """Advance the sim one frame (positions only) without grabbing depth."""
        self.pos += self.vel
        for k in range(2):
            lim = self.bx if k == 0 else self.by
            over = np.abs(self.pos[:, k]) > lim
            self.pos[over, k] = np.clip(self.pos[over, k], -lim, lim)
            self.vel[over, k] *= -1.0
        self._resolve_collisions()

    def _z(self):
        return self.plane - self.cfg.sphere_height_mm

    def get_depth(self):
        self.step()
        z = self._z()
        depth = np.full((H, W), self.plane, dtype=np.float32)
        rad_px = int(round(self.radius * self.intr.fx / z))
        for X, Y in self.pos:
            u, v = project(X, Y, z, self.intr)
            cv2.circle(depth, (int(round(u)), int(round(v))), rad_px, float(z), -1)
        depth += self.rng.normal(0.0, self.noise, depth.shape).astype(np.float32)
        return depth

    def get_video(self):
        """RGB frame: every blob a DIM disc; the lit blob (set_lit) BLOWN-OUT white.

        Registration detects the lit Sphero by brightness INCREASE vs the dark
        pre-frame (the real LED saturates the Kinect RGB to white), so the lit
        disc must be clearly brighter than the unlit/background -- not merely a
        different colour. Unlit discs stay dim and equal in pre/post (diff ~0).
        """
        z = self._z()
        rad_px = int(round(self.radius * self.intr.fx / z))
        rgb = np.zeros((H, W, 3), dtype=np.uint8)
        for i, (X, Y) in enumerate(self.pos):
            u, v = project(X, Y, z, self.intr)
            c = (int(round(u)), int(round(v)))
            if self._lit_index is not None and i == self._lit_index:
                cv2.circle(rgb, c, rad_px, (255, 255, 255), -1)  # lit: blown-out white
            else:
                cv2.circle(rgb, c, rad_px, (60, 60, 60), -1)     # unlit: dim
        return rgb

    def stop(self):
        """No-op: the sim has no isochronous streams to tear down."""
        pass

    def close(self):
        pass
