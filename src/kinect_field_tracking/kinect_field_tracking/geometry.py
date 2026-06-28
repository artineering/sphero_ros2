#!/usr/bin/env python3
"""Pure field-calibration geometry (no ROS, no hardware).

All math for turning a Kinect depth+RGB capture into a calibrated `field` frame:
  * ground-plane fit (RANSAC + least-squares refit),
  * pixel ray <-> plane intersection (corner back-projection),
  * field-frame construction (origin = bottom-left of the camera view in IMAGE
    space, +x along the LONGER edge, +y along the SHORTER edge, z up, right-handed),
  * camera->field 4x4 transform + point transforms,
  * contact-point radius offset (blob-centroid ray -> ball centre -> ground).

Units: depth/camera-frame in mm; the field frame is reported in cm (the shared
/localization contract). Conversions happen here so the node stays unit-clean.

Camera optical frame convention (matches backproject): +x right, +y down,
+z forward (into the scene). The overhead ground is therefore roughly at large
+z, and the plane normal points back toward the camera (-z component).
"""

import numpy as np

MM_TO_CM = 10.0


# --------------------------------------------------------------------------- #
# Ground-plane fit
# --------------------------------------------------------------------------- #
def fit_plane_lsq(points):
    """Least-squares plane n.X + d = 0 (unit |n|) through Nx3 points.

    Returns (n, d) with n a unit normal. Solves via the centroid + smallest
    singular vector of the centred points.
    """
    pts = np.asarray(points, dtype=float).reshape(-1, 3)
    centroid = pts.mean(axis=0)
    centred = pts - centroid
    _, _, vh = np.linalg.svd(centred, full_matrices=False)
    n = vh[-1]
    n = n / np.linalg.norm(n)
    d = -float(n @ centroid)
    return n, d


def fit_plane_ransac(points, thresh_mm=15.0, iters=200, seed=0):
    """RANSAC ground-plane fit, refit on inliers. Returns (n, d, inlier_mask).

    Falls back to a plain least-squares fit when there are too few points.
    """
    pts = np.asarray(points, dtype=float).reshape(-1, 3)
    if len(pts) < 3:
        n, d = fit_plane_lsq(pts)
        return n, d, np.ones(len(pts), dtype=bool)

    rng = np.random.default_rng(seed)
    best_inliers = None
    best_count = -1
    for _ in range(iters):
        idx = rng.choice(len(pts), size=3, replace=False)
        p0, p1, p2 = pts[idx]
        nrm = np.cross(p1 - p0, p2 - p0)
        norm = np.linalg.norm(nrm)
        if norm < 1e-9:
            continue
        nrm = nrm / norm
        d = -float(nrm @ p0)
        dist = np.abs(pts @ nrm + d)
        inliers = dist < thresh_mm
        count = int(inliers.sum())
        if count > best_count:
            best_count = count
            best_inliers = inliers

    if best_inliers is None or best_count < 3:
        n, d = fit_plane_lsq(pts)
        return n, d, np.ones(len(pts), dtype=bool)

    n, d = fit_plane_lsq(pts[best_inliers])  # refit on inliers
    return n, d, best_inliers


def orient_normal_toward_camera(n, d):
    """Flip (n, d) so the normal points toward the camera (origin side).

    The camera optical centre is at the origin; the signed distance of the
    origin to the plane is d. We want the half-space containing the camera to be
    the +normal side, i.e. n . (camera - point_on_plane) > 0. Picking n so its
    +z faces the camera (n_z < 0, since the ground is at +z forward) is the
    physically-meaningful "up" for an overhead camera.
    """
    n = np.asarray(n, dtype=float)
    if n[2] > 0:
        return -n, -d
    return n, d


# --------------------------------------------------------------------------- #
# Pixel ray <-> plane
# --------------------------------------------------------------------------- #
def pixel_ray(u, v, intr):
    """Unit-ish direction of the ray through pixel (u,v) (camera frame)."""
    dx = (u - intr.cx) / intr.fx
    dy = (v - intr.cy) / intr.fy
    d = np.array([dx, dy, 1.0], dtype=float)
    return d / np.linalg.norm(d)


def ray_plane_intersection(origin, direction, n, d):
    """Intersection of ray (origin + t*direction) with plane n.X + d = 0.

    Returns the 3D point (mm). Raises ValueError if the ray is parallel.
    """
    origin = np.asarray(origin, dtype=float)
    direction = np.asarray(direction, dtype=float)
    denom = float(n @ direction)
    if abs(denom) < 1e-9:
        raise ValueError("ray parallel to plane")
    t = -(float(n @ origin) + d) / denom
    return origin + t * direction


def backproject_corner(u, v, n, d, intr):
    """Corner pixel -> camera-frame 3D point on the fitted ground plane (mm)."""
    direction = pixel_ray(u, v, intr)
    return ray_plane_intersection(np.zeros(3), direction, n, d)


# --------------------------------------------------------------------------- #
# Field-frame construction
# --------------------------------------------------------------------------- #
def select_bottom_left(corners_px):
    """Index of the bottom-left corner in IMAGE space (operator overhead view).

    Bottom = largest pixel v (image y grows downward); among the two
    bottom-most corners, left = smallest pixel u.
    """
    corners_px = np.asarray(corners_px, dtype=float).reshape(-1, 2)
    order_v = np.argsort(corners_px[:, 1])      # ascending v
    bottom_two = order_v[-2:]                    # two largest v
    # of those, the smaller u is the left one
    i0, i1 = bottom_two
    return int(i0) if corners_px[i0, 0] <= corners_px[i1, 0] else int(i1)


def _plane_basis(plane_n):
    """An orthonormal 2D basis (u, v) spanning the plane with normal plane_n."""
    n = np.asarray(plane_n, dtype=float)
    n = n / np.linalg.norm(n)
    ref = np.array([1.0, 0.0, 0.0]) if abs(n[0]) < 0.9 else np.array([0.0, 1.0, 0.0])
    u = ref - (ref @ n) * n
    u = u / np.linalg.norm(u)
    v = np.cross(n, u)
    v = v / np.linalg.norm(v)
    return u, v


def cyclic_order(corners_cam_mm, plane_n):
    """Indices of the 4 metric corners in cyclic (ring) order around the field.

    Projects the corners onto the fitted ground plane (2D), then sorts by angle
    about their centroid. The resulting ring gives TRUE rectangle adjacency,
    perspective-invariantly.
    """
    pts = np.asarray(corners_cam_mm, dtype=float).reshape(-1, 3)
    u, v = _plane_basis(plane_n)
    rel = pts - pts.mean(axis=0)
    angles = np.arctan2(rel @ v, rel @ u)
    return [int(i) for i in np.argsort(angles)]


def _adjacent_corners(corners_cam_mm, plane_n, origin_idx):
    """The two rectangle corners edge-adjacent to origin_idx (perspective-invariant).

    Adjacency is taken from rectangle TOPOLOGY, not pixel distance: the corners
    are ordered cyclically around their centroid in the ground-plane 2D basis, so
    the origin's two ring-neighbours are the real edge-adjacent corners and the
    corner two steps away is the diagonal. Pixel distance is WRONG under a tilted
    (non-overhead) camera -- perspective compresses the far edge, so the diagonal
    corner can appear closer in pixels than a true edge-adjacent corner.
    """
    ring = cyclic_order(corners_cam_mm, plane_n)
    pos = ring.index(int(origin_idx))
    return ring[(pos - 1) % 4], ring[(pos + 1) % 4]


def build_field_frame(corners_cam_mm, corners_px, plane_n):
    """Construct the camera->field transform and the field-frame corners.

    Parameters
    ----------
    corners_cam_mm : (4,3) corners on the ground plane in the camera frame (mm).
    corners_px : (4,2) the same corners' pixels (used to pick bottom-left + edges
        in image space).
    plane_n : the ground-plane unit normal, oriented toward the camera ("up").

    Returns dict:
      {
        'origin_idx': int,
        'R_fc': (3,3) camera->field rotation (rows = field x,y,z in camera frame),
        't_fc': (3,) field-frame origin expressed so X_field = R_fc @ X_cam + t_fc,
        'T_field_from_cam': (4,4),
        'T_cam_from_field': (4,4),
        'corners_field_cm': (4,2) the corners in field coords (z ~ 0),
        'long_edge_len_cm': float, 'short_edge_len_cm': float,
      }
    """
    corners_cam_mm = np.asarray(corners_cam_mm, dtype=float).reshape(-1, 3)
    corners_px = np.asarray(corners_px, dtype=float).reshape(-1, 2)
    up = np.asarray(plane_n, dtype=float)
    up = up / np.linalg.norm(up)

    o_idx = select_bottom_left(corners_px)
    a_idx, b_idx = _adjacent_corners(corners_cam_mm, up, o_idx)

    O = corners_cam_mm[o_idx]
    ea = corners_cam_mm[a_idx] - O
    eb = corners_cam_mm[b_idx] - O
    # project edges into the plane (drop any normal component) for clean axes
    ea = ea - (ea @ up) * up
    eb = eb - (eb @ up) * up

    if np.linalg.norm(ea) >= np.linalg.norm(eb):
        long_e, short_e = ea, eb
    else:
        long_e, short_e = eb, ea

    x_hat = long_e / np.linalg.norm(long_e)
    z_hat = up
    y_hat = np.cross(z_hat, x_hat)
    y_hat = y_hat / np.linalg.norm(y_hat)
    # ensure +y points along the short edge from the origin
    if y_hat @ short_e < 0:
        y_hat = -y_hat
        z_hat = np.cross(x_hat, y_hat)  # keep right-handed
    z_hat = z_hat / np.linalg.norm(z_hat)

    R_fc = np.vstack([x_hat, y_hat, z_hat])     # rows are field axes in cam frame
    t_fc = -R_fc @ O                            # X_field(mm) = R_fc @ X_cam + t_fc

    T_field_from_cam = np.eye(4)
    T_field_from_cam[:3, :3] = R_fc
    T_field_from_cam[:3, 3] = t_fc
    T_cam_from_field = np.linalg.inv(T_field_from_cam)

    corners_field_mm = (R_fc @ corners_cam_mm.T).T + t_fc
    corners_field_cm = corners_field_mm[:, :2] / MM_TO_CM
    long_len_cm = float(np.linalg.norm(long_e) / MM_TO_CM)
    short_len_cm = float(np.linalg.norm(short_e) / MM_TO_CM)

    return {
        'origin_idx': int(o_idx),
        'R_fc': R_fc,
        't_fc': t_fc,
        'T_field_from_cam': T_field_from_cam,
        'T_cam_from_field': T_cam_from_field,
        'corners_field_cm': corners_field_cm,
        'long_edge_len_cm': long_len_cm,
        'short_edge_len_cm': short_len_cm,
    }


# --------------------------------------------------------------------------- #
# Point transforms
# --------------------------------------------------------------------------- #
def cam_mm_to_field_cm(point_cam_mm, T_field_from_cam):
    """Camera-frame point (mm) -> field-frame point (cm), returns (x_cm, y_cm)."""
    p = np.asarray(point_cam_mm, dtype=float).reshape(3)
    ph = np.array([p[0], p[1], p[2], 1.0])
    f = T_field_from_cam @ ph
    return float(f[0] / MM_TO_CM), float(f[1] / MM_TO_CM)


# --------------------------------------------------------------------------- #
# Contact-point radius offset
# --------------------------------------------------------------------------- #
def contact_point_cam_mm(u, v, z_mm, intr, n, d, sphere_radius_mm):
    """Ground-contact point (camera frame, mm) for a lit/detected ball blob.

    The blob centroid sits on the TOP of the ball (the surface the camera sees),
    along the camera ray. Step inward along that ray by sphere_radius_mm to reach
    the ball CENTRE, then drop the centre vertically onto the ground plane (along
    the plane normal) to get the ground contact point.

    Returns the contact point as a 3D camera-frame coordinate (mm).
    """
    direction = pixel_ray(u, v, intr)
    surface = direction * (z_mm / direction[2])  # the blob surface point at depth z
    centre = surface + direction * sphere_radius_mm
    # drop the centre onto the plane along the normal: subtract signed distance
    n = np.asarray(n, dtype=float)
    signed = float(n @ centre + d)
    contact = centre - signed * n
    return contact
