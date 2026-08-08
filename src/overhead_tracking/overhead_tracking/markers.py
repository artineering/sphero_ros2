#!/usr/bin/env python3
"""Field-frame visualisation markers (metres).

The /localization pose is in CM because that is the device controller's contract;
these markers are the metric counterpart so they render correctly in a 3D view.
Publishing the cm values directly would place every robot 100x too far out.
"""

from builtin_interfaces.msg import Duration
from visualization_msgs.msg import Marker, MarkerArray

# status -> RGB
STATUS_COLOR = {
    'HEALTHY': (0.0, 1.0, 0.0),
    'COASTING': (1.0, 0.8, 0.0),
    'LOST': (1.0, 0.3, 0.0),
    'UNRESOLVED': (1.0, 0.0, 0.0),
    'OUT_OF_ARENA': (0.4, 0.4, 1.0),
    'SUSPECT': (1.0, 0.0, 1.0),
}


def build_track_markers(positions_cm, sphere_radius_mm, stamp, id_map,
                        statuses=None, lifetime_s=0.5, frame_id='field'):
    """MarkerArray (metres) for the tracked Spheros.

    Per callsign: a SPHERE resting on the ground (z = radius) and a TEXT label
    above it. Ids are stable per callsign (sphere=base, text=base+1) so updates
    replace rather than accumulate; a short lifetime auto-expires robots that
    stop being tracked.

    `positions_cm` : {name -> (x_cm, y_cm)}; `id_map` is mutated to keep ids stable.
    """
    statuses = statuses or {}
    r_m = sphere_radius_mm / 1000.0
    sec = int(lifetime_s)
    nsec = int(round((lifetime_s - sec) * 1e9))
    arr = MarkerArray()
    for name, (x_cm, y_cm) in positions_cm.items():
        base = id_map.setdefault(name, 2 * len(id_map))
        x_m, y_m = x_cm / 100.0, y_cm / 100.0
        r, g, b = STATUS_COLOR.get(statuses.get(name, 'HEALTHY'), (0.0, 1.0, 0.0))

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
        sph.color.r, sph.color.g, sph.color.b = float(r), float(g), float(b)
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


def build_arena_marker(corners_cm, stamp, frame_id='field', marker_id=0):
    """Closed LINE_STRIP of the arena boundary, in metres."""
    from geometry_msgs.msg import Point
    m = Marker()
    m.header.frame_id = frame_id
    m.header.stamp = stamp
    m.ns = 'arena'
    m.id = int(marker_id)
    m.type = Marker.LINE_STRIP
    m.action = Marker.ADD
    m.pose.orientation.w = 1.0
    m.scale.x = 0.02
    m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 1.0, 0.0, 1.0
    pts = list(corners_cm) + [corners_cm[0]]          # close the loop
    for x_cm, y_cm in pts:
        p = Point()
        p.x, p.y, p.z = float(x_cm) / 100.0, float(y_cm) / 100.0, 0.0
        m.points.append(p)
    return m
