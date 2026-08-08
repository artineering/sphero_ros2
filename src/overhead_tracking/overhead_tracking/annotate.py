#!/usr/bin/env python3
"""Overlay drawing and JPEG encoding (pure, no ROS).

Takes plain dicts and tuples so it is testable without a graph. Threading lives
in the node, not here -- this module only knows how to turn a frame plus a label
dict into JPEG bytes.
"""

import cv2
import numpy as np

# BGR, matching markers.STATUS_COLOR
STATUS_BGR = {
    'HEALTHY': (0, 255, 0),
    'COASTING': (0, 200, 255),
    'LOST': (0, 80, 255),
    'UNRESOLVED': (0, 0, 255),
    'OUT_OF_ARENA': (255, 100, 100),
    'SUSPECT': (255, 0, 255),
}
_FONT = cv2.FONT_HERSHEY_SIMPLEX


def draw_overlay(gray, render, ball_d=34, draw_roi=True):
    """Annotate a grayscale frame.

    `render` is a plain dict:
        {'state': str,
         'arena_px': [[u,v] x4] | None,
         'blobs': [{'id': int, 'u': float, 'v': float}],       # pre-link staging
         'tracks': [{'name', 'u', 'v', 'status', 'score',
                     'angle', 'roi': (x0,y0,x1,y1)}],
         'banner': str | None}
    """
    if gray is None:
        return None
    img = cv2.cvtColor(np.asarray(gray), cv2.COLOR_GRAY2BGR)
    r = int(ball_d // 2)
    render = render or {}

    arena = render.get('arena_px')
    if arena is not None and len(arena) >= 3:
        pts = np.asarray(arena, dtype=np.int32).reshape(-1, 1, 2)
        cv2.polylines(img, [pts], True, (0, 255, 255), 2)

    for b in render.get('blobs') or ():
        u, v = int(b['u']), int(b['v'])
        cv2.circle(img, (u, v), r, (200, 200, 200), 1)
        cv2.putText(img, f"#{b.get('id', '?')}", (u + r + 3, v),
                    _FONT, 0.45, (200, 200, 200), 1, cv2.LINE_AA)

    for t in render.get('tracks') or ():
        status = t.get('status', 'HEALTHY')
        col = STATUS_BGR.get(status, (0, 255, 0))
        u, v = int(t['u']), int(t['v'])
        if draw_roi and t.get('roi'):
            x0, y0, x1, y1 = (int(q) for q in t['roi'])
            cv2.rectangle(img, (x0, y0), (x1, y1), (70, 70, 70), 1)
        cv2.circle(img, (u, v), r, col, 2)
        ang = t.get('angle')
        if ang is not None:
            th = np.deg2rad(float(ang))
            d = r
            cv2.line(img,
                     (int(u - d * np.cos(th)), int(v - d * np.sin(th))),
                     (int(u + d * np.cos(th)), int(v + d * np.sin(th))),
                     col, 1)
        label = str(t.get('name', '?'))
        if status != 'HEALTHY':
            label = f'{label} [{status}]'
        cv2.putText(img, label, (u + r + 4, v - 2), _FONT, 0.45, col, 1,
                    cv2.LINE_AA)

    banner = render.get('banner') or render.get('state')
    if banner:
        cv2.putText(img, str(banner), (10, 24), _FONT, 0.7, (255, 255, 255), 2,
                    cv2.LINE_AA)
    return img


def encode_jpeg(bgr, quality=70, scale=1.0):
    """BGR -> JPEG bytes. Returns b'' if encoding fails rather than raising."""
    if bgr is None:
        return b''
    img = bgr
    if scale and scale != 1.0:
        img = cv2.resize(bgr, None, fx=float(scale), fy=float(scale),
                         interpolation=cv2.INTER_AREA)
    ok, buf = cv2.imencode('.jpg', img,
                           [int(cv2.IMWRITE_JPEG_QUALITY), int(quality)])
    return buf.tobytes() if ok else b''
