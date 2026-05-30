#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Matrix-marker detector (active LED-marker robot-ID front-end).

Per frame: mask to the calibrated field quad + exclude corner-marker regions ->
bright-blob threshold + morphology + connected components (area filtered) ->
hue classification (nearest learned centroid in HSV chroma) -> filled-vs-ring
(center vs ring intensity) -> (hue, fill) -> name lookup -> blob centroid ->
field cm via FieldMapper -> dedupe duplicate IDs.

The MATRIX is authoritative for identity; tracking never depends on any LED.
"""

import cv2
import numpy as np
from typing import Dict, List, Optional, Tuple


class MatrixMarkerDetector:
    """Detects emissive matrix markers and classifies them by (hue, fill)."""

    def __init__(
        self,
        color_model: Dict[str, dict],
        marker_to_name: Dict[Tuple[str, str], str],
        min_blob_area_px: int = 80,
        max_blob_area_px: int = 1200,
        ring_center_frac: float = 0.45,
        ring_contrast_thresh: float = 0.4,
        brightness_thresh: int = 200,
    ):
        """
        Args:
            color_model: hue_name -> {"h": <0-179>, "s": <0-255>, "v": <0-255>,
                "radius": <hue-distance>}. Learned by color_calibrator; if a hue
                is missing it falls back to the nominal palette HSV.
            marker_to_name: (hue, fill) -> robot name reverse-lookup table.
            min_blob_area_px, max_blob_area_px: connected-component area filter.
            ring_center_frac: radius fraction defining the "center" disc for the
                filled-vs-ring test.
            ring_contrast_thresh: center-vs-ring intensity ratio below which a
                blob is classified as a ring (center dark).
            brightness_thresh: V-channel threshold for the bright-blob mask.
        """
        self.color_model = color_model
        self.marker_to_name = marker_to_name
        self.min_blob_area_px = min_blob_area_px
        self.max_blob_area_px = max_blob_area_px
        self.ring_center_frac = ring_center_frac
        self.ring_contrast_thresh = ring_contrast_thresh
        self.brightness_thresh = brightness_thresh

        # Precompute hue centroids as a list for nearest-neighbor classification.
        self._hue_names: List[str] = list(self.color_model.keys())

    def detect(
        self,
        frame: np.ndarray,
        field_mask: Optional[np.ndarray] = None,
    ) -> Dict[str, np.ndarray]:
        """Detect markers in a frame.

        Args:
            frame: BGR camera frame.
            field_mask: optional uint8 mask (255 = inside field, corners
                excluded). When None, the whole frame is searched.

        Returns:
            map robot name -> camera-pixel centroid np.array([x, y]).
        """
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        v = hsv[:, :, 2]

        # 1. Bright-blob mask (emissive matrix is bright vs field).
        _, bright = cv2.threshold(v, self.brightness_thresh, 255, cv2.THRESH_BINARY)
        if field_mask is not None:
            bright = cv2.bitwise_and(bright, field_mask)

        # 2. Morphology: open then close to clean specks / fill gaps.
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
        bright = cv2.morphologyEx(bright, cv2.MORPH_OPEN, kernel)
        bright = cv2.morphologyEx(bright, cv2.MORPH_CLOSE, kernel)

        # 3. Connected components.
        num, labels, stats, centroids = cv2.connectedComponentsWithStats(bright)

        # name -> (centroid, area) so duplicate IDs keep the larger blob.
        best: Dict[str, Tuple[np.ndarray, int]] = {}

        for label in range(1, num):  # 0 is background
            area = int(stats[label, cv2.CC_STAT_AREA])
            if area < self.min_blob_area_px or area > self.max_blob_area_px:
                continue

            blob_mask = (labels == label).astype(np.uint8)

            hue = self._classify_hue(hsv, blob_mask)
            if hue is None:
                continue

            fill = self._classify_fill(v, blob_mask, stats[label], centroids[label])
            name = self.marker_to_name.get((hue, fill))
            if name is None:
                continue

            centroid = np.array([centroids[label][0], centroids[label][1]], dtype=np.float32)
            prev = best.get(name)
            if prev is None or area > prev[1]:
                best[name] = (centroid, area)

        return {name: centroid for name, (centroid, _) in best.items()}

    def _classify_hue(self, hsv: np.ndarray, blob_mask: np.ndarray) -> Optional[str]:
        """Nearest learned hue centroid (circular hue distance) within radius."""
        mean = cv2.mean(hsv, mask=blob_mask)  # (h, s, v, _)
        mean_h, mean_s = mean[0], mean[1]

        best_name = None
        best_dist = None
        for name in self._hue_names:
            model = self.color_model[name]
            dh = abs(mean_h - model["h"])
            dh = min(dh, 180.0 - dh)  # hue is circular over [0, 180)
            if dh > model.get("radius", 90.0):
                continue
            if best_dist is None or dh < best_dist:
                best_dist = dh
                best_name = name

        # Reject near-grey blobs (low chroma) that match no real hue.
        if best_name is not None and mean_s < 40:
            return None
        return best_name

    def _classify_fill(
        self,
        v: np.ndarray,
        blob_mask: np.ndarray,
        stat: np.ndarray,
        centroid: np.ndarray,
    ) -> str:
        """Filled vs ring: compare center-disc intensity to ring intensity.

        center bright -> filled; center dark -> ring.
        """
        w = int(stat[cv2.CC_STAT_WIDTH])
        h = int(stat[cv2.CC_STAT_HEIGHT])
        radius = max(w, h) / 2.0
        center_r = max(1.0, radius * self.ring_center_frac)

        cx, cy = float(centroid[0]), float(centroid[1])
        yy, xx = np.nonzero(blob_mask)
        if len(xx) == 0:
            return "filled"

        dist = np.sqrt((xx - cx) ** 2 + (yy - cy) ** 2)
        center_sel = dist <= center_r
        ring_sel = dist > center_r

        if not np.any(center_sel) or not np.any(ring_sel):
            return "filled"

        center_v = float(np.mean(v[yy[center_sel], xx[center_sel]]))
        ring_v = float(np.mean(v[yy[ring_sel], xx[ring_sel]]))

        if ring_v <= 0:
            return "filled"

        ratio = center_v / ring_v
        return "ring" if ratio < self.ring_contrast_thresh else "filled"
