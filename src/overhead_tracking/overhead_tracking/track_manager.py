#!/usr/bin/env python3
"""Per-robot tracks over blob detections (pure, no ROS).

Replaces the node's per-track association (ROI claim, innovation gate,
exclusivity, frame-count miss ladder, `_reacquire`). Call `step()` once per
processed frame with the frame's blobs already in field cm; everything is driven
by frame stamps, so 60 and 30 Hz processing behave the same.

The track set is fixed at link time (`add_track`); association never spawns one.

Per frame:
  1. predict every track to the frame stamp (HeadingEKF, inputs from the
     robot's CommandHistory).
  2. gate: a blob is a candidate for a track if it is within `gate_sigma`
     Mahalanobis on the EKF position covariance, but always within
     `gate_min_cm` and never beyond `gate_max_cm`. The gate opens while a robot
     is unseen because P grows.
  3. claim: greedy on the negative log-likelihood d2 + ln|S|, so a confident
     track wins its own blob over a coasting neighbour whose large P would
     otherwise make every blob look cheap. Position + LED theta, except for
     k-means split blobs, whose LED search window overlaps the neighbour.
  4. pairs: for two claimed tracks that each gate the other's blob, keep the
     cheaper of the two assignments (sum of d2). A track that missed a clean
     fix (COASTING or MERGED) is `pending`; while it is, a pairing whose two
     assignments are within `ambiguity_d2` of each other raises a doubt, and
     a pairing the LED heading term decides (by >= `ambiguity_d2`) clears it.
     Split blobs carry no heading, so a pair that comes apart through k-means
     splits keeps its doubt until the first clean blob, where the heading
     either resolves it or both tracks are flagged SUSPECT: the identity may
     be swapped. The flag is sticky until `clear_suspect` / `add_track` (time
     does not verify an identity; a re-link does).
  5. merges: a blob of >= `merge_area_frac` x the link-time ball area that was
     NOT split, claimed by one track while another track gates it and has no
     blob of its own, holds both robots. Both go MERGED and coast on the
     prediction; neither is updated from it (its centroid is the pair's
     midpoint: updating would pull both tracks together and destroy exactly
     the separation the step-4 assignment needs when they part). The only
     correction is a clamp: a prediction further than one ball diameter from
     the blob is pulled back to that radius (both robots are inside it), which
     bounds drift if the pair stays together, e.g. pushing at a wall.
  6. ladder (seconds): HEALTHY -> COASTING (no fix) -> LOST after
     `lost_after_s` without a fix -> UNRESOLVED `unresolved_after_s` later.
     MERGED counts as seen (we know where it is to a ball diameter).
  7. reacquire LOST/UNRESOLVED tracks against unclaimed blobs not within
     `min_sep_cm` of a claimed one, only on a mutually unambiguous match: the
     blob is the track's nearest gated blob and the track the blob's nearest
     lost track, each at most `reacquire_ratio` x the runner-up distance.
     With several tracks lost at once, a blob that two of them could equally
     be does not get assigned to either.
"""

import math
from collections import namedtuple

import numpy as np

from .ekf import CommandHistory, HeadingEKF, wrap

HEALTHY = 'HEALTHY'
COASTING = 'COASTING'
MERGED = 'MERGED'
LOST = 'LOST'
UNRESOLVED = 'UNRESOLVED'
SUSPECT = 'SUSPECT'          # reported in place of HEALTHY while .suspect is set

# x, y: field cm. area: px, as blobs.detect reports it (None = unknown).
# split: came out of a k-means split. theta: LED heading, field rad, or None.
Blob = namedtuple('Blob', 'x y area split theta', defaults=(None, False, None))


class Track:

    def __init__(self, name, t, ekf, cmds, area_px):
        self.name = name
        self.ekf = ekf
        self.cmds = cmds
        self.area_px = area_px
        self.t = float(t)                 # stamp the EKF is predicted to
        self.last_seen = float(t)
        self.lost_since = None
        self.state = HEALTHY
        self.suspect = False
        self.pending = False              # no clean fix since it coasted / merged
        self.doubt = False                # ambiguous pairing seen while pending
        self.blob = None                  # index of this frame's blob, or None
        self.led_pending = []             # (t, rad) LED readings before the first fix

    @property
    def status(self):
        return SUSPECT if (self.suspect and self.state == HEALTHY) else self.state

    @property
    def pos_cm(self):
        return self.ekf.pos_cm

    @property
    def theta(self):
        return self.ekf.theta

    @property
    def heading_deg(self):
        return self.ekf.heading_deg

    @property
    def vel_cms(self):
        return self.ekf.vel_cms


class TrackManager:

    def __init__(self, gate_sigma=3.0, gate_min_cm=15.0, gate_max_cm=50.0,
                 lost_after_s=0.5, unresolved_after_s=2.0, ambiguity_d2=4.0,
                 reacquire_ratio=0.5, merge_area_frac=1.3, ball_d_cm=7.3,
                 min_sep_cm=6.0, first_led_n=8, first_led_window_s=0.5,
                 first_led_tol_deg=15.0, ekf_kwargs=None, cmd_kwargs=None):
        self.gate_sigma2 = float(gate_sigma) ** 2
        self.gate_min = float(gate_min_cm)
        self.gate_max = float(gate_max_cm)
        self.lost_after = float(lost_after_s)
        self.unresolved_after = float(unresolved_after_s)
        self.ambiguity_d2 = float(ambiguity_d2)
        self.reacquire_ratio = float(reacquire_ratio)
        self.merge_area_frac = float(merge_area_frac)
        self.ball_d = float(ball_d_cm)
        self.min_sep = float(min_sep_cm)
        self.first_led_n = int(first_led_n)
        self.first_led_window = float(first_led_window_s)
        self.first_led_tol = math.radians(first_led_tol_deg)
        self.ekf_kwargs = dict(ekf_kwargs or {})
        self.cmd_kwargs = dict(cmd_kwargs or {})
        self.tracks = {}

    # ------------------------------------------------------------ setup
    def add_track(self, name, t, xy, theta=None, area_px=None):
        """(Re)start a robot's track at link time. area_px: its blob area."""
        ekf = HeadingEKF(xy[0], xy[1], theta, **self.ekf_kwargs)
        self.tracks[name] = Track(name, t, ekf, CommandHistory(**self.cmd_kwargs),
                                  area_px)
        return self.tracks[name]

    def add_cmd(self, name, t, speed, heading_deg):
        """A /motion_cmd echo for `name` (ignored for unknown robots)."""
        if name in self.tracks:
            self.tracks[name].cmds.add(t, speed, heading_deg)

    def clear_suspect(self, name):
        self.tracks[name].suspect = False

    @property
    def ref_area(self):
        """Median link-time ball area in px -> blobs.detect(ref_area=)."""
        a = [tr.area_px for tr in self.tracks.values() if tr.area_px]
        return float(np.median(a)) if a else None

    # ------------------------------------------------------------ helpers
    @staticmethod
    def _prep(tr):
        """Cache S^-1 of this frame's prediction (= ekf.mahalanobis2's): ~70
        distances a frame at 12 robots cost 3.5 ms through numpy per call."""
        P, R = tr.ekf.P, tr.ekf.R
        a, b, c, d = (float(P[0, 0] + R[0, 0]), float(P[0, 1] + R[0, 1]),
                      float(P[1, 0] + R[1, 0]), float(P[1, 1] + R[1, 1]))
        det = a * d - b * c
        tr.s2inv = (d / det, -b / det, -c / det, a / det)
        tr.logdet2 = math.log(det)
        tr.s3inv = None                                   # on first use

    @staticmethod
    def _s3(tr):
        if tr.s3inv is None:
            S = tr.ekf.P + tr.ekf.R
            tr.s3inv, tr.logdet3 = np.linalg.inv(S), math.log(np.linalg.det(S))
        return tr.s3inv

    def _in_gate(self, tr, b):
        d = math.hypot(b.x - tr.ekf.x[0], b.y - tr.ekf.x[1])
        if d <= self.gate_min:
            return True
        return d <= self.gate_max and self._d2(tr, b, False) <= self.gate_sigma2

    @staticmethod
    def _theta(tr, b):
        """LED heading usable for association costs (needs a theta state)."""
        return None if (b.split or b.theta is None or not tr.ekf.theta_known) \
            else b.theta

    def _led(self, tr, t, b):
        """LED heading to feed the EKF update. Unlike _theta this does not
        require theta_known: a track linked without heading takes its first
        fix here, but only once `first_led_n` readings within
        `first_led_window_s` agree to `first_led_tol_deg` of their circular
        mean (a robot with dead LEDs still yields scattered false readings),
        and then from that mean. kf_20261005_171405: SB-3660's 20 false
        readings (0.3 s burst) never have 8 within 21 deg; SB-418F's real ones
        pass in ~0.12 s (8 frames at 60 Hz)."""
        th = None if (b.split or b.theta is None) else b.theta
        if th is None or tr.ekf.theta_known:
            return th
        p = [r for r in tr.led_pending if t - r[0] <= self.first_led_window]
        p.append((t, th))
        tr.led_pending = p = p[-self.first_led_n:]
        if len(p) < self.first_led_n:
            return None
        mean = math.atan2(sum(math.sin(r[1]) for r in p),
                          sum(math.cos(r[1]) for r in p))
        if any(abs(wrap(r[1] - mean)) > self.first_led_tol for r in p):
            return None
        tr.led_pending = []
        return mean

    def _d2(self, tr, b, theta=True):
        """ekf.mahalanobis2 against the cached S of this frame's prediction."""
        th = self._theta(tr, b) if theta else None
        x = tr.ekf.x
        dx, dy = b.x - x[0], b.y - x[1]
        if th is None:
            a, b_, c, d = tr.s2inv
            return a * dx * dx + (b_ + c) * dx * dy + d * dy * dy
        y = np.array([dx, dy, wrap(th - x[2])])
        return float(y @ self._s3(tr) @ y)

    def _nll(self, tr, b):
        d2 = self._d2(tr, b)
        return d2 + (tr.logdet2 if self._theta(tr, b) is None else tr.logdet3)

    def _merged_blob(self, b, ref):
        return (not b.split and b.area is not None and ref is not None
                and b.area >= self.merge_area_frac * ref)

    # ------------------------------------------------------------ the frame
    def step(self, t, blobs):
        """Advance every track to frame stamp t (s) with this frame's blobs
        (Blob, field cm). Returns {name: blob index} of the blobs used to
        update a track this frame (claims and reacquisitions)."""
        t = float(t)
        blobs = [b if isinstance(b, Blob) else Blob(*b) for b in blobs]
        trs = list(self.tracks.values())
        for tr in trs:
            if t > tr.t:
                v, w = tr.cmds.inputs(t)
                tr.ekf.predict(t - tr.t, v, w)
                tr.t = t
            tr.blob = None
            self._prep(tr)

        # 2/3 -- gate + greedy claim for every track still being followed
        active = [tr for tr in trs if tr.state not in (LOST, UNRESOLVED)]
        gated = {tr.name: [j for j, b in enumerate(blobs) if self._in_gate(tr, b)]
                 for tr in active}
        pairs = sorted((self._nll(tr, blobs[j]), tr.name, j)
                       for tr in active for j in gated[tr.name])
        claim, owner = {}, {}
        for _c, name, j in pairs:
            if name not in claim and j not in owner:
                claim[name], owner[j] = j, name

        # 4 -- pairwise: cheaper assignment; doubt when one is recovering
        names = sorted(claim)
        for ia, a in enumerate(names):
            for b in names[ia + 1:]:
                i, j = claim[a], claim[b]
                if j not in gated[a] or i not in gated[b]:
                    continue
                ta, tb = self.tracks[a], self.tracks[b]
                dd = {th: (self._d2(ta, blobs[j], th) + self._d2(tb, blobs[i], th))
                      - (self._d2(ta, blobs[i], th) + self._d2(tb, blobs[j], th))
                      for th in (True, False)}             # swap - direct
                if dd[True] < 0:
                    claim[a], claim[b] = j, i
                    owner[i], owner[j] = b, a
                if not (ta.pending or tb.pending):
                    continue
                if abs(dd[True]) < self.ambiguity_d2:
                    ta.doubt = tb.doubt = True
                elif abs(dd[True] - dd[False]) >= self.ambiguity_d2:
                    ta.doubt = tb.doubt = False             # the LED heading decided

        # 5 -- an unsplit, oversized blob claimed by one track and gated by
        # another that found nothing: both robots are in it
        ref = self.ref_area
        merged = {}                                     # name -> blob index
        for tr in active:
            if tr.name in claim:
                continue
            for j in sorted(gated[tr.name], key=lambda j: self._d2(tr, blobs[j])):
                if j in owner and self._merged_blob(blobs[j], ref):
                    merged[tr.name] = merged[owner[j]] = j
                    claim.pop(owner[j], None)
                    break

        # update the claims. A recovering track stays pending through split
        # blobs (still touching); its first clean blob settles any doubt.
        for name, j in claim.items():
            tr, b = self.tracks[name], blobs[j]
            tr.ekf.update((b.x, b.y), self._led(tr, t, b))
            tr.state, tr.last_seen, tr.lost_since, tr.blob = HEALTHY, t, None, j
            if tr.pending and not b.split:
                tr.suspect |= tr.doubt
                tr.pending = tr.doubt = False

        for name, j in merged.items():
            tr, b = self.tracks[name], blobs[j]
            d = np.array(tr.pos_cm) - (b.x, b.y)
            n = float(np.hypot(*d))
            if n > self.ball_d:
                tr.ekf.x[:2] = np.array([b.x, b.y]) + d * (self.ball_d / n)
            tr.state, tr.last_seen, tr.pending = MERGED, t, True

        # 6 -- the ladder, in seconds
        for tr in active:
            if tr.name in claim or tr.name in merged:
                continue
            if t - tr.last_seen > self.lost_after:
                tr.state, tr.lost_since, tr.led_pending = LOST, t, []
            else:
                tr.state, tr.pending = COASTING, True
        for tr in trs:
            if tr.state == LOST and t - tr.lost_since > self.unresolved_after:
                tr.state = UNRESOLVED

        # 7 -- reacquire
        used = set(claim.values()) | set(merged.values())
        self._reacquire(t, blobs, used, claim)
        return {tr.name: tr.blob for tr in trs if tr.blob is not None}

    def _reacquire(self, t, blobs, used, claim):
        lost = [tr for tr in self.tracks.values() if tr.state in (LOST, UNRESOLVED)]
        taken = [(blobs[j].x, blobs[j].y) for j in claim.values()]
        free = [j for j, b in enumerate(blobs) if j not in used and
                all(math.hypot(b.x - x, b.y - y) >= self.min_sep for x, y in taken)]
        if not lost or not free:
            return
        D = np.full((len(lost), len(free)), np.inf)
        for i, tr in enumerate(lost):
            for k, j in enumerate(free):
                if self._in_gate(tr, blobs[j]):
                    D[i, k] = math.hypot(blobs[j].x - tr.pos_cm[0],
                                         blobs[j].y - tr.pos_cm[1])

        def clear(row):
            s = np.sort(row)
            return len(s) < 2 or s[0] <= self.reacquire_ratio * s[1]
        for i, tr in enumerate(lost):
            k = int(np.argmin(D[i]))
            if not np.isfinite(D[i, k]) or int(np.argmin(D[:, k])) != i:
                continue
            if not (clear(D[i]) and clear(D[:, k])):
                continue
            b = blobs[free[k]]
            tr.ekf.update((b.x, b.y), self._led(tr, t, b))
            tr.state, tr.last_seen, tr.lost_since, tr.blob = HEALTHY, t, None, free[k]
            tr.pending = tr.doubt = False
