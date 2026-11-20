"""A measured point on the white nose cap, within a supplied fish box."""
import cv2
import numpy as np

from white_black_tracker import WhiteBlackTracker
from path_guidance import VelocityEstimator


HEAD_GAP_S = .25  # Since last accepted camera measurement, not last control tick.


class WhiteHeadPoint:
    def __init__(self):
        self.colour = WhiteBlackTracker()
        self.previous = None
        self.previous_time = None
        self.template = None
        self.source = 'MISSING'

    def measure(self, frame, box, timestamp):
        self.source = 'MISSING'
        if box is None:
            self.template = self.previous_time = None
            return None
        point = self._detect(frame, box, timestamp)
        centre = .5*(np.asarray(box[:2])+box[2:])
        length = np.linalg.norm(np.asarray(box[2:])-box[:2])
        if point is not None:
            # Capture the cap and its boundary with the hull, not a uniform white patch.
            r = max(3, round(.075*length))
            u,v = np.rint(point).astype(int)
            chip = frame[max(0,v-r):v+r+1,max(0,u-r):u+r+1]
            self.template = None
            if chip.shape[:2] == (2*r+1,2*r+1):
                self.template = (cv2.cvtColor(chip,cv2.COLOR_BGR2GRAY),point-centre,timestamp)
            self.source = 'CAP'
            return tuple(map(float,point))
        if self.template is None:
            return None
        chip,offset,last = self.template
        if not 0 < timestamp-last <= .5:
            return None
        # Look for current-frame image evidence close to the last confirmed cap,
        # compensating for translation of the fish box. Never output the prediction.
        r = chip.shape[0]//2
        u,v = np.rint(centre+offset).astype(int)
        margin = r+max(3,round(.09*length))
        x,y = max(0,u-margin),max(0,v-margin)
        right,bottom=min(frame.shape[1],u+margin+1),min(frame.shape[0],v+margin+1)
        if right <= x or bottom <= y:
            return None
        patch = frame[y:bottom,x:right]
        if min(patch.shape[:2]) < 2*r+1 or chip.std()<12:
            return None
        gray = cv2.cvtColor(patch,cv2.COLOR_BGR2GRAY)
        _,score,_,(dx,dy) = cv2.minMaxLoc(cv2.matchTemplate(gray,chip,cv2.TM_CCOEFF_NORMED))
        if score < .76:
            return None
        cap = patch[dy+r-1:dy+r+2,dx+r-1:dx+r+2]
        _,s,vv = cv2.split(cv2.cvtColor(cap,cv2.COLOR_BGR2HSV))
        if np.median(s)>65 or np.median(vv)<max(110,.75*float(np.median(chip[r-1:r+2,r-1:r+2]))):
            return None
        self.source = 'PATCH'
        return float(x+dx+r),float(y+dy+r)

    def _detect(self, frame, box, timestamp):
        x, y, right, bottom = map(int, box)
        length = np.hypot(right - x, bottom - y)
        if length < 15:
            return None
        pad = round(.06 * length)
        x, y = max(0, x-pad), max(0, y-pad)
        patch = frame[y:bottom+pad, x:right+pad]
        if patch.size == 0:
            return None
        hsv = cv2.cvtColor(patch, cv2.COLOR_BGR2HSV)
        h, s, v = cv2.split(hsv)
        dark = ((v < 115) & ((s < 60) | (h < 75) | (h > 135))).astype('uint8')
        if length >= 65:
            dark = cv2.morphologyEx(dark, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
        n, labels, stats, centers = cv2.connectedComponentsWithStats(dark)
        cy, cx = np.array(patch.shape[:2]) / 2
        eligible = [k for k in range(1, n) if stats[k, 4] > .012 * length**2
                    and np.linalg.norm(centers[k] - [cx, cy]) < .30 * length]
        if not eligible:
            return None
        body = (labels == max(eligible, key=lambda k: stats[k, 4])).astype('uint8')
        depth = cv2.distanceTransform(body, cv2.DIST_L2, 5)
        radius = depth.max()
        core_y, core_x = np.where(depth > .35 * radius)
        if radius < .035 * length or len(core_x) < 12:
            return None
        core = np.column_stack((core_x, core_y))
        center = core.mean(axis=0)
        values, vectors = np.linalg.eigh(np.cov(core.T))
        if values[1] < 2 * values[0]:
            return None
        axis = vectors[:, 1]
        yy, xx = np.indices(body.shape)
        along = (xx-center[0])*axis[0] + (yy-center[1])*axis[1]
        across = abs((yy-center[1])*axis[0] - (xx-center[0])*axis[1])
        # Keep only white pixels touching the black hull's end, close to its
        # centreline. Side fins and small white tail joints are not nose caps.
        gap = cv2.distanceTransform(1-body, cv2.DIST_L2, 5)
        # Warm off-white or plainly neutral white under a cooler camera balance.
        warmth = (patch[:, :, 1].astype('int16') + patch[:, :, 2])/2 - patch[:, :, 0]
        neutral = (s <= 25) & (v >= 150)
        # On a grey/white surround, do not mistake the background clipped by
        # the axial search mask for a cap. A neutral cap must stand out there.
        if neutral.mean() > .45:
            neutral &= v.astype(float) > np.median(v[neutral])+10
        white = self.colour.binary_preview(patch).astype(bool) & ((warmth >= 10) | neutral)
        choices = []
        for sign in (-1, 1):
            axial = sign * along
            end = np.percentile(axial[(depth > .35*radius)], 97)
            mask = (white & (v >= 135) & (axial > end) & (axial < end+2.7*radius)
                    & (across < .78*radius) & (gap < 1.65*radius))
            count, groups, sizes, _ = cv2.connectedComponentsWithStats(mask.astype('uint8'))
            for k in range(1, count):
                if not .19*radius**2 < sizes[k, 4] < 3.1*radius**2:
                    continue
                rows, cols = np.where(groups == k)
                light = v[rows, cols]
                # Use the bright interior, not fringe pixels that can merge with a fin.
                good = light >= np.percentile(light, 55)
                point = np.array([x+cols[good].mean(), y+rows[good].mean()])
                offset = point - (center+[x, y])
                score = (min(1.5, sizes[k, 4]/radius**2)
                         + .6 * (light[good].mean()-135)/120
                         - .45*across[rows[good], cols[good]].mean()/radius)
                if self.previous_time is not None and 0 < timestamp-self.previous_time < .25:
                    if np.dot(offset, self.previous) < 0:
                        continue
                choices.append((score, point, offset))
        choices.sort(key=lambda item: item[0], reverse=True)
        if not choices or choices[0][0] < .56:
            return None
        if len(choices) > 1 and choices[0][0] - choices[1][0] < .2:
            return None
        _, point, self.previous = choices[0]
        self.previous_time = timestamp
        return point


class WhiteHeadSpeed:
    """Fit signed forward travel, not the length of the arc swept by the nose."""
    def __init__(self, max_speed, slack, max_gap):
        self.fit = VelocityEstimator(max_speed_mps=max_speed, reject_overspeed=True, robust_floor_m=.003)
        self.slack = slack
        self.max_gap = max_gap
        self.epoch = 0
        self.reset()

    def reset(self, reason='WARMUP'):
        self.epoch += 1
        self.fit.reset()
        self.last = None
        self.distance = 0.
        self.state = reason
        self.signed_mps = None

    def _gap(self, stamp, identity, reason):
        if self.last is not None:
            age = stamp-self.last[2]
            if self.last[3] == identity and 0 < age <= self.max_gap:
                if self.signed_mps is not None:
                    self.state = 'HELD'
                    return max(0., self.signed_mps)
                self.state = reason
                return None
            self.reset(reason)
        self.state = reason
        return None

    def update(self, head, centre, stamp, identity):
        if centre is None:
            if self.last is not None:self.reset('MISSING')
            self.state='MISSING'
            return None
        if head is None:
            return self._gap(stamp, identity, 'MISSING')
        head = np.asarray(head, dtype=float)
        axis = head-centre
        length = np.linalg.norm(axis)
        if not np.isfinite(length) or length < .01 or not np.isfinite(stamp):
            self.reset('INVALID')
            return None
        axis = axis/length
        if self.last is not None:
            prev, direction, t, key, prev_length = self.last
            dt = stamp-t
            if key != identity or dt > self.max_gap:
                self.reset()
            else:
                # Subtract only the displacement due to rotation of the lever.
                # The forward fit itself still uses the actual nose displacement.
                rotation = .5*(prev_length+length)*(axis-direction)
                if dt <= 0:
                    self.reset('REJECTED')
                    return None
                if (np.dot(axis, direction) <= 0 or abs(length-prev_length)>self.slack+prev_length*.25
                        or np.linalg.norm(head-prev-rotation) > self.slack+dt*self.fit.max_speed_mps):
                    return self._gap(stamp, identity, 'REJECTED')
                # Midpoint axis: a rigid nose rotating on the spot has zero
                # projected displacement, even when its image moves quickly.
                mid = axis+direction
                self.distance += float((head-prev) @ mid / np.linalg.norm(mid))
                # Same smoothing time for a 15 Hz camera and a 30 Hz recording.
                self.fit.blend = -np.expm1(-dt/.09)
        self.last = head.copy(), axis, stamp, identity, length
        velocity = self.fit.update([self.distance, 0.], stamp)
        self.state = 'TRACKING' if velocity is not None else 'WARMUP'
        self.signed_mps = float(velocity[0]) if velocity is not None else None
        return max(0., self.signed_mps) if self.signed_mps is not None else None
