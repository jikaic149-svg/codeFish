"""OpenCV colour confirmation at 5 Hz, with a locked optical-flow reference.

Colour detection seeds the body position and a fixed appearance profile.
Between 200 ms confirmations, forward/backward checked optical flow follows
features on that body. White colour never supplies yaw. The tracked position
is an image measurement, not extrapolation; failed tracking yields no position.
"""

from __future__ import annotations

import math
import cv2
import numpy as np

from fixed_reference_tracker import ReferenceObservation, ReferenceSource, _clip_roi


class WhiteBlackTracker:
    def __init__(self, *, black_value_max=120, white_value_min=105,
                 white_saturation_max=65, search_hold_s=0.6):
        self.black_value_max = int(np.clip(black_value_max, 0, 254))
        self.white_value_min = int(np.clip(white_value_min, 1, 255))
        self.white_saturation_max = int(np.clip(white_saturation_max, 0, 255))
        self.search_hold_s = float(search_hold_s)
        self.detection_interval_s = .20
        self.colour_timeout_s = 1.0
        self._identity_counter = 0
        self.reset_motion()

    def reset_motion(self):
        self._selection_roi = None
        self._bbox = None
        self._time = None
        self._frame_time = None
        self._position = None
        self._track_id = None
        self._body_area = None
        self._last_pair_time = None
        self._cached_observation = None
        self._cached_head = None
        self.head_result = self._empty_head()
        self._update_time = None
        self._last_detection = None
        self._flow_gray = None
        self._flow_points = None
        self._flow_time = None
        self._lock_id = None
        self._lock_position = None
        self._lock_bbox = None
        self._lock_area = None
        self._lock_hist = None
        self._last_colour = None
        self._last_visual = None
        self._candidate_hist = None
        self._feature_mask = None
        self._colour_distance = None
        self._body_bbox = None
        self._flow_quality = 0.
        self._detection_count = 0
        self._flow_count = 0

    def select(self, frame, roi, timestamp):
        """Explicitly select a complete fish; never persist a camera-space ROI."""
        clipped = _clip_roi(roi, frame.shape[1], frame.shape[0])
        if clipped is None:
            raise ValueError('Select the complete white head and black body')
        self.reset_motion()
        self._selection_roi = clipped
        observation = self.update(frame, timestamp)
        self._selection_roi = None
        if not observation.valid:
            self.reset_motion()
            raise ValueError('No unambiguous white-head/black-body pair in selection')
        return observation

    @staticmethod
    def _empty_head():
        return {"source": "UNAVAILABLE", "position": None, "bbox": None,
                "confidence": 0.0, "valid": False, "area": 0.0,
                "selection_mode": "WHITE_HEAD_BLACK_BODY"}

    def set_binary_threshold(self, value):
        self.white_value_min = int(np.clip(value, 1, 255))

    @property
    def val_min(self):
        return self.white_value_min

    def binary_preview(self, frame):
        return self._white_mask(cv2.cvtColor(frame, cv2.COLOR_BGR2HSV))

    def _white_mask(self, hsv):
        neutral = ((hsv[:, :, 1] <= min(40,self.white_saturation_max)) |
                   (hsv[:, :, 0] < 80) | (hsv[:, :, 0] > 135))
        return (((hsv[:, :, 1] <= self.white_saturation_max) & neutral
                 & (hsv[:, :, 2] >= self.white_value_min)).astype(np.uint8) * 255)

    @staticmethod
    def _components(mask, offset):
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        result = []
        for contour in contours:
            area = float(cv2.contourArea(contour))
            if area < 8:
                continue
            m = cv2.moments(contour)
            x, y, w, h = cv2.boundingRect(contour)
            result.append({
                "contour": contour,
                "area": area,
                "center": np.array([m["m10"] / m["m00"], m["m01"] / m["m00"]]),
                "bbox": (x + offset[0], y + offset[1],
                         x + w + offset[0], y + h + offset[1]),
                "extent": area / (w * h),
            })
        return result

    @staticmethod
    def _body_histogram(hsv, body, offset):
        x1,y1,x2,y2=body['bbox'];x=x1-offset[0];y=y1-offset[1]
        patch=hsv[y:y+y2-y1,x:x+x2-x1]
        mask=np.zeros(patch.shape[:2],np.uint8)
        cv2.drawContours(mask,[body['contour']-np.array([[[x,y]]],np.int32)],-1,255,-1)
        hist=cv2.calcHist([patch],[1,2],mask,[8,8],[0,256,0,256])
        return cv2.normalize(hist,hist,alpha=1,norm_type=cv2.NORM_L1)

    def _seed_flow(self, gray, body_bbox):
        x1,y1,x2,y2=map(int,body_bbox)
        mask=self._feature_mask
        if mask is None or mask.shape!=gray.shape:
            mask=np.zeros(gray.shape,np.uint8)
            mask[max(0,y1):min(gray.shape[0],y2),max(0,x1):min(gray.shape[1],x2)]=255
        self._flow_points=cv2.goodFeaturesToTrack(gray,maxCorners=45,qualityLevel=.02,
                                                 minDistance=4,mask=mask,blockSize=5)

    def _track_flow(self, gray, now):
        if (self._flow_gray is None or self._flow_points is None or len(self._flow_points)<4
                or self._flow_time is None or not 0<now-self._flow_time<=.3):
            return False
        try:
            following,status,error=cv2.calcOpticalFlowPyrLK(self._flow_gray,gray,self._flow_points,None,
                winSize=(21,21),maxLevel=3,criteria=(cv2.TERM_CRITERIA_COUNT|cv2.TERM_CRITERIA_EPS,20,.03))
            if following is None:return False
            previous,back_status,_=cv2.calcOpticalFlowPyrLK(gray,self._flow_gray,following,None,
                winSize=(21,21),maxLevel=3)
            if previous is None:return False
            good=(status.ravel()==1)&(back_status.ravel()==1)&(error.ravel()<30)
            good &= np.linalg.norm(previous-self._flow_points,axis=2).ravel()<1.5
            if np.count_nonzero(good)<4:return False
            before=self._flow_points[good].reshape(-1,2);after=following[good].reshape(-1,2)
            matrix,inliers=cv2.estimateAffinePartial2D(before,after,method=cv2.RANSAC,
                ransacReprojThreshold=2.5,maxIters=100,confidence=.98)
            if matrix is None or inliers is None or np.count_nonzero(inliers)<4:return False
            scale=math.hypot(matrix[0,0],matrix[1,0])
            if not .85<=scale<=1.18:return False
            new_position=matrix[:,:2]@self._lock_position+matrix[:,2]
            x1,y1,x2,y2=self._lock_bbox
            if np.linalg.norm(new_position-self._lock_position)>max(25,.6*math.hypot(x2-x1,y2-y1)):
                return False
            if not (0<=new_position[0]<gray.shape[1] and 0<=new_position[1]<gray.shape[0]):return False
            corners=np.array([[x1,y1],[x2,y1],[x2,y2],[x1,y2]])@matrix[:,:2].T+matrix[:,2]
            self._lock_bbox=_clip_roi((*corners.min(axis=0),*corners.max(axis=0)),gray.shape[1],gray.shape[0])
            if self._lock_bbox is None:return False
            self._lock_position=new_position
            self._flow_points=after[inliers.ravel().astype(bool)].reshape(-1,1,2)
            self._flow_quality=float(np.count_nonzero(inliers))/max(1,len(before))
            self._flow_count+=1
            return True
        except cv2.error:
            return False

    def _lock_observation(self, now, valid, mode, reason=None):
        colour_age=now-self._last_colour if math.isfinite(now) and self._last_colour is not None else None
        lost_age=now-self._last_visual if math.isfinite(now) and self._last_visual is not None else None
        metrics={'detector':'OPENCV_COLOR','reference_role':'BLACK_BODY','track_id':self._lock_id,
            'tracking_mode':mode,'locked':self._lock_id is not None,'detection_interval_ms':200,
            'detection_count':self._detection_count,'flow_count':self._flow_count,
            'flow_points':len(self._flow_points) if self._flow_points is not None else 0,
            'flow_quality':self._flow_quality,'colour_age_ms':round(colour_age*1000) if colour_age is not None else None,
            'colour_distance':self._colour_distance,'colour_distance_limit':.65,
            'lost_ms':round(lost_age*1000) if lost_age is not None else None,
            'head_source':'UNAVAILABLE','head_quality':0.}
        if valid:
            point=tuple(self._lock_position)
            metrics.update(body_center=point,body_bbox=self._lock_bbox,bbox=self._lock_bbox,
                fish_bbox=self._lock_bbox,body_area=self._lock_area,tail_marker_position=point,
                tail_marker_bbox=self._lock_bbox,tail_marker_quality=max(.5,self._flow_quality))
        else:point=None
        quality=max(.5,min(.9,self._flow_quality)) if valid else 0.
        return ReferenceObservation(now,ReferenceSource.RIGID_BODY if valid else ReferenceSource.INVALID,
            point,quality,quality,False,False,valid,[] if valid else [reason or 'LOCK_WAIT'],metrics,self._lock_bbox)

    def update(self, frame, timestamp):
        now=float(timestamp)
        if now==self._update_time and self._cached_observation is not None:
            self.head_result=self._cached_head
            return self._cached_observation
        if not math.isfinite(now) or (self._update_time is not None and now<self._update_time):
            return self._lock_observation(now,False,'STALE','STALE_FRAME')
        if frame is None or frame.ndim!=3 or frame.shape[2]!=3:
            return self._lock_observation(now,False,'LOST','INVALID_FRAME')
        gray=cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
        if self._flow_gray is not None and self._flow_gray.shape!=gray.shape:self.reset_motion()
        self._update_time=now;self.head_result=self._empty_head()
        flowing=self._lock_id is not None and self._track_flow(gray,now)
        if flowing:
            self._last_visual=now
            # Centre the slow detector around the measured flow displacement.
            self._position=self._lock_position.copy();self._bbox=self._lock_bbox;self._time=now
        if (self._lock_id is not None and self._last_visual is not None
                and now-self._last_visual>self.search_hold_s):
            self._lock_id=None;self._lock_hist=None;self._lock_position=None
            self._lock_bbox=None;self._flow_points=None;self._bbox=None;self._time=None
        observation=None
        if self._last_detection is None or now-self._last_detection>=self.detection_interval_s:
            self._last_detection=now;self._detection_count+=1
            candidate=self._detect(frame,now)
            if candidate.valid and (self._lock_id is not None or candidate.metrics.get('tracking_mode')=='COLOR_PAIR'):
                if self._lock_id is None:
                    self._identity_counter+=1;self._lock_id=self._identity_counter
                    self._lock_hist=self._candidate_hist.copy()
                self._lock_position=np.asarray(candidate.position,dtype=np.float64)
                self._lock_bbox=candidate.metrics['fish_bbox'];self._lock_area=candidate.metrics['body_area']
                self._last_visual=now
                if candidate.metrics.get('tracking_mode')=='COLOR_PAIR':self._last_colour=now
                self._seed_flow(gray,candidate.metrics['body_bbox'])
                self._flow_quality=candidate.quality
                observation=candidate
                observation.metrics.update(self._lock_observation(now,True,'COLOR_CONFIRMED').metrics)
                # A real colour confirmation retains the real head observation.
                observation.metrics.update(head_source=self.head_result['source'],head_quality=self.head_result['confidence'])
        if observation is None:
            valid=bool(flowing and self._lock_id is not None and self._last_colour is not None
                       and now-self._last_colour<=self.colour_timeout_s)
            if not valid:self._flow_points=None
            observation=self._lock_observation(now,valid,'LOCKED_FLOW' if valid else 'LOCK_WAIT')
        self._flow_gray=gray;self._flow_time=now
        self._cached_observation=observation;self._cached_head=self.head_result
        return observation

    def _detect(self, frame, timestamp):
        now = float(timestamp)
        # The UI loop can read one camera frame several times. Reuse the same
        # timestamped measurement; speed/PI consumers deduplicate its timestamp.
        if now == self._frame_time and self._cached_observation is not None:
            self.head_result = self._cached_head
            return self._cached_observation
        self.head_result = self._empty_head()

        def invalid(reason, roi=None):
            observation = ReferenceObservation(now, ReferenceSource.INVALID, None, 0.0,
                                        0.0, False, False, False, [reason],
                                        {"reference_role": "BLACK_BODY",
                                         "detector": "OPENCV_COLOR"}, roi)
            if now == self._frame_time:
                self._cached_observation, self._cached_head = observation, self.head_result
            return observation

        if frame is None or frame.ndim != 3 or frame.shape[2] != 3:
            return invalid("INVALID_FRAME")
        if not math.isfinite(now) or (self._frame_time is not None and now <= self._frame_time):
            return invalid("STALE_FRAME")
        height, width = frame.shape[:2]
        self._frame_time = now
        roi = self._selection_roi
        recent = self._time is not None and now - self._time <= self.search_hold_s
        if roi is None and self._bbox is not None and recent:
            x1, y1, x2, y2 = self._bbox
            margin = max(32, (x2 - x1), (y2 - y1))
            roi = _clip_roi((x1 - margin, y1 - margin, x2 + margin, y2 + margin), width, height)
        if roi is None:
            roi = (0, 0, width, height)
        x1, y1, x2, y2 = roi
        hsv = cv2.cvtColor(frame[y1:y2, x1:x2], cv2.COLOR_BGR2HSV)
        # A broad blue-water region excludes the pool rim, cables and UI border
        # in the supplied recording. Other backgrounds retain geometric checks.
        water = cv2.inRange(hsv, np.array([80,45,55]), np.array([130,255,255]))
        water = cv2.morphologyEx(water, cv2.MORPH_CLOSE, np.ones((21,21),np.uint8))
        water_contours, _ = cv2.findContours(water, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        pool = max(water_contours, key=cv2.contourArea) if water_contours else None
        if pool is not None and cv2.contourArea(pool) < hsv.shape[0]*hsv.shape[1]*.15:
            pool = None
        if pool is not None:
            pool = cv2.convexHull(pool)
        kernel = np.ones((3, 3), np.uint8)
        # The real pool is blue, including dark tiles beside the fish. Keep
        # neutral/warm black body pixels rather than merging into blue shadows.
        neutral = (hsv[:, :, 1] <= 55) | (hsv[:, :, 0] < 75) | (hsv[:, :, 0] > 135)
        black = ((hsv[:, :, 2] <= self.black_value_max) & neutral).astype(np.uint8) * 255
        white = self._white_mask(hsv)
        black = cv2.morphologyEx(black, cv2.MORPH_OPEN, kernel)
        black = cv2.morphologyEx(black, cv2.MORPH_CLOSE, kernel)
        white = cv2.morphologyEx(white, cv2.MORPH_OPEN, kernel)
        bodies = self._components(black, (x1, y1))
        heads = self._components(white, (x1, y1))
        pairs = []
        body_candidates = []
        for body in bodies:
            if pool is not None and cv2.pointPolygonTest(pool,tuple(body['center']),False)<0:
                continue
            bx1, by1, bx2, by2 = body["bbox"]
            if body["area"] < 30 or body["area"] > width * height * 0.25:
                continue
            if body["extent"] < 0.15 or min(bx1-x1, by1-y1, x2-bx2, y2-by2) <= 1:
                continue
            if recent and self._body_area and not 0.30 <= body["area"] / self._body_area <= 3.0:
                continue
            body['histogram']=self._body_histogram(hsv,body,(x1,y1))
            body['colour_distance']=0.
            if self._lock_hist is not None:
                body['colour_distance']=cv2.compareHist(self._lock_hist,body['histogram'],cv2.HISTCMP_BHATTACHARYYA)
                if body['colour_distance']>.65:
                    continue
                if self._lock_position is not None and np.linalg.norm(body['center']+(x1,y1)-self._lock_position)>max(24,(bx2-bx1+by2-by1)*.35):
                    continue
            points = body["contour"].reshape(-1, 2).astype(np.float64)
            eigenvalues, eigenvectors = np.linalg.eigh(np.cov(points, rowvar=False))
            if eigenvalues[0] <= 0 or eigenvalues[1] / eigenvalues[0] < 1.30:
                continue
            axis = eigenvectors[:, 1]
            projection = (points - body["center"]) @ axis
            length = float(np.ptp(projection))
            short_axis = np.array([-axis[1], axis[0]])
            thickness = float(np.ptp((points - body["center"]) @ short_axis))
            center = body['center'] + (x1,y1)
            if recent and self._last_pair_time is not None and now-self._last_pair_time<=self.search_hold_s:
                travel = float(np.linalg.norm(center-self._position))
                if (travel<=max(18,length*.4) and self._body_area
                        and .55<=body['area']/self._body_area<=1.8):
                    body_candidates.append((travel,body))
            for head in heads:
                hx1, hy1, hx2, hy2 = head["bbox"]
                if not 0.015 <= head["area"] / body["area"] <= 1.3:
                    continue
                if head["extent"] < 0.20 or max(hx2-hx1, hy2-hy1) > 6 * min(hx2-hx1, hy2-hy1):
                    continue
                if min(hx1-x1, hy1-y1, x2-hx2, y2-hy2) <= 1:
                    continue
                delta = head["center"] - body["center"]
                distance = float(np.linalg.norm(delta))
                along = abs(float(delta @ axis))
                across = abs(float(delta @ short_axis))
                if not max(2, length * 0.05) <= along <= length * 1.1:
                    continue
                if across > max(8, thickness * 1.2) or distance < 4:
                    continue
                gap = -cv2.pointPolygonTest(body["contour"], tuple(head["center"]), True)
                head_radius = math.sqrt(head["area"] / math.pi)
                if gap > head_radius + max(8, thickness * 0.40):
                    continue
                alignment = along / distance
                if alignment < 0.45:
                    continue
                quality = min(0.98, 0.55 + 0.25 * alignment + 0.15 * min(1, body["extent"]))
                score = quality
                center = body["center"] + (x1, y1)
                if self._position is not None and recent:
                    travel = float(np.linalg.norm(center - self._position))
                    if travel > max(32, length * 1.2):
                        continue
                    score += 0.30 * math.exp(-travel / max(length, 1))
                # White panels/fins belonging to one body are one target.
                # Their direction does not determine yaw (position does).
                end = 1 if float(delta @ axis) >= 0 else -1
                previous = next((i for i, pair in enumerate(pairs)
                                 if pair[2] is body), None)
                candidate = (score, quality, body, head, end)
                if previous is None:
                    pairs.append(candidate)
                elif (score, head["area"]) > (pairs[previous][0], pairs[previous][3]["area"]):
                    pairs[previous] = candidate
        if not pairs:
            # Only a recently colour-confirmed fish may bridge a white-head miss.
            # Position still comes from a real black contour; never extrapolate.
            body_candidates.sort(key=lambda item:item[0])
            if body_candidates and (len(body_candidates)==1 or body_candidates[1][0]>body_candidates[0][0]+12):
                pairs.append((.55,.55,body_candidates[0][1],None,0))
            else:
                return invalid("WHITE_HEAD_BLACK_BODY_NOT_FOUND", roi)
        pairs.sort(key=lambda item: item[0], reverse=True)
        if len(pairs) > 1 and pairs[1][0] >= pairs[0][0] * 0.98:
            return invalid("WHITE_HEAD_BLACK_BODY_AMBIGUOUS", roi)
        _, quality, body, head, _ = pairs[0]
        self._candidate_hist=body['histogram']
        self._colour_distance=body['colour_distance']
        self._feature_mask=np.zeros((height,width),np.uint8)
        cv2.drawContours(self._feature_mask,[body['contour']+np.array([[[x1,y1]]],np.int32)],-1,255,-1)
        self._feature_mask=cv2.dilate(self._feature_mask,np.ones((3,3),np.uint8))
        position = body["center"] + (x1, y1)
        head_position = head["center"] + (x1, y1) if head else None
        head_bbox = head['bbox'] if head else None
        bbox = body['bbox'] if head is None else (
                min(body["bbox"][0], head["bbox"][0]), min(body["bbox"][1], head["bbox"][1]),
                max(body["bbox"][2], head["bbox"][2]), max(body["bbox"][3], head["bbox"][3]))
        if not recent:
            self._track_id = (self._track_id or 0) + 1
        self._position, self._time, self._bbox = position, now, bbox
        self._body_area = body["area"]
        if head:
            self._last_pair_time = now
            self.head_result = {
                "source": "WHITE", "position": tuple(head_position), "bbox": head["bbox"],
                "confidence": quality, "valid": True, "area": head["area"],
                "selection_mode": "WHITE_HEAD_BLACK_BODY", "dynamic_value_min": self.white_value_min,
                "tail_distance_px": float(np.linalg.norm(head_position - position)),
            }
        observation = ReferenceObservation(
            now, ReferenceSource.RIGID_BODY, tuple(position), quality, quality,
            False, False, True, [], {
                "reference_role": "BLACK_BODY", "bbox": body["bbox"],
                "detector": "OPENCV_COLOR", "track_id": self._track_id,
                "body_center": tuple(position), "head_quality": quality if head else 0,
                "head_source": "WHITE" if head else "UNAVAILABLE",
                "tracking_mode": "COLOR_PAIR" if head else "BODY_CONTINUITY",
                "body_bbox": body["bbox"], "body_area": body["area"],
                "head_position": tuple(head_position) if head is not None else None, "head_bbox": head_bbox,
                "fish_bbox": bbox,
                # Existing consumer fields mean the rear yaw reference. Their
                # provenance is explicit: this is the body, not an orange tail.
                "tail_marker_position": tuple(position), "tail_marker_bbox": body["bbox"],
                "tail_marker_area": body["area"], "tail_marker_quality": quality,
            }, roi)
        self._cached_observation, self._cached_head = observation, self.head_result
        return observation
