#!/usr/bin/env python3
"""Pure, conservative map-head/YOLO association (no ROS or native simulator API).

Projection is an identity gate, not visibility or lamp-state ground truth. The
transform maps the explicit geometry frame to camera forward/left/up axes.
Already-corrected localization must never receive an additional Ego yaw offset.
"""
import json
import math
from dataclasses import dataclass, asdict

MISSION_GROUP_IDS = frozenset((201351, 201365, 201290, 201326))
SELECTOR_CONTRACT_VERSION = 2


@dataclass(frozen=True)
class MapSelectorConfig:
    max_view_angle_degrees: float = 45.
    min_depth_m: float = 3.
    max_depth_m: float = 180.
    min_height_px: float = 3.
    base_margin_px: float = 12.
    pitch_tolerance_degrees: float = 1.
    max_padding_px: float = 90.
    max_vertical_padding_px: float = 200.
    max_position_sigma_m: float = 1.5
    max_vertical_sigma_m: float = 1.5
    max_yaw_sigma_degrees: float = 3.
    min_width_ratio: float = .45
    max_width_ratio: float = 2.5
    min_height_ratio: float = .35
    max_height_ratio: float = 3.5
    min_aspect_ratio_factor: float = .4
    max_aspect_ratio_factor: float = 2.5
    ambiguity_margin: float = .15
    head_continuity_bonus: float = .15

    def to_dict(self):
        return asdict(self)

    def projection_kwargs(self):
        return {key: value for key, value in asdict(self).items() if key in (
            "max_view_angle_degrees", "min_depth_m", "max_depth_m", "min_height_px", "base_margin_px",
            "pitch_tolerance_degrees", "max_padding_px", "max_vertical_padding_px")}


DEFAULT_SELECTOR_CONFIG = MapSelectorConfig()


def selector_config(overrides=None):
    """Single validated contract shared by helper, packet consumer and offline replay."""
    values = DEFAULT_SELECTOR_CONFIG.to_dict()
    if overrides is not None:
        if not isinstance(overrides, dict) or set(overrides)-set(values):
            raise ValueError("unknown/non-object map selector configuration")
        values.update(overrides)
    if any(isinstance(value, bool) for value in values.values()):
        raise ValueError("map selector configuration requires numeric values, not booleans")
    values = {key: finite(value) for key, value in values.items()}
    nonnegative = ("base_margin_px", "pitch_tolerance_degrees", "ambiguity_margin", "head_continuity_bonus")
    if any(value < 0 if key in nonnegative else value <= 0 for key, value in values.items()):
        raise ValueError("invalid nonpositive selector bound")
    if values["max_view_angle_degrees"] > 60 or values["pitch_tolerance_degrees"] > 5:
        raise ValueError("invalid front-view/pitch tolerance")
    for low, high in (("min_depth_m", "max_depth_m"), ("min_width_ratio", "max_width_ratio"),
                      ("min_height_ratio", "max_height_ratio"), ("min_aspect_ratio_factor", "max_aspect_ratio_factor")):
        if values[low] >= values[high]:
            raise ValueError("selector lower bound must be below upper bound")
    return MapSelectorConfig(**values)


class ProjectionUncertaintyError(ValueError):
    def __init__(self, padding_xy_px, bounds_xy_px):
        super().__init__("projection_uncertainty_exceeds_bound")
        self.padding_xy_px = tuple(padding_xy_px)
        self.bounds_xy_px = tuple(bounds_xy_px)


def finite(value):
    value = float(value)
    if not math.isfinite(value):
        raise ValueError("non-finite geometry")
    return value


def quaternion_matrix(quaternion):
    x, y, z, w = [finite(v) for v in quaternion]
    norm = math.sqrt(x*x + y*y + z*z + w*w)
    if norm < 1e-9:
        raise ValueError("invalid zero quaternion")
    x, y, z, w = [v / norm for v in (x, y, z, w)]
    return ((1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)),
            (2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)),
            (2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)))


def rotate(matrix, point):
    return tuple(sum(matrix[i][j] * point[j] for j in range(3)) for i in range(3))


@dataclass(frozen=True)
class RigidTransform:
    rotation: tuple
    translation: tuple

    @classmethod
    def from_quaternion(cls, translation, quaternion):
        if len(translation) != 3 or len(quaternion) != 4:
            raise ValueError("transform requires xyz and xyzw")
        return cls(quaternion_matrix(quaternion), tuple(finite(v) for v in translation))

    def point(self, point):
        rotated = rotate(self.rotation, point)
        return tuple(rotated[i] + self.translation[i] for i in range(3))

    def inverse_origin(self):
        return tuple(-sum(self.rotation[j][i] * self.translation[j] for j in range(3))
                     for i in range(3))


@dataclass(frozen=True)
class CameraModel:
    width: int
    height: int
    fx: float
    fy: float
    cx: float
    cy: float
    frame: str
    position: tuple
    quaternion: tuple


def load_camera(path):
    with open(path) as stream:
        document = json.load(stream)
    cameras = [c for c in document["cameraList"] if c["m_SensorUniqueID"] == 1]
    if len(cameras) != 1:
        raise ValueError("exactly one explicit front camera ID 1 is required")
    camera = cameras[0]
    cc = camera["cc"]
    width, height = int(cc["cameraResWidth"]), int(cc["cameraResHeight"])
    fov = finite(cc["cameraFOV"])
    if width <= 0 or height <= 0 or not 0 < fov < 180:
        raise ValueError("invalid camera resolution/FOV")
    if any(abs(finite(v)) > 1e-9 for v in cc.get("lensDistortion", [])):
        raise ValueError("nonzero lens distortion requires a calibrated projector")
    frame = cc.get("rosConfig", {}).get("frameID", "")
    if not frame:
        frame = cc.get("ros2NativeConfig", {}).get("frameID", "")
    if not frame:
        raise ValueError("camera frame is required")
    roll, pitch, yaw = [math.radians(finite(camera["rot"][k])) for k in ("roll", "pitch", "yaw")]
    cr, sr, cp, sp, cy, sy = (math.cos(roll/2), math.sin(roll/2), math.cos(pitch/2),
                             math.sin(pitch/2), math.cos(yaw/2), math.sin(yaw/2))
    quaternion = (sr*cp*cy-cr*sp*sy, cr*sp*cy+sr*cp*sy,
                  cr*cp*sy-sr*sp*cy, cr*cp*cy+sr*sp*sy)
    focal = width / (2 * math.tan(math.radians(fov / 2)))
    return CameraModel(width, height, focal, focal, width/2., height/2., frame,
                       tuple(finite(camera["pos"][k]) for k in "xyz"), quaternion)


def load_geometry(path):
    with open(path) as stream:
        document = json.load(stream)
    if document.get("schema_version") != 1 or not document.get("geometry_frame"):
        raise ValueError("unsupported/missing map geometry schema/frame")
    seen = set()
    for head in document["heads"]:
        if head["id"] in seen or head.get("type") != "car":
            raise ValueError("duplicate or non-vehicle head")
        seen.add(head["id"])
        if len(head["point"]) != 3:
            raise ValueError("head point must be xyz")
        for value in head["point"] + [head["heading"], head["width"], head["height"]]:
            finite(value)
        if head["width"] <= 0 or head["height"] <= 0:
            raise ValueError("head size must be measured and positive")
        if head.get("group_id") is not None and int(head["group_id"]) <= 0:
            raise ValueError("invalid group ID")
        if head.get("group_id") is not None and int(head["group_id"]) not in MISSION_GROUP_IDS:
            raise ValueError("geometry may not promote a non-mission group")
    return document


def uncertainty_padding(camera, depth, position_sigma_m, yaw_sigma_rad,
                        base_margin_px=DEFAULT_SELECTOR_CONFIG.base_margin_px,
                        pitch_tolerance_degrees=DEFAULT_SELECTOR_CONFIG.pitch_tolerance_degrees,
                        max_padding_px=DEFAULT_SELECTOR_CONFIG.max_padding_px,
                        vertical_sigma_m=None, camera_up_m=0.,
                        max_vertical_padding_px=DEFAULT_SELECTOR_CONFIG.max_vertical_padding_px):
    """Bounded 2-sigma search only; a large covariance rejects instead of widening forever."""
    position_sigma_m, yaw_sigma_rad = finite(position_sigma_m), finite(yaw_sigma_rad)
    vertical_sigma_m = position_sigma_m if vertical_sigma_m is None else finite(vertical_sigma_m)
    camera_up_m = finite(camera_up_m)
    base_margin_px, pitch_tolerance_degrees, max_padding_px = [finite(v) for v in
        (base_margin_px, pitch_tolerance_degrees, max_padding_px)]
    max_vertical_padding_px = max_padding_px if max_vertical_padding_px is None else finite(max_vertical_padding_px)
    if (base_margin_px < 0 or not 0 <= pitch_tolerance_degrees <= 5
            or max_padding_px <= 0 or max_vertical_padding_px <= 0):
        raise ValueError("invalid projection padding limits")
    if position_sigma_m < 0 or vertical_sigma_m < 0 or yaw_sigma_rad < 0 or depth <= 0:
        raise ValueError("invalid projection uncertainty")
    # Floors prevent an all-zero/unreported covariance from claiming perfect pose.
    position_sigma_m, yaw_sigma_rad = max(.20, position_sigma_m), max(math.radians(.35), yaw_sigma_rad)
    vertical_sigma_m = max(.20, vertical_sigma_m)
    # XY and GNSS Z are different measurements. Z never inflates horizontal search.
    horizontal = base_margin_px + camera.fx * (2*position_sigma_m/depth + 2*yaw_sigma_rad)
    depth_coupling = 2*position_sigma_m*abs(camera_up_m)/(depth*depth)
    vertical = base_margin_px + camera.fy * (2*vertical_sigma_m/depth + depth_coupling
                                             + math.tan(math.radians(pitch_tolerance_degrees)))
    if horizontal > max_padding_px or vertical > max_vertical_padding_px:
        raise ProjectionUncertaintyError((horizontal, vertical), (max_padding_px, max_vertical_padding_px))
    return horizontal, vertical


def project_heads(document, camera, transform, group_id, position_sigma_m=.2,
                  yaw_sigma_rad=math.radians(.35),
                  max_view_angle_degrees=DEFAULT_SELECTOR_CONFIG.max_view_angle_degrees,
                  min_depth_m=DEFAULT_SELECTOR_CONFIG.min_depth_m,
                  max_depth_m=DEFAULT_SELECTOR_CONFIG.max_depth_m,
                  min_height_px=DEFAULT_SELECTOR_CONFIG.min_height_px,
                  base_margin_px=DEFAULT_SELECTOR_CONFIG.base_margin_px,
                  pitch_tolerance_degrees=DEFAULT_SELECTOR_CONFIG.pitch_tolerance_degrees,
                  max_padding_px=DEFAULT_SELECTOR_CONFIG.max_padding_px,
                  vertical_sigma_m=None,
                  max_vertical_padding_px=DEFAULT_SELECTOR_CONFIG.max_vertical_padding_px):
    """Project all vehicle heads as identity competitors; only route heads are eligible."""
    if not 0 < finite(max_view_angle_degrees) <= 60:
        raise ValueError("front-facing angle must be within (0,60] degrees")
    if not 0 < min_depth_m < max_depth_m or min_height_px <= 0:
        raise ValueError("invalid depth/size limits")
    camera_origin = transform.inverse_origin()
    rois, rejected = [], []
    for head in document["heads"]:
        reason = None
        rejection_details = {}
        center = head["point"]
        to_camera = [camera_origin[i] - center[i] for i in range(3)]
        horizontal_range = math.hypot(to_camera[0], to_camera[1])
        heading = math.radians(head["heading"])
        facing_cos = ((math.cos(heading)*to_camera[0] + math.sin(heading)*to_camera[1]) /
                      horizontal_range) if horizontal_range > 1e-6 else -1.
        angle = math.degrees(math.acos(max(-1., min(1., facing_cos))))
        camera_center = transform.point(center)
        depth = camera_center[0]
        if not min_depth_m <= depth <= max_depth_m:
            reason = "depth_out_of_bounds"
        elif angle > max_view_angle_degrees:
            reason = "not_front_facing"
        width_axis = (-math.sin(heading), math.cos(heading), 0.)
        corners = []
        if reason is None:
            for sx, sz in ((-1, -1), (1, -1), (1, 1), (-1, 1)):
                point = [center[i] + sx*head["width"]/2*width_axis[i] for i in range(3)]
                point[2] += sz*head["height"]/2
                forward, left, up = transform.point(point)
                if forward <= 0:
                    reason = "crossing_camera_plane"
                    break
                corners.append((camera.cx-camera.fx*left/forward, camera.cy-camera.fy*up/forward))
        if reason is None:
            bbox = [min(p[0] for p in corners), min(p[1] for p in corners),
                    max(p[0] for p in corners), max(p[1] for p in corners)]
            if bbox[0] < 0 or bbox[1] < 0 or bbox[2] > camera.width or bbox[3] > camera.height:
                reason = "housing_not_fully_in_image"
            elif bbox[3]-bbox[1] < min_height_px:
                reason = "housing_too_small"
        if reason is None:
            try:
                px, py = uncertainty_padding(camera, depth, position_sigma_m, yaw_sigma_rad,
                    base_margin_px, pitch_tolerance_degrees, max_padding_px,
                    vertical_sigma_m=vertical_sigma_m, camera_up_m=camera_center[2],
                    max_vertical_padding_px=max_vertical_padding_px)
            except ValueError as error:
                reason = str(error)
                if isinstance(error, ProjectionUncertaintyError):
                    rejection_details.update(required_padding_xy_px=list(error.padding_xy_px),
                                             max_padding_xy_px=list(error.bounds_xy_px))
        if reason is not None:
            rejected.append({"head_id": head["id"], "reason": reason, "view_angle_degrees": angle,
                             "depth_m": depth, **rejection_details})
            continue
        rois.append({"head_id": head["id"], "group_id": head.get("group_id"),
                     "eligible": head.get("group_id") == int(group_id) and int(group_id) > 0,
                     "bbox_xyxy": bbox, "gate_xyxy": [bbox[0]-px, bbox[1]-py, bbox[2]+px, bbox[3]+py],
                     "depth_m": depth, "view_angle_degrees": angle,
                     "padding_xy_px": [px, py], "projected_height_px": bbox[3]-bbox[1]})
    return rois, rejected


def _pair_cost(box, roi, config=DEFAULT_SELECTOR_CONFIG):
    x1, y1, x2, y2, confidence = box[4]
    if not all(math.isfinite(float(v)) for v in (x1, y1, x2, y2, confidence)):
        return None
    width, height = x2-x1, y2-y1
    rx1, ry1, rx2, ry2 = roi["bbox_xyxy"]
    rw, rh = rx2-rx1, ry2-ry1
    if width <= 0 or height <= 0 or rw <= 0 or rh <= 0:
        return None
    if (not config.min_width_ratio <= width/rw <= config.max_width_ratio
            or not config.min_height_ratio <= height/rh <= config.max_height_ratio):
        return None
    # Mission heads are horizontal housings. Independent W/H ratios alone can
    # admit a tall pedestrian-shaped detection inside the larger GNSS-Z band.
    aspect_factor = (width/height)/(rw/rh)
    if not config.min_aspect_ratio_factor <= aspect_factor <= config.max_aspect_ratio_factor:
        return None
    cx, cy = (x1+x2)/2, (y1+y2)/2
    gx1, gy1, gx2, gy2 = roi["gate_xyxy"]
    if not gx1 <= cx <= gx2 or not gy1 <= cy <= gy2:
        return None
    px, py = roi["padding_xy_px"]
    dx = (cx-(rx1+rx2)/2) / (rw/2+px)
    dy = (cy-(ry1+ry2)/2) / (rh/2+py)
    return dx*dx + dy*dy + .12*abs(math.log(width/rw)) + .06*abs(math.log(height/rh))


def associate_boxes(boxes, rois, group_id, previous_head_id=None, ambiguity_margin=None, config=None):
    """Never fall back to an unrelated box. Other mapped car heads veto ambiguity."""
    config = selector_config(config.to_dict() if isinstance(config, MapSelectorConfig) else config)
    ambiguity_margin = config.ambiguity_margin if ambiguity_margin is None else finite(ambiguity_margin)
    candidates, rejected = [], []
    for index, box in enumerate(boxes):
        pairs = [(cost, roi) for roi in rois for cost in [_pair_cost(box, roi, config)] if cost is not None]
        eligible = [(cost, roi) for cost, roi in pairs
                    if roi.get("eligible") and roi.get("group_id") == int(group_id) and int(group_id) > 0]
        if not eligible:
            rejected.append({"box_index": index, "reason": "no_route_head_match"})
            continue
        cost, roi = min(eligible, key=lambda pair: pair[0])
        competitors = [(other_cost, other) for other_cost, other in pairs
                       if other.get("group_id") != int(group_id)]
        if competitors and min(pair[0] for pair in competitors) <= cost + ambiguity_margin:
            rejected.append({"box_index": index, "reason": "other_head_identity_ambiguous"})
            continue
        x1, y1, x2, y2, confidence = box[4]
        # Prefer readable, larger detected housing; continuity is only a small tie-break.
        score = math.log(max(1., (x2-x1)*(y2-y1))) + .5*confidence - 2*cost
        if roi["head_id"] == previous_head_id:
            score += config.head_continuity_bonus
        candidates.append((score, index, roi, cost))
    if not candidates:
        return None, {"status": "missing", "reason": "no_unambiguous_route_detection", "rejected": rejected}
    score, index, roi, cost = max(candidates, key=lambda row: row[0])
    return boxes[index], {"status": "matched", "reason": "map_head_association", "box_index": index,
                          "head_id": roi["head_id"], "score": score, "geometry_cost": cost,
                          "roi": roi, "rejected": rejected}


def select_packet(packets, source_stamp_ns, group_id, width, height, now_ns,
                  max_image_delta_sec=.05, max_packet_age_sec=.5, camera_frame=None):
    """Choose a fresh camera-frame packet; future/old route IDs never leak through."""
    valid = []
    for packet in packets:
        try:
            delta = abs(int(packet["source_stamp_ns"])-int(source_stamp_ns))/1e9
            age = (int(now_ns)-int(packet["generated_stamp_ns"]))/1e9
            if (packet.get("schema_version") == 1 and int(group_id) > 0
                    and packet["group_id"] == int(group_id)
                    and packet["image_width"] == width and packet["image_height"] == height
                    and (camera_frame is None or packet.get("camera_frame", "").strip("/") == camera_frame.strip("/"))
                    and int(source_stamp_ns) > 0 and delta <= max_image_delta_sec
                    and 0 <= age <= max_packet_age_sec):
                valid.append((delta, packet))
        except (KeyError, TypeError, ValueError, OverflowError):
            continue
    return min(valid, key=lambda pair: pair[0])[1] if valid else None


def covariance_sigmas(covariance, max_position_sigma_m=DEFAULT_SELECTOR_CONFIG.max_position_sigma_m,
                      max_yaw_sigma_degrees=DEFAULT_SELECTOR_CONFIG.max_yaw_sigma_degrees,
                      max_vertical_sigma_m=DEFAULT_SELECTOR_CONFIG.max_vertical_sigma_m):
    """Return (XY principal-axis sigma, independent GNSS-Z sigma, yaw sigma).

    Production inserts GNSS z variance into covariance[14]. It is not an XY
    EKF variance and must not widen the horizontal association gate.
    """
    if len(covariance) != 36:
        raise ValueError("invalid_pose_covariance")
    values = [finite(covariance[i]) for i in (0, 7, 14, 35)]
    if any(v < 0 for v in values):
        raise ValueError("invalid_pose_covariance")
    xx, yy, zz, yaw_variance = values
    xy, yx = finite(covariance[1]), finite(covariance[6])
    if abs(xy-yx) > 1e-6:
        raise ValueError("asymmetric_XY_pose_covariance")
    cross = (xy+yx)/2
    radius = math.sqrt((xx-yy)**2+4*cross*cross)
    if (xx+yy-radius)/2 < -1e-8:
        raise ValueError("non_positive_semidefinite_XY_pose_covariance")
    position, vertical, yaw = math.sqrt(max(0., (xx+yy+radius)/2)), math.sqrt(zz), math.sqrt(yaw_variance)
    if position > max_position_sigma_m or yaw > math.radians(max_yaw_sigma_degrees):
        raise ValueError("localization_uncertainty_exceeds_bound")
    if vertical > max_vertical_sigma_m:
        raise ValueError("localization_vertical_uncertainty_exceeds_bound")
    return position, vertical, yaw


def pose_tf_consistency(pose_position, pose_quaternion, tf_position, tf_quaternion,
                        time_delta_sec, speed_mps=0., yaw_rate_radps=0.):
    """Reject a different/stale TF provider; permit bounded nearest-odom motion."""
    position = [finite(v) for v in pose_position]
    tf_position = [finite(v) for v in tf_position]
    q, tq = [tuple(finite(v) for v in values) for values in (pose_quaternion, tf_quaternion)]
    norms = [math.sqrt(sum(v*v for v in values)) for values in (q, tq)]
    if min(norms) < 1e-9 or not .99 <= norms[0] <= 1.01:
        raise ValueError("invalid_localization_quaternion")
    distance = math.sqrt(sum((position[i]-tf_position[i])**2 for i in range(3)))
    dot = min(1., abs(sum(q[i]*tq[i]/(norms[0]*norms[1]) for i in range(4))))
    angle = 2*math.acos(dot)
    dt, speed, rate = abs(finite(time_delta_sec)), abs(finite(speed_mps)), abs(finite(yaw_rate_radps))
    if distance > .5 + speed*dt or angle > math.radians(3.) + rate*dt:
        raise ValueError("localization_tf_pose_inconsistent")
    return distance, math.degrees(angle)
