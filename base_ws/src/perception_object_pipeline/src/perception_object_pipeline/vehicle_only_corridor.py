"""Route-verified vehicle-only planning corridor.

The tracker remains unfiltered.  Only the planner-facing PredictedObjects
message is filtered, and only while a fresh ego pose matches this portion of
the competition route.  Unknown map/pose evidence always disables filtering.
"""

import math

import numpy as np


class VehicleOnlyCorridor:
    def __init__(self, network, route_doc, start_lanelet_id=200258,
                 start_s_m=30.0, end_lanelet_id=200088,
                 max_lateral_distance_m=12.0,
                 max_heading_error_rad=math.pi / 2.0):
        if route_doc.get('map', {}).get('sha256') != network.map_sha256:
            raise ValueError('vehicle-only route map SHA differs from loaded prediction map')
        route_a = route_doc.get('route_a', {}).get('waypoint_path_lanelet_ids', [])
        route_b = route_doc.get('route_b', {}).get('waypoint_path_lanelet_ids', [])
        start_id, end_id = int(start_lanelet_id), int(end_lanelet_id)
        if start_id not in route_a or end_id not in route_a:
            raise ValueError('vehicle-only route anchor lanelet is missing')
        start_index, end_index = route_a.index(start_id), route_a.index(end_id)
        if end_index <= start_index:
            raise ValueError('vehicle-only route anchors are not in driving order')
        lanelet_ids = tuple(int(value) for value in route_a[start_index:end_index + 1])
        if start_id not in route_b or end_id not in route_b:
            raise ValueError('vehicle-only corridor is not shared by route B')
        b_ids = tuple(int(value) for value in route_b[route_b.index(start_id):route_b.index(end_id) + 1])
        if b_ids != lanelet_ids:
            raise ValueError('vehicle-only corridor differs between routes A and B')
        if not 0.0 <= float(start_s_m) < network.records[start_id].length:
            raise ValueError('vehicle-only start_s_m lies outside its lanelet')
        if not 0.0 < float(max_lateral_distance_m) <= 15.0:
            raise ValueError('vehicle-only lateral gate is invalid')
        if not 0.0 < float(max_heading_error_rad) <= math.pi:
            raise ValueError('vehicle-only heading gate is invalid')

        points = []
        gap_indices = []
        for lanelet_id in lanelet_ids:
            record = network.records.get(lanelet_id)
            if record is None or len(record.centerline) < 2:
                raise ValueError('vehicle-only route lanelet is absent from the loaded map')
            centerline = list(record.centerline)
            if points:
                gap = math.hypot(points[-1][0] - centerline[0][0],
                                 points[-1][1] - centerline[0][1])
                if gap > 15.0:
                    raise ValueError('vehicle-only route has a disconnected lanelet gap')
                if gap < 0.01:
                    centerline = centerline[1:]
                else:
                    gap_indices.append(len(points) - 1)
            points.extend(centerline)
        xy = np.asarray([point[:2] for point in points], dtype=np.float64)
        segment = xy[1:] - xy[:-1]
        length = np.linalg.norm(segment, axis=1)
        valid = length > 1.e-6
        self.origins = xy[:-1][valid]
        self.segment = segment[valid]
        self.length = length[valid]
        self.length_sq = self.length ** 2
        walked = np.r_[0.0, np.cumsum(length)]
        self.walked = walked[:-1][valid]
        self.heading = np.arctan2(self.segment[:, 1], self.segment[:, 0])
        original_indices = np.flatnonzero(valid)
        for gap_index in gap_indices:
            match = np.flatnonzero(original_indices == gap_index)
            if len(match):
                index = int(match[0])
                # A lane change can leave two centerlines ~11 m apart while
                # the ego keeps driving forward. The bridging chord points
                # sideways; use the adjacent road heading for this chord.
                if 0 < index < len(self.heading) - 1:
                    before, after = self.heading[index - 1], self.heading[index + 1]
                    disagreement = abs(math.atan2(math.sin(before - after),
                                                   math.cos(before - after)))
                    if disagreement <= math.pi / 4.0:
                        self.heading[index] = before
        self.start_s_m = float(start_s_m)
        self.end_s_m = float(walked[-1])
        self.max_lateral_distance_m = float(max_lateral_distance_m)
        self.max_heading_error_rad = float(max_heading_error_rad)
        self.start_xy = tuple(self.point_at_arc(self.start_s_m))
        self.end_xy = tuple(xy[-1])
        self.lanelet_ids = lanelet_ids

    def point_at_arc(self, arc_m):
        index = int(np.searchsorted(self.walked, arc_m, side='right') - 1)
        index = max(0, min(index, len(self.length) - 1))
        ratio = min(1.0, max(0.0, (arc_m - self.walked[index]) / self.length[index]))
        return self.origins[index] + ratio * self.segment[index]

    def evaluate(self, x, y, yaw):
        """Return (active, route_s_m, lateral_error_m); unknown pose fails open."""
        if not all(math.isfinite(float(value)) for value in (x, y, yaw)):
            return False, None, None
        position = np.asarray((x, y), dtype=np.float64)
        ratio = np.clip(np.einsum('ij,ij->i', position - self.origins, self.segment)
                        / self.length_sq, 0.0, 1.0)
        nearest = self.origins + self.segment * ratio[:, None]
        distance_sq = np.einsum('ij,ij->i', nearest - position, nearest - position)
        index = int(np.argmin(distance_sq))
        lateral = math.sqrt(float(distance_sq[index]))
        arc = float(self.walked[index] + ratio[index] * self.length[index])
        tangent = float(self.heading[index])
        heading_error = abs(math.atan2(math.sin(yaw - tangent), math.cos(yaw - tangent)))
        active = (self.start_s_m <= arc < self.end_s_m - 0.5
                  and lateral <= self.max_lateral_distance_m
                  and heading_error <= self.max_heading_error_rad)
        return active, arc, lateral
