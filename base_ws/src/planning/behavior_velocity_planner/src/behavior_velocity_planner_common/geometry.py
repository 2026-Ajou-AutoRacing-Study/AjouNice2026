import math


EPSILON = 1.0e-9


def distance_2d(a, b):
    return math.hypot(float(a[0]) - float(b[0]), float(a[1]) - float(b[1]))


def cumulative_lengths(points):
    if not points:
        return []
    lengths = [0.0]
    for previous, current in zip(points[:-1], points[1:]):
        lengths.append(lengths[-1] + distance_2d(previous, current))
    return lengths


def interpolate_point(a, b, ratio):
    ratio = max(0.0, min(float(ratio), 1.0))
    az = float(a[2]) if len(a) > 2 else 0.0
    bz = float(b[2]) if len(b) > 2 else 0.0
    return (
        float(a[0]) + (float(b[0]) - float(a[0])) * ratio,
        float(a[1]) + (float(b[1]) - float(a[1])) * ratio,
        az + (bz - az) * ratio,
    )


def point_at_station(points, station):
    if not points:
        return None
    if len(points) == 1:
        return tuple(points[0])
    lengths = cumulative_lengths(points)
    station = max(0.0, min(float(station), lengths[-1]))
    for index in range(len(points) - 1):
        if station > lengths[index + 1]:
            continue
        segment_length = max(lengths[index + 1] - lengths[index], EPSILON)
        ratio = (station - lengths[index]) / segment_length
        return interpolate_point(points[index], points[index + 1], ratio)
    return tuple(points[-1])


def project_point_to_polyline(points, target):
    if not points:
        return None
    if len(points) == 1:
        return 0.0, distance_2d(points[0], target)

    lengths = cumulative_lengths(points)
    best = None
    for index in range(len(points) - 1):
        a = points[index]
        b = points[index + 1]
        vx = float(b[0]) - float(a[0])
        vy = float(b[1]) - float(a[1])
        segment_length_sq = vx * vx + vy * vy
        if segment_length_sq <= EPSILON:
            continue
        wx = float(target[0]) - float(a[0])
        wy = float(target[1]) - float(a[1])
        ratio = max(0.0, min(1.0, (wx * vx + wy * vy) / segment_length_sq))
        projected = (float(a[0]) + vx * ratio, float(a[1]) + vy * ratio)
        distance = distance_2d(projected, target)
        station = lengths[index] + math.sqrt(segment_length_sq) * ratio
        if best is None or distance < best[1]:
            best = (station, distance, index, ratio, projected)
    return best


def segment_intersection(first_a, first_b, second_a, second_b, tolerance=1.0e-8):
    """Return (first_ratio, second_ratio, xyz) for a 2-D segment intersection."""
    rx = float(first_b[0]) - float(first_a[0])
    ry = float(first_b[1]) - float(first_a[1])
    sx = float(second_b[0]) - float(second_a[0])
    sy = float(second_b[1]) - float(second_a[1])
    denominator = rx * sy - ry * sx
    if abs(denominator) <= tolerance:
        return None

    qpx = float(second_a[0]) - float(first_a[0])
    qpy = float(second_a[1]) - float(first_a[1])
    first_ratio = (qpx * sy - qpy * sx) / denominator
    second_ratio = (qpx * ry - qpy * rx) / denominator
    if not (-tolerance <= first_ratio <= 1.0 + tolerance):
        return None
    if not (-tolerance <= second_ratio <= 1.0 + tolerance):
        return None

    first_ratio = max(0.0, min(1.0, first_ratio))
    second_ratio = max(0.0, min(1.0, second_ratio))
    az = float(first_a[2]) if len(first_a) > 2 else 0.0
    bz = float(first_b[2]) if len(first_b) > 2 else az
    point = (
        float(first_a[0]) + rx * first_ratio,
        float(first_a[1]) + ry * first_ratio,
        az + (bz - az) * first_ratio,
    )
    return first_ratio, second_ratio, point


def first_polyline_intersection(path_points, line_points):
    if len(path_points) < 2 or len(line_points) < 2:
        return None
    path_lengths = cumulative_lengths(path_points)
    best = None
    for path_index in range(len(path_points) - 1):
        path_segment_length = path_lengths[path_index + 1] - path_lengths[path_index]
        for line_index in range(len(line_points) - 1):
            intersection = segment_intersection(
                path_points[path_index],
                path_points[path_index + 1],
                line_points[line_index],
                line_points[line_index + 1],
            )
            if intersection is None:
                continue
            path_ratio, line_ratio, point = intersection
            station = path_lengths[path_index] + path_segment_length * path_ratio
            candidate = {
                "station": station,
                "point": point,
                "path_index": path_index,
                "path_ratio": path_ratio,
                "line_index": line_index,
                "line_ratio": line_ratio,
            }
            if best is None or station < best["station"]:
                best = candidate
    return best


def point_in_polygon(point, polygon_points):
    if len(polygon_points) < 3:
        return False
    x = float(point[0])
    y = float(point[1])
    inside = False
    previous = polygon_points[-1]
    for current in polygon_points:
        x1, y1 = float(previous[0]), float(previous[1])
        x2, y2 = float(current[0]), float(current[1])
        crosses = (y1 > y) != (y2 > y)
        if crosses:
            boundary_x = (x2 - x1) * (y - y1) / (y2 - y1) + x1
            if x < boundary_x:
                inside = not inside
        previous = current
    return inside


def polyline_polygon_intersections(path_points, polygon_points, tolerance=1.0e-6):
    """Return path intersections with a closed polygon, ordered by station."""
    if len(path_points) < 2 or len(polygon_points) < 3:
        return []

    path_lengths = cumulative_lengths(path_points)
    polygon_edges = list(zip(polygon_points, polygon_points[1:]))
    polygon_edges.append((polygon_points[-1], polygon_points[0]))
    intersections = []
    for path_index in range(len(path_points) - 1):
        segment_length = path_lengths[path_index + 1] - path_lengths[path_index]
        for polygon_index, (polygon_a, polygon_b) in enumerate(polygon_edges):
            intersection = segment_intersection(
                path_points[path_index],
                path_points[path_index + 1],
                polygon_a,
                polygon_b,
            )
            if intersection is None:
                continue
            path_ratio, polygon_ratio, point = intersection
            station = path_lengths[path_index] + segment_length * path_ratio
            if any(abs(item["station"] - station) <= tolerance for item in intersections):
                continue
            intersections.append(
                {
                    "station": station,
                    "point": point,
                    "path_index": path_index,
                    "path_ratio": path_ratio,
                    "polygon_index": polygon_index,
                    "polygon_ratio": polygon_ratio,
                }
            )

    if point_in_polygon(path_points[0], polygon_points):
        intersections.append(
            {
                "station": 0.0,
                "point": tuple(path_points[0]),
                "path_index": 0,
                "path_ratio": 0.0,
                "polygon_index": -1,
                "polygon_ratio": 0.0,
            }
        )
    if point_in_polygon(path_points[-1], polygon_points):
        intersections.append(
            {
                "station": path_lengths[-1],
                "point": tuple(path_points[-1]),
                "path_index": len(path_points) - 2,
                "path_ratio": 1.0,
                "polygon_index": -1,
                "polygon_ratio": 1.0,
            }
        )
    return sorted(intersections, key=lambda item: item["station"])
