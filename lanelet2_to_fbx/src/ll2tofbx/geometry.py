from __future__ import annotations

from .mesh_types import Face, Vec3


def signed_area_xy(points: list[Vec3]) -> float:
    total = 0.0
    for index, point in enumerate(points):
        nxt = points[(index + 1) % len(points)]
        total += point.x * nxt.y - nxt.x * point.y
    return total * 0.5


def triangle_area(face: Face, vertices: list[Vec3]) -> float:
    a, b, c = face
    return triangle_area_vertices(vertices[a], vertices[b], vertices[c])


def triangle_area_vertices(a: Vec3, b: Vec3, c: Vec3) -> float:
    return (b - a).cross(c - a).length() * 0.5


def triangle_normal(a: Vec3, b: Vec3, c: Vec3) -> Vec3:
    return (b - a).cross(c - a)


def ensure_face_normal(face: Face, vertices: list[Vec3], expected: Vec3) -> Face:
    a, b, c = face
    normal = triangle_normal(vertices[a], vertices[b], vertices[c])
    if normal.dot(expected) < 0.0:
        return (a, c, b)
    return face


def ensure_faces_upward(faces: list[Face], vertices: list[Vec3]) -> list[Face]:
    upward = Vec3(0.0, 0.0, 1.0)
    return [ensure_face_normal(face, vertices, upward) for face in faces]


def cumulative_distances(points: list[Vec3]) -> list[float]:
    """Return cumulative segment lengths along a polyline."""
    distances = [0.0]
    for previous, current in zip(points, points[1:]):
        distances.append(distances[-1] + current.distance_to(previous))
    return distances


def polyline_length(points: list[Vec3]) -> float:
    """Return total length of a polyline."""
    return cumulative_distances(points)[-1]


def sanitize_polyline(
    points: list[Vec3],
    min_segment_length: float,
    counts: dict[str, int],
) -> list[Vec3]:
    if not points:
        return []
    cleaned = [points[0]]
    for point in points[1:]:
        distance = point.distance_to(cleaned[-1])
        if distance <= 1e-12:
            counts["consecutive_duplicate_points_removed"] = (
                counts.get("consecutive_duplicate_points_removed", 0) + 1
            )
            continue
        if distance <= min_segment_length:
            counts["short_segments_removed"] = counts.get("short_segments_removed", 0) + 1
            continue
        cleaned.append(point)
    return cleaned


def sanitize_loop(
    points: list[Vec3],
    min_segment_length: float,
    counts: dict[str, int],
) -> list[Vec3]:
    if not points:
        return []
    cleaned = sanitize_polyline(points, min_segment_length, counts)
    if len(cleaned) > 1 and cleaned[0].distance_to(cleaned[-1]) <= 1e-12:
        cleaned.pop()
        counts["consecutive_duplicate_points_removed"] = (
            counts.get("consecutive_duplicate_points_removed", 0) + 1
        )
    changed = True
    while changed and len(cleaned) >= 3:
        changed = False
        candidate = [cleaned[0]]
        for point in cleaned[1:]:
            if point.distance_to(candidate[-1]) <= min_segment_length:
                counts["short_segments_removed"] = counts.get("short_segments_removed", 0) + 1
                changed = True
                continue
            candidate.append(point)
        if len(candidate) > 2 and candidate[-1].distance_to(candidate[0]) <= min_segment_length:
            candidate.pop()
            counts["short_segments_removed"] = counts.get("short_segments_removed", 0) + 1
            changed = True
        cleaned = candidate
    return cleaned


def ensure_loop_ccw(points: list[Vec3]) -> list[Vec3]:
    if signed_area_xy(points) < 0.0:
        return list(reversed(points))
    return list(points)
