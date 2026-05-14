from __future__ import annotations

from .geometry import cumulative_distances, ensure_faces_upward, signed_area_xy
from .mesh_types import Face, SurfacePatch, Vec3

try:  # pragma: no cover - optional dependency
    import mapbox_earcut
except ImportError:  # pragma: no cover - optional dependency
    mapbox_earcut = None


class TriangulationError(RuntimeError):
    """Raised when polygon triangulation fails."""


def triangulate_strip(
    left: list[Vec3],
    right: list[Vec3],
    merge_distance: float = 1e-9,
) -> SurfacePatch:
    vertices, left_indices, right_indices = _strip_vertex_indices(left, right, merge_distance)
    faces: list[Face] = []
    left_dist = cumulative_distances(left)
    right_dist = cumulative_distances(right)
    left_total = left_dist[-1] if left_dist[-1] > 1e-3 else 1.0
    right_total = right_dist[-1] if right_dist[-1] > 1e-3 else 1.0
    left_index = 0
    right_index = 0

    while left_index < len(left) - 1 or right_index < len(right) - 1:
        left_next = left_dist[left_index + 1] / left_total if left_index < len(left) - 1 else 2.0
        right_next = (
            right_dist[right_index + 1] / right_total if right_index < len(right) - 1 else 2.0
        )
        if left_next <= right_next and left_index < len(left) - 1:
            faces.append(
                (
                    left_indices[left_index],
                    right_indices[right_index],
                    left_indices[left_index + 1],
                )
            )
            left_index += 1
        elif right_index < len(right) - 1:
            faces.append(
                (
                    left_indices[left_index],
                    right_indices[right_index],
                    right_indices[right_index + 1],
                )
            )
            right_index += 1
        else:
            break

    faces = ensure_faces_upward(faces, vertices)
    boundary_loop = _collapse_boundary_loop(left_indices + list(reversed(right_indices)))
    return SurfacePatch(vertices=vertices, faces=faces, boundary_loops=[boundary_loop])


def triangulate_polygon(points: list[Vec3]) -> SurfacePatch:
    candidate = list(points)
    faces = _triangulate_polygon_once(candidate)
    if faces is None:
        candidate = list(reversed(points))
        faces = _triangulate_polygon_once(candidate)
        if faces is None:
            raise TriangulationError("Polygon triangulation failed after reversing winding.")
    faces = ensure_faces_upward(faces, candidate)
    return SurfacePatch(
        vertices=candidate,
        faces=faces,
        boundary_loops=[list(range(len(candidate)))],
    )


def _triangulate_polygon_once(points: list[Vec3]) -> list[Face] | None:
    if len(points) < 3:
        return None
    if mapbox_earcut is not None:  # pragma: no branch - direct backend when available
        try:
            coordinates: list[float] = []
            for point in points:
                coordinates.extend([point.x, point.y])
            indices = mapbox_earcut.triangulate_float64(coordinates, [len(points)])
            if len(indices) % 3 != 0:
                return _ear_clip(points)
            return [
                (int(indices[index]), int(indices[index + 1]), int(indices[index + 2]))
                for index in range(0, len(indices), 3)
            ]
        except Exception:
            return _ear_clip(points)
    return _ear_clip(points)


def _ear_clip(points: list[Vec3]) -> list[Face] | None:
    if len(points) < 3:
        return None
    working = list(range(len(points)))
    if signed_area_xy(points) < 0.0:
        working.reverse()

    faces: list[Face] = []
    guard = 0
    while len(working) > 3 and guard < len(points) * len(points):
        guard += 1
        ear_found = False
        for index in range(len(working)):
            prev_index = working[(index - 1) % len(working)]
            curr_index = working[index]
            next_index = working[(index + 1) % len(working)]
            a = points[prev_index]
            b = points[curr_index]
            c = points[next_index]
            if not _is_convex(a, b, c):
                continue
            if _contains_other_point(points, working, prev_index, curr_index, next_index):
                continue
            faces.append((prev_index, curr_index, next_index))
            del working[index]
            ear_found = True
            break
        if not ear_found:
            return None

    if len(working) != 3:
        return None
    faces.append((working[0], working[1], working[2]))
    return faces


def _is_convex(a: Vec3, b: Vec3, c: Vec3) -> bool:
    ab_x = b.x - a.x
    ab_y = b.y - a.y
    bc_x = c.x - b.x
    bc_y = c.y - b.y
    return ab_x * bc_y - ab_y * bc_x > 1e-12


def _contains_other_point(
    points: list[Vec3],
    working: list[int],
    prev_index: int,
    curr_index: int,
    next_index: int,
) -> bool:
    a = points[prev_index]
    b = points[curr_index]
    c = points[next_index]
    for candidate_index in working:
        if candidate_index in (prev_index, curr_index, next_index):
            continue
        if _point_in_triangle(points[candidate_index], a, b, c):
            return True
    return False


def _point_in_triangle(point: Vec3, a: Vec3, b: Vec3, c: Vec3) -> bool:
    b1 = _sign(point, a, b) >= -1e-12
    b2 = _sign(point, b, c) >= -1e-12
    b3 = _sign(point, c, a) >= -1e-12
    return b1 and b2 and b3


def _sign(point: Vec3, a: Vec3, b: Vec3) -> float:
    return (point.x - b.x) * (a.y - b.y) - (a.x - b.x) * (point.y - b.y)


def _strip_vertex_indices(
    left: list[Vec3],
    right: list[Vec3],
    merge_distance: float,
) -> tuple[list[Vec3], list[int], list[int]]:
    vertices = list(left)
    left_indices = list(range(len(left)))
    right_indices: list[int] = []
    for index, point in enumerate(right):
        if index == 0 and _same_point_xy(point, left[0], merge_distance):
            right_indices.append(left_indices[0])
            continue
        if index == len(right) - 1 and _same_point_xy(point, left[-1], merge_distance):
            right_indices.append(left_indices[-1])
            continue
        right_indices.append(len(vertices))
        vertices.append(point)
    return vertices, left_indices, right_indices


def _collapse_boundary_loop(indices: list[int]) -> list[int]:
    collapsed: list[int] = []
    for index in indices:
        if collapsed and collapsed[-1] == index:
            continue
        collapsed.append(index)
    if len(collapsed) > 1 and collapsed[0] == collapsed[-1]:
        collapsed.pop()
    return collapsed


def _same_point_xy(a: Vec3, b: Vec3, tolerance: float) -> bool:
    return a.distance_to(b) <= tolerance
