from __future__ import annotations

import logging

from .geometry import (
    cumulative_distances,
    ensure_face_normal,
    ensure_loop_ccw,
    sanitize_loop,
    sanitize_polyline,
    triangle_area,
)
from .mesh_types import (
    Face,
    LaneletFeature,
    MeshLayer,
    PolylineFeature,
    PolygonFeature,
    SurfacePatch,
    TriangleMesh,
    Vec3,
)
from .triangulation import triangulate_polygon, triangulate_strip
from .validate import mesh_boundary_edge_count, mesh_component_signed_volume, mesh_face_components


class FeatureBuildError(RuntimeError):
    """Raised when one feature cannot be converted into a mesh."""


def build_surface_layer(
    layer_name: str,
    features: list[LaneletFeature] | list[PolygonFeature],
    thickness: float,
    min_segment_length: float,
    min_triangle_area: float,
    filtered_counts: dict[str, int],
    logger: logging.Logger | None = None,
    side_overlap: float = 0.0,
    surface_style: str = "solid",
) -> MeshLayer:
    mesh = TriangleMesh()
    top_vertices: list[Vec3] = []
    patch_count = 0
    for feature in features:
        try:
            patches = _surface_patches_for_feature(
                feature,
                min_segment_length,
                filtered_counts,
                logger,
                layer_name,
                side_overlap,
            )
        except Exception as exc:
            message = _feature_error_message(layer_name, feature, exc)
            if logger:
                logger.exception(message)
            raise FeatureBuildError(message) from exc
        if not patches:
            continue
        feature_built = False
        for patch_index, patch in enumerate(patches):
            if surface_style == "flat":
                patch_mesh = _build_double_sided_surface_from_patch(
                    patch=patch,
                    min_triangle_area=min_triangle_area,
                    filtered_counts=filtered_counts,
                )
            elif surface_style == "solid":
                patch_mesh = _extrude_patch(
                    patch=patch,
                    thickness=thickness,
                    min_triangle_area=min_triangle_area,
                    filtered_counts=filtered_counts,
                )
            else:  # pragma: no cover - parser/config protect this
                raise ValueError(f"Unsupported surface style: {surface_style}")
            if not patch_mesh.faces:
                continue
            if surface_style == "solid":
                _ensure_closed_feature_mesh(
                    mesh=patch_mesh,
                    layer_name=layer_name,
                    feature=feature,
                    patch_index=patch_index,
                    logger=logger,
                )
            feature_built = True
            mesh.append_mesh(patch_mesh)
            top_vertices.extend(patch.vertices)
        if feature_built:
            patch_count += 1
    return MeshLayer(
        name=layer_name,
        mesh=mesh,
        source_feature_count=patch_count,
        top_vertices=top_vertices,
        requires_closed_volume=surface_style == "solid",
    )


def build_wall_layer(
    layer_name: str,
    features: list[PolylineFeature],
    wall_height: float,
    wall_thickness: float,
    min_segment_length: float,
    min_triangle_area: float,
    filtered_counts: dict[str, int],
    logger: logging.Logger | None = None,
) -> MeshLayer:
    mesh = TriangleMesh()
    built_count = 0
    for feature in features:
        points = sanitize_polyline(feature.points, min_segment_length, filtered_counts)
        if len(points) < 2:
            continue
        built_any = False
        for start, end in zip(points, points[1:]):
            segment_mesh = _build_polyline_segment_prism(
                start=start,
                end=end,
                width=wall_thickness,
                bottom_offset=0.0,
                top_offset=wall_height,
                min_triangle_area=min_triangle_area,
                filtered_counts=filtered_counts,
            )
            if not segment_mesh.faces:
                continue
            built_any = True
            mesh.append_mesh(segment_mesh)
        if built_any:
            built_count += 1
    return MeshLayer(name=layer_name, mesh=mesh, source_feature_count=built_count)


def build_marking_layer(
    layer_name: str,
    features: list[PolylineFeature],
    marking_width: float,
    marking_thickness: float,
    marking_offset: float,
    marking_style: str,
    min_segment_length: float,
    min_triangle_area: float,
    filtered_counts: dict[str, int],
    logger: logging.Logger | None = None,
) -> MeshLayer:
    mesh = TriangleMesh()
    built_count = 0
    for feature in features:
        points = sanitize_polyline(feature.points, min_segment_length, filtered_counts)
        if len(points) < 2:
            continue
        built_any = False
        for start, end in zip(points, points[1:]):
            if marking_style == "flat":
                segment_mesh = _build_polyline_segment_flat_mesh(
                    start=start,
                    end=end,
                    width=marking_width,
                    lift=marking_offset,
                    min_triangle_area=min_triangle_area,
                    filtered_counts=filtered_counts,
                )
            elif marking_style == "prism":
                segment_mesh = _build_polyline_segment_prism(
                    start=start,
                    end=end,
                    width=marking_width,
                    bottom_offset=marking_offset,
                    top_offset=marking_offset + marking_thickness,
                    min_triangle_area=min_triangle_area,
                    filtered_counts=filtered_counts,
                )
            else:  # pragma: no cover - parser/config protect this
                raise ValueError(f"Unsupported marking style: {marking_style}")
            if not segment_mesh.faces:
                continue
            built_any = True
            mesh.append_mesh(segment_mesh)
        if built_any:
            built_count += 1
    return MeshLayer(
        name=layer_name,
        mesh=mesh,
        source_feature_count=built_count,
        requires_closed_volume=marking_style == "prism",
    )


def compute_origin_shift(road_top_vertices: list[Vec3], layers: list[MeshLayer]) -> Vec3:
    if not road_top_vertices:
        return Vec3(0.0, 0.0, 0.0)
    min_x = max_x = road_top_vertices[0].x
    min_y = max_y = road_top_vertices[0].y
    for vertex in road_top_vertices[1:]:
        min_x = min(min_x, vertex.x)
        min_y = min(min_y, vertex.y)
        max_x = max(max_x, vertex.x)
        max_y = max(max_y, vertex.y)
    min_z = min((vertex.z for layer in layers for vertex in layer.mesh.vertices), default=0.0)
    return Vec3((min_x + max_x) * 0.5, (min_y + max_y) * 0.5, min_z)


def _surface_patches_for_feature(
    feature: LaneletFeature | PolygonFeature,
    min_segment_length: float,
    filtered_counts: dict[str, int],
    logger: logging.Logger | None,
    layer_name: str,
    side_overlap: float,
) -> list[SurfacePatch]:
    if isinstance(feature, LaneletFeature):
        left = sanitize_polyline(feature.left, min_segment_length, filtered_counts)
        right = sanitize_polyline(feature.right, min_segment_length, filtered_counts)
        if len(left) < 2 or len(right) < 2:
            return []
        if side_overlap > 0.0:
            left, right = _widen_lanelet_boundaries(left, right, side_overlap)
        return [triangulate_strip(left, right, merge_distance=min_segment_length)]

    points = sanitize_loop(feature.points, min_segment_length, filtered_counts)
    if len(points) < 3:
        return []

    repaired_loops = _split_repeated_vertex_loops(points, filtered_counts)
    if len(repaired_loops) > 1 and logger:
        logger.info(
            "%s: feature_id=%d repaired into %d simple loops",
            layer_name,
            feature.feature_id,
            len(repaired_loops),
        )
    return [triangulate_polygon(loop) for loop in repaired_loops if len(loop) >= 3]


def _widen_lanelet_boundaries(
    left: list[Vec3],
    right: list[Vec3],
    side_overlap: float,
) -> tuple[list[Vec3], list[Vec3]]:
    left_distances = cumulative_distances(left)
    right_distances = cumulative_distances(right)
    return (
        _offset_boundary_away_from_opposite(
            boundary=left,
            boundary_distances=left_distances,
            opposite=right,
            opposite_distances=right_distances,
            distance=side_overlap,
            fallback_side="left",
        ),
        _offset_boundary_away_from_opposite(
            boundary=right,
            boundary_distances=right_distances,
            opposite=left,
            opposite_distances=left_distances,
            distance=side_overlap,
            fallback_side="right",
        ),
    )


def _offset_boundary_away_from_opposite(
    boundary: list[Vec3],
    boundary_distances: list[float],
    opposite: list[Vec3],
    opposite_distances: list[float],
    distance: float,
    fallback_side: str,
) -> list[Vec3]:
    if not boundary:
        return []

    total_distance = boundary_distances[-1] if boundary_distances else 0.0
    opposite_total = opposite_distances[-1] if opposite_distances else 0.0
    widened: list[Vec3] = []
    for index, point in enumerate(boundary):
        if total_distance > 1e-9:
            ratio = boundary_distances[index] / total_distance
        elif len(boundary) > 1:
            ratio = index / (len(boundary) - 1)
        else:
            ratio = 0.0
        opposite_point = _interpolate_polyline_point(opposite, opposite_distances, ratio * opposite_total)
        outward = Vec3(point.x - opposite_point.x, point.y - opposite_point.y, 0.0)
        if outward.length() <= 1e-9:
            outward = _polyline_side_normal(boundary, index, fallback_side)
        else:
            outward = outward.normalized()
        widened.append(
            Vec3(
                point.x + outward.x * distance,
                point.y + outward.y * distance,
                point.z,
            )
        )
    return widened


def _interpolate_polyline_point(
    points: list[Vec3],
    distances: list[float],
    target_distance: float,
) -> Vec3:
    if not points:
        return Vec3(0.0, 0.0, 0.0)
    if len(points) == 1 or not distances or target_distance <= 0.0:
        return points[0]
    if target_distance >= distances[-1]:
        return points[-1]

    for index in range(1, len(points)):
        if distances[index] < target_distance:
            continue
        start = points[index - 1]
        end = points[index]
        segment_length = distances[index] - distances[index - 1]
        if segment_length <= 1e-12:
            return end
        ratio = (target_distance - distances[index - 1]) / segment_length
        return Vec3(
            start.x + (end.x - start.x) * ratio,
            start.y + (end.y - start.y) * ratio,
            start.z + (end.z - start.z) * ratio,
        )
    return points[-1]


def _polyline_side_normal(points: list[Vec3], index: int, side: str) -> Vec3:
    tangent = Vec3(0.0, 0.0, 0.0)
    point = points[index]
    if index > 0:
        prev_point = points[index - 1]
        tangent = tangent + Vec3(point.x - prev_point.x, point.y - prev_point.y, 0.0)
    if index + 1 < len(points):
        next_point = points[index + 1]
        tangent = tangent + Vec3(next_point.x - point.x, next_point.y - point.y, 0.0)
    if tangent.length() <= 1e-9:
        if index + 1 < len(points):
            next_point = points[index + 1]
            tangent = Vec3(next_point.x - point.x, next_point.y - point.y, 0.0)
        elif index > 0:
            prev_point = points[index - 1]
            tangent = Vec3(point.x - prev_point.x, point.y - prev_point.y, 0.0)
    tangent = tangent.normalized()
    if side == "left":
        return Vec3(-tangent.y, tangent.x, 0.0)
    return Vec3(tangent.y, -tangent.x, 0.0)


def _extrude_patch(
    patch: SurfacePatch,
    thickness: float,
    min_triangle_area: float,
    filtered_counts: dict[str, int],
) -> TriangleMesh:
    mesh = TriangleMesh()
    top_indices = [mesh.add_vertex(vertex) for vertex in patch.vertices]
    bottom_indices = [mesh.add_vertex(Vec3(vertex.x, vertex.y, vertex.z - thickness)) for vertex in patch.vertices]
    top_faces: list[Face] = []

    for face in patch.faces:
        top_face = (top_indices[face[0]], top_indices[face[1]], top_indices[face[2]])
        if _add_face_if_valid(mesh, top_face, min_triangle_area, filtered_counts):
            top_faces.append(top_face)
        bottom_face = (bottom_indices[face[0]], bottom_indices[face[2]], bottom_indices[face[1]])
        _add_face_if_valid(mesh, bottom_face, min_triangle_area, filtered_counts)

    for top_start, top_end in _boundary_edges_from_faces(top_faces):
        bottom_start = bottom_indices[top_start]
        bottom_end = bottom_indices[top_end]
        _add_side_quad(
            mesh=mesh,
            top_start=top_start,
            top_end=top_end,
            bottom_end=bottom_end,
            bottom_start=bottom_start,
            expected_outward=_edge_outward(
                mesh.vertices[top_start],
                mesh.vertices[top_end],
            ),
            min_triangle_area=min_triangle_area,
            filtered_counts=filtered_counts,
        )
    return mesh


def _build_polyline_segment_prism(
    start: Vec3,
    end: Vec3,
    width: float,
    bottom_offset: float,
    top_offset: float,
    min_triangle_area: float,
    filtered_counts: dict[str, int],
) -> TriangleMesh:
    direction = Vec3(end.x - start.x, end.y - start.y, 0.0)
    if direction.length() <= 1e-12:
        return TriangleMesh()
    perpendicular = Vec3(-direction.y, direction.x, 0.0).normalized() * (width * 0.5)
    bottom_loop = ensure_loop_ccw(
        [
            Vec3(start.x + perpendicular.x, start.y + perpendicular.y, start.z + bottom_offset),
            Vec3(end.x + perpendicular.x, end.y + perpendicular.y, end.z + bottom_offset),
            Vec3(end.x - perpendicular.x, end.y - perpendicular.y, end.z + bottom_offset),
            Vec3(start.x - perpendicular.x, start.y - perpendicular.y, start.z + bottom_offset),
        ]
    )
    top_loop = [Vec3(point.x, point.y, point.z + (top_offset - bottom_offset)) for point in bottom_loop]
    return _build_closed_prism_from_loops(
        top_loop=top_loop,
        bottom_loop=bottom_loop,
        min_triangle_area=min_triangle_area,
        filtered_counts=filtered_counts,
    )


def _build_polyline_segment_flat_mesh(
    start: Vec3,
    end: Vec3,
    width: float,
    lift: float,
    min_triangle_area: float,
    filtered_counts: dict[str, int],
) -> TriangleMesh:
    direction = Vec3(end.x - start.x, end.y - start.y, 0.0)
    if direction.length() <= 1e-12:
        return TriangleMesh()
    perpendicular = Vec3(-direction.y, direction.x, 0.0).normalized() * (width * 0.5)
    loop = ensure_loop_ccw(
        [
            Vec3(start.x + perpendicular.x, start.y + perpendicular.y, start.z + lift),
            Vec3(end.x + perpendicular.x, end.y + perpendicular.y, end.z + lift),
            Vec3(end.x - perpendicular.x, end.y - perpendicular.y, end.z + lift),
            Vec3(start.x - perpendicular.x, start.y - perpendicular.y, start.z + lift),
        ]
    )
    return _build_double_sided_surface_from_loop(
        loop=loop,
        min_triangle_area=min_triangle_area,
        filtered_counts=filtered_counts,
    )


def _build_closed_prism_from_loops(
    top_loop: list[Vec3],
    bottom_loop: list[Vec3],
    min_triangle_area: float,
    filtered_counts: dict[str, int],
) -> TriangleMesh:
    mesh = TriangleMesh()
    top_indices = [mesh.add_vertex(point) for point in top_loop]
    bottom_indices = [mesh.add_vertex(point) for point in bottom_loop]
    top_faces = _fan_faces(top_indices)
    bottom_faces = [(face[0], face[2], face[1]) for face in _fan_faces(bottom_indices)]

    for face in top_faces + bottom_faces:
        _add_face_if_valid(mesh, face, min_triangle_area, filtered_counts)

    for loop_index in range(len(top_indices)):
        next_index = (loop_index + 1) % len(top_indices)
        _add_side_quad(
            mesh=mesh,
            top_start=top_indices[loop_index],
            top_end=top_indices[next_index],
            bottom_end=bottom_indices[next_index],
            bottom_start=bottom_indices[loop_index],
            expected_outward=_edge_outward(
                mesh.vertices[top_indices[loop_index]],
                mesh.vertices[top_indices[next_index]],
            ),
            min_triangle_area=min_triangle_area,
            filtered_counts=filtered_counts,
    )
    return mesh


def _build_double_sided_surface_from_loop(
    loop: list[Vec3],
    min_triangle_area: float,
    filtered_counts: dict[str, int],
) -> TriangleMesh:
    mesh = TriangleMesh()
    indices = [mesh.add_vertex(point) for point in loop]
    faces = _fan_faces(indices)
    for face in faces:
        _add_face_if_valid(mesh, face, min_triangle_area, filtered_counts)
    for a, b, c in faces:
        _add_face_if_valid(mesh, (a, c, b), min_triangle_area, filtered_counts)
    return mesh


def _build_double_sided_surface_from_patch(
    patch: SurfacePatch,
    min_triangle_area: float,
    filtered_counts: dict[str, int],
) -> TriangleMesh:
    mesh = TriangleMesh()
    indices = [mesh.add_vertex(point) for point in patch.vertices]
    faces: list[Face] = []
    for a, b, c in patch.faces:
        face = (indices[a], indices[b], indices[c])
        if _add_face_if_valid(mesh, face, min_triangle_area, filtered_counts):
            faces.append(face)
    for a, b, c in faces:
        _add_face_if_valid(mesh, (a, c, b), min_triangle_area, filtered_counts)
    return mesh


def _fan_faces(indices: list[int]) -> list[Face]:
    return [(indices[0], indices[offset], indices[offset + 1]) for offset in range(1, len(indices) - 1)]


def _add_side_quad(
    mesh: TriangleMesh,
    top_start: int,
    top_end: int,
    bottom_end: int,
    bottom_start: int,
    expected_outward: Vec3,
    min_triangle_area: float,
    filtered_counts: dict[str, int],
) -> None:
    first = ensure_face_normal(
        (top_start, bottom_end, top_end),
        mesh.vertices,
        expected_outward,
    )
    second = ensure_face_normal(
        (top_start, bottom_start, bottom_end),
        mesh.vertices,
        expected_outward,
    )
    _add_face_if_valid(mesh, first, min_triangle_area, filtered_counts)
    _add_face_if_valid(mesh, second, min_triangle_area, filtered_counts)


def _edge_outward(start: Vec3, end: Vec3) -> Vec3:
    edge = end - start
    return Vec3(edge.y, -edge.x, 0.0).normalized()


def _add_face_if_valid(
    mesh: TriangleMesh,
    face: Face,
    min_triangle_area: float,
    filtered_counts: dict[str, int],
) -> bool:
    if triangle_area(face, mesh.vertices) < min_triangle_area:
        filtered_counts["degenerate_triangles_removed"] = (
            filtered_counts.get("degenerate_triangles_removed", 0) + 1
        )
        return False
    mesh.add_face(face)
    return True


def _boundary_edges_from_faces(faces: list[Face]) -> list[tuple[int, int]]:
    counts: dict[tuple[int, int], int] = {}
    directions: dict[tuple[int, int], tuple[int, int]] = {}
    for a, b, c in faces:
        for start, end in ((a, b), (b, c), (c, a)):
            key = (start, end) if start < end else (end, start)
            counts[key] = counts.get(key, 0) + 1
            directions[key] = (start, end)
    boundary: list[tuple[int, int]] = []
    for key, count in counts.items():
        if count == 1:
            boundary.append(directions[key])
    return boundary


def _feature_error_message(
    layer_name: str,
    feature: LaneletFeature | PolygonFeature,
    exc: Exception,
) -> str:
    if isinstance(feature, LaneletFeature):
        points = list(feature.left) + list(feature.right)
        point_count = len(feature.left) + len(feature.right)
    else:
        points = list(feature.points)
        point_count = len(feature.points)
    if points:
        min_x = min(point.x for point in points)
        min_y = min(point.y for point in points)
        min_z = min(point.z for point in points)
        max_x = max(point.x for point in points)
        max_y = max(point.y for point in points)
        max_z = max(point.z for point in points)
        bbox = [[min_x, min_y, min_z], [max_x, max_y, max_z]]
    else:
        bbox = None
    return (
        f"{layer_name}: feature_id={feature.feature_id} point_count={point_count} "
        f"bbox={bbox} failed: {exc}"
    )


def _ensure_closed_feature_mesh(
    mesh: TriangleMesh,
    layer_name: str,
    feature: LaneletFeature | PolygonFeature,
    patch_index: int,
    logger: logging.Logger | None,
) -> None:
    boundary_edges = mesh_boundary_edge_count(mesh)
    if boundary_edges:
        message = (
            f"{layer_name}: feature_id={feature.feature_id} patch_index={patch_index} "
            f"generated solid has {boundary_edges} open edges"
        )
        if logger:
            logger.error(message)
        raise FeatureBuildError(message)

    components = mesh_face_components(mesh)
    for component_index, component in enumerate(components):
        volume = mesh_component_signed_volume(mesh, component)
        if volume < 0.0:
            _flip_mesh_component_faces(mesh, component)
            corrected_volume = mesh_component_signed_volume(mesh, component)
            if logger:
                logger.info(
                    "%s: feature_id=%d patch_index=%d component_index=%d "
                    "flipped inside-out winding (signed volume %.12g -> %.12g)",
                    layer_name,
                    feature.feature_id,
                    patch_index,
                    component_index,
                    volume,
                    corrected_volume,
                )
            volume = corrected_volume
        if volume > 0.0:
            continue
        message = (
            f"{layer_name}: feature_id={feature.feature_id} patch_index={patch_index} "
            f"component_index={component_index} signed volume is not positive ({volume:.12g})"
        )
        if logger:
            logger.warning(message)


def _flip_mesh_component_faces(mesh: TriangleMesh, component: list[int]) -> None:
    for face_index in component:
        a, b, c = mesh.faces[face_index]
        mesh.faces[face_index] = (a, c, b)


def _split_repeated_vertex_loops(
    points: list[Vec3],
    filtered_counts: dict[str, int],
) -> list[list[Vec3]]:
    if len(points) < 3:
        return []

    path = list(points) + [points[0]]
    loops: list[list[Vec3]] = []
    while True:
        seen: dict[Vec3, int] = {}
        split_pair: tuple[int, int] | None = None
        for index, point in enumerate(path):
            if point in seen and index - seen[point] > 1:
                if index == len(path) - 1 and seen[point] == 0:
                    continue
                split_pair = (seen[point], index)
                break
            seen[point] = index
        if split_pair is None:
            break
        start_index, end_index = split_pair
        cycle = path[start_index:end_index]
        if len(cycle) >= 3:
            loops.append(cycle)
            filtered_counts["polygons_split_on_repeated_vertices"] = (
                filtered_counts.get("polygons_split_on_repeated_vertices", 0) + 1
            )
        path = path[: start_index + 1] + path[end_index + 1 :]
        if len(path) < 4:
            break

    final_loop = path[:-1]
    if len(final_loop) >= 3:
        loops.append(final_loop)
    return loops
