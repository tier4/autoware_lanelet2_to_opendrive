from __future__ import annotations

from heapq import heappop, heappush
from typing import Callable

from .geometry import polyline_length
from .mesh_types import FeatureSet, LaneletFeature, PolygonFeature, PolylineFeature, Vec3

try:  # pragma: no cover - optional dependency
    from scipy.spatial import KDTree
except ImportError:  # pragma: no cover - optional dependency
    KDTree = None


MARKING_TYPES = {"line_thin", "line_thick", "stop_line"}


def extract_feature_set(lanelet_map: object, resolver: Callable[[object], Vec3]) -> FeatureSet:
    return FeatureSet(
        lanelet_roads=_extract_lanelets(lanelet_map, resolver, excluded_subtypes={"road_shoulder"}),
        intersection_areas=_extract_polygons_by_type(lanelet_map, resolver, "intersection_area"),
        hatched_areas=_extract_polygons_by_type(lanelet_map, resolver, "hatched_road_markings"),
        parking_lots=_extract_polygons_by_type(lanelet_map, resolver, "parking_lot"),
        road_borders=_extract_road_borders(lanelet_map, resolver),
        shoulders=_extract_shoulders(lanelet_map, resolver),
        road_markings=_extract_road_markings(lanelet_map, resolver),
    )


def _extract_lanelets(
    lanelet_map: object,
    resolver: Callable[[object], Vec3],
    allowed_subtypes: set[str] | None = None,
    excluded_subtypes: set[str] | None = None,
) -> list[LaneletFeature]:
    features: list[LaneletFeature] = []
    for lanelet in lanelet_map.laneletLayer:
        subtype = _attr_str(lanelet, "subtype")
        if allowed_subtypes is not None and subtype not in allowed_subtypes:
            continue
        if excluded_subtypes is not None and subtype in excluded_subtypes:
            continue
        left = [resolver(point) for point in lanelet.leftBound]
        right = [resolver(point) for point in lanelet.rightBound]
        if len(left) < 2 or len(right) < 2:
            continue
        features.append(LaneletFeature(feature_id=int(lanelet.id), left=left, right=right))
    return features


def _extract_polygons_by_type(
    lanelet_map: object,
    resolver: Callable[[object], Vec3],
    type_name: str,
) -> list[PolygonFeature]:
    features: list[PolygonFeature] = []
    for polygon in lanelet_map.polygonLayer:
        if _attr_str(polygon, "type") != type_name:
            continue
        points = [resolver(point) for point in polygon]
        if len(points) < 3:
            continue
        features.append(PolygonFeature(feature_id=int(polygon.id), points=points))
    return features


def _extract_road_borders(
    lanelet_map: object,
    resolver: Callable[[object], Vec3],
) -> list[PolylineFeature]:
    features: list[PolylineFeature] = []
    for line_string in lanelet_map.lineStringLayer:
        if _attr_str(line_string, "type") != "road_border":
            continue
        points = [resolver(point) for point in line_string]
        if len(points) < 2:
            continue
        features.append(
            PolylineFeature(
                feature_id=int(line_string.id),
                points=points,
                feature_type="road_border",
            )
        )
    return features


def _extract_shoulders(
    lanelet_map: object,
    resolver: Callable[[object], Vec3],
) -> list[LaneletFeature]:
    explicit_shoulders = _extract_lanelets(
        lanelet_map,
        resolver,
        allowed_subtypes={"road_shoulder"},
    )
    inferred_shoulders = _extract_inferred_shoulders(lanelet_map, resolver)
    return explicit_shoulders + inferred_shoulders


def _extract_inferred_shoulders(
    lanelet_map: object,
    resolver: Callable[[object], Vec3],
) -> list[LaneletFeature]:
    boundary_usage: dict[int, int] = {}
    for lanelet in lanelet_map.laneletLayer:
        boundary_usage[int(lanelet.leftBound.id)] = boundary_usage.get(int(lanelet.leftBound.id), 0) + 1
        boundary_usage[int(lanelet.rightBound.id)] = boundary_usage.get(int(lanelet.rightBound.id), 0) + 1

    road_border_graph = _build_road_border_graph(lanelet_map, resolver)
    if not road_border_graph:
        return []

    features: list[LaneletFeature] = []
    seen_boundaries: set[int] = set()
    for lanelet in lanelet_map.laneletLayer:
        if _attr_str(lanelet, "subtype") != "road":
            continue
        for line_string in (lanelet.leftBound, lanelet.rightBound):
            line_id = int(line_string.id)
            if line_id in seen_boundaries:
                continue
            seen_boundaries.add(line_id)
            if boundary_usage.get(line_id, 0) != 1:
                continue
            if _attr_str(line_string, "type") == "road_border":
                continue
            feature = _feature_between_boundary_and_road_border(
                line_string=line_string,
                resolver=resolver,
                road_border_graph=road_border_graph,
            )
            if feature is not None:
                features.append(feature)
    return features


def _extract_road_markings(
    lanelet_map: object,
    resolver: Callable[[object], Vec3],
) -> list[PolylineFeature]:
    features: list[PolylineFeature] = []
    for line_string in lanelet_map.lineStringLayer:
        feature_type = _attr_str(line_string, "type")
        if feature_type not in MARKING_TYPES:
            continue
        points = [resolver(point) for point in line_string]
        if len(points) < 2:
            continue
        features.append(
            PolylineFeature(
                feature_id=int(line_string.id),
                points=points,
                feature_type=feature_type,
            )
        )
    return features


def _build_road_border_graph(
    lanelet_map: object,
    resolver: Callable[[object], Vec3],
) -> tuple[dict[int, Vec3], dict[int, list[tuple[int, float]]], list[int], object | None] | None:
    positions: dict[int, Vec3] = {}
    adjacency: dict[int, list[tuple[int, float]]] = {}
    for line_string in lanelet_map.lineStringLayer:
        if _attr_str(line_string, "type") != "road_border":
            continue
        node_ids: list[int] = []
        for point in line_string:
            point_id = getattr(point, "id", None)
            if point_id is None:
                node_ids = []
                break
            node_id = int(point_id)
            node_ids.append(node_id)
            if node_id not in positions:
                positions[node_id] = resolver(point)
            adjacency.setdefault(node_id, [])
        for start, end in zip(node_ids, node_ids[1:]):
            distance = positions[start].distance_to(positions[end])
            adjacency[start].append((end, distance))
            adjacency[end].append((start, distance))
    if not positions:
        return None
    node_ids = list(positions)
    tree = None
    if KDTree is not None:  # pragma: no branch - optional acceleration
        tree = KDTree([(positions[node_id].x, positions[node_id].y) for node_id in node_ids])
    return positions, adjacency, node_ids, tree


def _feature_between_boundary_and_road_border(
    line_string: object,
    resolver: Callable[[object], Vec3],
    road_border_graph: tuple[dict[int, Vec3], dict[int, list[tuple[int, float]]], list[int], object | None],
) -> LaneletFeature | None:
    boundary_points = [resolver(point) for point in line_string]
    if len(boundary_points) < 2:
        return None

    border_points = _best_matching_road_border_path(boundary_points, road_border_graph)
    if border_points is None or len(border_points) < 2:
        return None

    average_distance = _average_distance_to_polyline(boundary_points, border_points)
    if average_distance > 10.0 or average_distance < 0.3:
        return None

    boundary_length = polyline_length(boundary_points)
    border_length = polyline_length(border_points)
    if boundary_length <= 1e-6 or border_length <= 1e-6:
        return None
    if boundary_length > border_length * 3.0 or border_length > boundary_length * 3.0:
        return None

    return LaneletFeature(
        feature_id=int(line_string.id),
        left=boundary_points,
        right=border_points,
    )


def _best_matching_road_border_path(
    boundary_points: list[Vec3],
    road_border_graph: tuple[dict[int, Vec3], dict[int, list[tuple[int, float]]], list[int], object | None],
) -> list[Vec3] | None:
    positions, adjacency, node_ids, tree = road_border_graph
    start_candidates = _nearest_graph_nodes(boundary_points[0], positions, node_ids, tree)
    end_candidates = _nearest_graph_nodes(boundary_points[-1], positions, node_ids, tree)
    if not start_candidates or not end_candidates:
        return None

    boundary_length = polyline_length(boundary_points)
    best_score: float | None = None
    best_path: list[Vec3] | None = None
    end_ids = [node_id for _, node_id in end_candidates]
    for start_distance, start_id in start_candidates:
        distances, previous = _dijkstra_to_targets(start_id, end_ids, adjacency)
        for end_distance, end_id in end_candidates:
            if end_id not in distances:
                continue
            path_ids = _reconstruct_path(start_id, end_id, previous)
            path_points = [positions[node_id] for node_id in path_ids]
            path_length = polyline_length(path_points)
            average_distance = _average_distance_to_polyline(boundary_points, path_points)
            score = (
                start_distance
                + end_distance
                + average_distance
                + abs(boundary_length - path_length) * 0.15
            )
            if best_score is None or score < best_score:
                best_score = score
                best_path = path_points
    return best_path


def _nearest_graph_nodes(
    point: Vec3,
    positions: dict[int, Vec3],
    node_ids: list[int],
    tree,
    limit: int = 6,
) -> list[tuple[float, int]]:
    if not node_ids:
        return []
    if tree is not None:  # pragma: no branch - optional acceleration
        distance_values, index_values = tree.query((point.x, point.y), k=min(limit, len(node_ids)))
        if isinstance(index_values, int):
            return [(float(distance_values), node_ids[int(index_values)])]
        return [
            (float(distance), node_ids[int(index)])
            for distance, index in zip(distance_values, index_values)
        ]

    distances = [
        (point.distance_to(positions[node_id]), node_id)
        for node_id in node_ids
    ]
    distances.sort()
    return distances[:limit]


def _dijkstra_to_targets(
    start_id: int,
    target_ids: list[int],
    adjacency: dict[int, list[tuple[int, float]]],
) -> tuple[dict[int, float], dict[int, int]]:
    queue: list[tuple[float, int]] = [(0.0, start_id)]
    distances = {start_id: 0.0}
    previous: dict[int, int] = {}
    remaining = set(target_ids)
    while queue and remaining:
        distance, node_id = heappop(queue)
        if distance != distances.get(node_id):
            continue
        remaining.discard(node_id)
        for neighbor_id, edge_length in adjacency.get(node_id, []):
            next_distance = distance + edge_length
            if next_distance < distances.get(neighbor_id, float("inf")):
                distances[neighbor_id] = next_distance
                previous[neighbor_id] = node_id
                heappush(queue, (next_distance, neighbor_id))
    return distances, previous


def _reconstruct_path(start_id: int, end_id: int, previous: dict[int, int]) -> list[int]:
    path = [end_id]
    while path[-1] != start_id:
        parent_id = previous.get(path[-1])
        if parent_id is None:
            return []
        path.append(parent_id)
    path.reverse()
    return path


def _average_distance_to_polyline(points: list[Vec3], candidate: list[Vec3]) -> float:
    if not points or not candidate:
        return float("inf")
    distance_sum = 0.0
    for point in points:
        distance_sum += min(point.distance_to(candidate_point) for candidate_point in candidate)
    return distance_sum / len(points)


def _attr_str(obj: object, key: str) -> str | None:
    attributes = getattr(obj, "attributes", {})
    try:
        if key in attributes:
            return str(attributes[key])
    except Exception:
        return None
    return None
