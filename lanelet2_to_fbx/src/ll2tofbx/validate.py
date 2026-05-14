from __future__ import annotations

from collections import Counter
from collections import defaultdict

from .geometry import triangle_area_vertices
from .mesh_types import MeshLayer, TriangleMesh, ValidationReport, Vec3


def validate_layers(layers: list[MeshLayer], min_triangle_area: float) -> ValidationReport:
    report = ValidationReport()
    for layer in layers:
        _validate_layer(layer, min_triangle_area, report)
    return report


def mesh_boundary_edge_count(mesh: TriangleMesh) -> int:
    return _boundary_edge_count(mesh.faces)


def mesh_component_signed_volumes(mesh: TriangleMesh) -> list[float]:
    return [
        mesh_component_signed_volume(mesh, component)
        for component in _face_components(mesh.faces)
    ]


def mesh_face_components(mesh: TriangleMesh) -> list[list[int]]:
    return _face_components(mesh.faces)


def mesh_component_signed_volume(mesh: TriangleMesh, component: list[int]) -> float:
    return _signed_volume(mesh.vertices, [mesh.faces[index] for index in component])


def _validate_layer(layer: MeshLayer, min_triangle_area: float, report: ValidationReport) -> None:
    mesh = layer.mesh
    for index, vertex in enumerate(mesh.vertices):
        if vertex.is_finite():
            continue
        report.non_finite_vertices += 1
        report.errors.append(f"{layer.name}: vertex {index} is not finite.")

    for face_index, face in enumerate(mesh.faces):
        a, b, c = face
        area = triangle_area_vertices(mesh.vertices[a], mesh.vertices[b], mesh.vertices[c])
        if area >= min_triangle_area:
            continue
        report.degenerate_triangles += 1
        report.errors.append(f"{layer.name}: triangle {face_index} area {area:.12g} is degenerate.")

    if not mesh.faces:
        return

    if not layer.requires_closed_volume:
        return

    boundary_edges = mesh_boundary_edge_count(mesh)
    if boundary_edges:
        report.open_edges += boundary_edges
        report.errors.append(f"{layer.name}: mesh has {boundary_edges} open edges.")

    negative_components = 0
    for volume in mesh_component_signed_volumes(mesh):
        if volume <= 0.0:
            negative_components += 1
            report.warnings.append(
                f"{layer.name}: connected component signed volume is not positive ({volume:.12g})."
            )
    report.inward_faces += negative_components


def _boundary_edge_count(faces: list[tuple[int, int, int]]) -> int:
    counts: Counter[tuple[int, int]] = Counter()
    for a, b, c in faces:
        counts[tuple(sorted((a, b)))] += 1
        counts[tuple(sorted((b, c)))] += 1
        counts[tuple(sorted((c, a)))] += 1
    return sum(1 for count in counts.values() if count == 1)


def _signed_volume(vertices: list[Vec3], faces: list[tuple[int, int, int]]) -> float:
    if not faces:
        return 0.0

    reference = _volume_reference_point(vertices, faces)
    total = 0.0
    for a, b, c in faces:
        va = vertices[a] - reference
        vb = vertices[b] - reference
        vc = vertices[c] - reference
        total += va.dot(vb.cross(vc))
    return total / 6.0


def _volume_reference_point(
    vertices: list[Vec3],
    faces: list[tuple[int, int, int]],
) -> Vec3:
    vertex_indices = {vertex for face in faces for vertex in face}
    count = len(vertex_indices)
    if count == 0:
        return Vec3(0.0, 0.0, 0.0)

    sum_x = 0.0
    sum_y = 0.0
    sum_z = 0.0
    for index in vertex_indices:
        vertex = vertices[index]
        sum_x += vertex.x
        sum_y += vertex.y
        sum_z += vertex.z
    return Vec3(sum_x / count, sum_y / count, sum_z / count)


def _face_components(faces: list[tuple[int, int, int]]) -> list[list[int]]:
    vertex_to_faces: dict[int, list[int]] = defaultdict(list)
    for face_index, face in enumerate(faces):
        for vertex in face:
            vertex_to_faces[vertex].append(face_index)

    visited: set[int] = set()
    components: list[list[int]] = []
    for face_index in range(len(faces)):
        if face_index in visited:
            continue
        stack = [face_index]
        component: list[int] = []
        visited.add(face_index)
        while stack:
            current = stack.pop()
            component.append(current)
            for vertex in faces[current]:
                for neighbor in vertex_to_faces[vertex]:
                    if neighbor in visited:
                        continue
                    visited.add(neighbor)
                    stack.append(neighbor)
        components.append(component)
    return components
