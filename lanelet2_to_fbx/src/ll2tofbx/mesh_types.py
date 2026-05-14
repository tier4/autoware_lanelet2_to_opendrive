from __future__ import annotations

from dataclasses import dataclass, field
from math import isfinite, sqrt


Face = tuple[int, int, int]


@dataclass(frozen=True)
class Vec3:
    x: float
    y: float
    z: float

    def __add__(self, other: "Vec3") -> "Vec3":
        return Vec3(self.x + other.x, self.y + other.y, self.z + other.z)

    def __sub__(self, other: "Vec3") -> "Vec3":
        return Vec3(self.x - other.x, self.y - other.y, self.z - other.z)

    def __mul__(self, scalar: float) -> "Vec3":
        return Vec3(self.x * scalar, self.y * scalar, self.z * scalar)

    def __truediv__(self, scalar: float) -> "Vec3":
        return Vec3(self.x / scalar, self.y / scalar, self.z / scalar)

    def dot(self, other: "Vec3") -> float:
        return self.x * other.x + self.y * other.y + self.z * other.z

    def cross(self, other: "Vec3") -> "Vec3":
        return Vec3(
            self.y * other.z - self.z * other.y,
            self.z * other.x - self.x * other.z,
            self.x * other.y - self.y * other.x,
        )

    def length(self) -> float:
        return sqrt(self.dot(self))

    def distance_to(self, other: "Vec3") -> float:
        return (self - other).length()

    def normalized(self) -> "Vec3":
        length = self.length()
        if length <= 1e-12:
            return Vec3(0.0, 0.0, 0.0)
        return self / length

    def is_finite(self) -> bool:
        return isfinite(self.x) and isfinite(self.y) and isfinite(self.z)

    def as_list(self) -> list[float]:
        return [self.x, self.y, self.z]


@dataclass
class TriangleMesh:
    vertices: list[Vec3] = field(default_factory=list)
    faces: list[Face] = field(default_factory=list)

    def add_vertex(self, vertex: Vec3) -> int:
        self.vertices.append(vertex)
        return len(self.vertices) - 1

    def add_face(self, face: Face) -> None:
        self.faces.append(face)

    def append_mesh(self, other: "TriangleMesh") -> None:
        base = len(self.vertices)
        self.vertices.extend(other.vertices)
        self.faces.extend((a + base, b + base, c + base) for a, b, c in other.faces)

    def translated(self, offset: Vec3) -> "TriangleMesh":
        return TriangleMesh(
            vertices=[vertex - offset for vertex in self.vertices],
            faces=list(self.faces),
        )

    def aabb(self) -> tuple[Vec3, Vec3] | None:
        if not self.vertices:
            return None
        min_x = max_x = self.vertices[0].x
        min_y = max_y = self.vertices[0].y
        min_z = max_z = self.vertices[0].z
        for vertex in self.vertices[1:]:
            min_x = min(min_x, vertex.x)
            min_y = min(min_y, vertex.y)
            min_z = min(min_z, vertex.z)
            max_x = max(max_x, vertex.x)
            max_y = max(max_y, vertex.y)
            max_z = max(max_z, vertex.z)
        return Vec3(min_x, min_y, min_z), Vec3(max_x, max_y, max_z)


@dataclass
class SurfacePatch:
    vertices: list[Vec3]
    faces: list[Face]
    boundary_loops: list[list[int]]


@dataclass(frozen=True)
class LaneletFeature:
    feature_id: int
    left: list[Vec3]
    right: list[Vec3]


@dataclass(frozen=True)
class PolygonFeature:
    feature_id: int
    points: list[Vec3]


@dataclass(frozen=True)
class PolylineFeature:
    feature_id: int
    points: list[Vec3]
    feature_type: str | None = None


@dataclass
class FeatureSet:
    lanelet_roads: list[LaneletFeature] = field(default_factory=list)
    intersection_areas: list[PolygonFeature] = field(default_factory=list)
    hatched_areas: list[PolygonFeature] = field(default_factory=list)
    parking_lots: list[PolygonFeature] = field(default_factory=list)
    road_borders: list[PolylineFeature] = field(default_factory=list)
    shoulders: list[LaneletFeature] = field(default_factory=list)
    road_markings: list[PolylineFeature] = field(default_factory=list)


@dataclass
class MeshLayer:
    name: str
    mesh: TriangleMesh
    source_feature_count: int
    top_vertices: list[Vec3] = field(default_factory=list)
    requires_closed_volume: bool = True

    def translated(self, offset: Vec3) -> "MeshLayer":
        return MeshLayer(
            name=self.name,
            mesh=self.mesh.translated(offset),
            source_feature_count=self.source_feature_count,
            top_vertices=[vertex - offset for vertex in self.top_vertices],
            requires_closed_volume=self.requires_closed_volume,
        )


@dataclass
class ValidationReport:
    errors: list[str] = field(default_factory=list)
    warnings: list[str] = field(default_factory=list)
    degenerate_triangles: int = 0
    open_edges: int = 0
    non_finite_vertices: int = 0
    inward_faces: int = 0

    def as_dict(self) -> dict[str, object]:
        return {
            "errors": list(self.errors),
            "warnings": list(self.warnings),
            "degenerate_triangles": self.degenerate_triangles,
            "open_edges": self.open_edges,
            "non_finite_vertices": self.non_finite_vertices,
            "inward_faces": self.inward_faces,
        }
