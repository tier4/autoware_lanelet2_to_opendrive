from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Callable

from .mesh_types import Vec3
from .osm_index import OsmIndex

try:
    from lanelet2.io import Origin, loadRobust
    from lanelet2.projection import UtmProjector
except ImportError:  # pragma: no cover - depends on user environment
    Origin = None
    UtmProjector = None
    loadRobust = None


class LaneletDependencyError(RuntimeError):
    """Raised when lanelet2 is not available."""


@dataclass
class LoadedLaneletMap:
    lanelet_map: object
    load_errors: list[str]
    coordinate_resolver: Callable[[object], Vec3]


def load_lanelet_map(path: str | Path, osm_index: OsmIndex) -> LoadedLaneletMap:
    if loadRobust is None or UtmProjector is None or Origin is None:
        raise LaneletDependencyError(
            "lanelet2 Python bindings are required. Install lanelet2 and retry."
        )

    osm_path = Path(path).resolve()
    origin = osm_index.first_geo_reference or (0.0, 0.0)
    projector = UtmProjector(Origin(origin[0], origin[1]))
    lanelet_map, errors = loadRobust(str(osm_path), projector)
    load_errors = [str(error) for error in errors]
    return LoadedLaneletMap(
        lanelet_map=lanelet_map,
        load_errors=load_errors,
        coordinate_resolver=_make_coordinate_resolver(osm_index),
    )


def _make_coordinate_resolver(osm_index: OsmIndex) -> Callable[[object], Vec3]:
    def resolve(point: object) -> Vec3:
        point_id = getattr(point, "id", None)
        if point_id is not None:
            osm_node = osm_index.nodes.get(int(point_id))
            if osm_node and osm_node.local_x is not None and osm_node.local_y is not None:
                z_value = osm_node.ele
                if z_value is None:
                    z_value = float(getattr(point, "z", 0.0))
                return Vec3(float(osm_node.local_x), float(osm_node.local_y), float(z_value))
        return Vec3(
            float(getattr(point, "x", 0.0)),
            float(getattr(point, "y", 0.0)),
            float(getattr(point, "z", 0.0)),
        )

    return resolve
