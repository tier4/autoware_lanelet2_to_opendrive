from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from xml.etree import ElementTree


@dataclass(frozen=True)
class OsmNode:
    node_id: int
    lat: float | None
    lon: float | None
    local_x: float | None
    local_y: float | None
    ele: float | None


@dataclass(frozen=True)
class OsmIndex:
    path: Path
    nodes: dict[int, OsmNode]
    first_geo_reference: tuple[float, float] | None


def load_osm_index(path: str | Path) -> OsmIndex:
    osm_path = Path(path)
    tree = ElementTree.parse(osm_path)
    root = tree.getroot()
    nodes: dict[int, OsmNode] = {}
    first_geo_reference: tuple[float, float] | None = None

    for node_elem in root.findall("node"):
        node_id = int(node_elem.attrib["id"])
        lat = _as_float(node_elem.attrib.get("lat"))
        lon = _as_float(node_elem.attrib.get("lon"))
        local_x = None
        local_y = None
        ele = None
        for tag_elem in node_elem.findall("tag"):
            key = tag_elem.attrib.get("k")
            value = tag_elem.attrib.get("v")
            if key == "local_x":
                local_x = _as_float(value)
            elif key == "local_y":
                local_y = _as_float(value)
            elif key == "ele":
                ele = _as_float(value)
        nodes[node_id] = OsmNode(
            node_id=node_id,
            lat=lat,
            lon=lon,
            local_x=local_x,
            local_y=local_y,
            ele=ele,
        )
        if first_geo_reference is None and lat is not None and lon is not None:
            first_geo_reference = (lat, lon)

    return OsmIndex(path=osm_path.resolve(), nodes=nodes, first_geo_reference=first_geo_reference)


def _as_float(value: str | None) -> float | None:
    if value is None:
        return None
    try:
        return float(value)
    except ValueError:
        return None
