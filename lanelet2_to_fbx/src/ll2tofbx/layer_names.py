from __future__ import annotations


LAYER_NAMES = (
    "lanelet_road",
    "road_extention",
    "intersection_area",
    "hatched_area",
    "parking_lot",
    "shoulder",
    "road_border_wall",
    "road_marking",
)

OPTIONAL_LAYER_NAMES = frozenset({"road_extention"})

ROAD_SURFACE_LAYER_NAMES = frozenset(
    {
        "lanelet_road",
        "road_extention",
        "intersection_area",
        "hatched_area",
        "parking_lot",
        "shoulder",
    }
)
