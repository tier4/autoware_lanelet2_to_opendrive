"""Binding resolution for auto-deriving parameter values per lanelet.

Bindings are parsed from the ``sweep.bindings`` section of a scenario YAML.
Each binding computes a parameter value (e.g. ``ego.spawn_s``) as a function
of the matched lanelet (e.g. "15 m before the stop line").
"""

from __future__ import annotations

import logging
from dataclasses import dataclass
from typing import Any, Protocol

import lanelet2.core
import lanelet2.geometry

from ..utils.stop_line import _collect_stop_lines_from_reg_elems

logger = logging.getLogger(__name__)


# ---------------------------------------------------------------------------
# Binding protocol & implementations
# ---------------------------------------------------------------------------


class Binding(Protocol):
    """Interface that all parameter bindings must satisfy."""

    @property
    def target_key(self) -> str:
        """Hydra override key this binding writes to (e.g. ``ego.spawn_s``)."""
        ...

    def resolve(self, lanelet_id: int, lanelet_map: Any) -> float:
        """Compute the parameter value for the given lanelet."""
        ...


@dataclass
class StopLineOffsetBinding:
    """Compute ``spawn_s = stop_line_arc_length - offset``.

    The stop line arc-length is determined by projecting the stop line
    centroid onto the lanelet centerline, following the same pattern as
    :func:`~autoware_carla_scenario.coordinate.stop_line._linestring_to_pose`.
    """

    target_key: str
    offset: float = 15.0

    def resolve(self, lanelet_id: int, lanelet_map: Any) -> float:
        """Return the arc-length position *offset* metres before the stop line."""
        lanelet = lanelet_map.laneletLayer[lanelet_id]

        # Collect stop lines from this lanelet's regulatory elements.
        seen_ids: set[int] = set()
        stop_lines = _collect_stop_lines_from_reg_elems(
            lanelet.regulatoryElements, seen_ids
        )
        if not stop_lines:
            raise ValueError(
                f"Lanelet {lanelet_id} has no stop line; "
                "cannot resolve stop_line_offset binding."
            )

        # Use the first stop line (same convention as get_stop_line_poses).
        ls = stop_lines[0]
        points = list(ls)
        if not points:
            raise ValueError(
                f"Stop line linestring on lanelet {lanelet_id} has no points."
            )

        cx = sum(p.x for p in points) / len(points)
        cy = sum(p.y for p in points) / len(points)

        pt = lanelet2.core.BasicPoint2d(cx, cy)
        centerline_2d = lanelet2.geometry.to2D(lanelet.centerline)
        arc = lanelet2.geometry.toArcCoordinates(centerline_2d, pt)
        stop_s = arc.length

        result = max(0.0, stop_s - self.offset)
        logger.debug(
            "Lanelet %d: stop_line_s=%.2f, offset=%.2f -> spawn_s=%.2f",
            lanelet_id,
            stop_s,
            self.offset,
            result,
        )
        return result


# ---------------------------------------------------------------------------
# Parsing helpers
# ---------------------------------------------------------------------------

_BINDING_REGISTRY: dict[str, type[StopLineOffsetBinding]] = {
    "stop_line_offset": StopLineOffsetBinding,
}


def parse_binding(target_key: str, cfg: dict[str, Any]) -> Binding:
    """Instantiate a :class:`Binding` from a YAML mapping.

    Args:
        target_key: The Hydra override key (e.g. ``"ego.spawn_s"``).
        cfg: A dict with at least a ``type`` key and optional parameters.

    Returns:
        The corresponding binding instance.

    Raises:
        ValueError: If the ``type`` value is not recognised.
    """
    binding_type = cfg.get("type")
    if binding_type is None:
        raise ValueError(f"Binding config is missing 'type': {cfg}")
    cls = _BINDING_REGISTRY.get(binding_type)
    if cls is None:
        raise ValueError(
            f"Unknown binding type: {binding_type!r}. "
            f"Available: {list(_BINDING_REGISTRY)}"
        )
    kwargs = {k: v for k, v in cfg.items() if k != "type"}
    return cls(target_key=target_key, **kwargs)
