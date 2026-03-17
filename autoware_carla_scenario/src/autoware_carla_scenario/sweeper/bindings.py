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

    def resolve(
        self, lanelet_id: int, lanelet_map: Any, routing_graph: Any | None = None
    ) -> float:
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

    @staticmethod
    def _project_stop_line(ls: Any, lanelet: Any) -> float:
        """Project stop line centroid onto lanelet centerline, return arc-length."""
        points = list(ls)
        if not points:
            raise ValueError("Stop line linestring has no points.")
        cx = sum(p.x for p in points) / len(points)
        cy = sum(p.y for p in points) / len(points)
        pt = lanelet2.core.BasicPoint2d(cx, cy)
        centerline_2d = lanelet2.geometry.to2D(lanelet.centerline)
        arc = lanelet2.geometry.toArcCoordinates(centerline_2d, pt)
        return arc.length

    def resolve(
        self, lanelet_id: int, lanelet_map: Any, routing_graph: Any | None = None
    ) -> float:
        """Return the arc-length position *offset* metres before the stop line.

        If the spawn lanelet itself has no stop line, following lanelets are
        searched via the routing graph.  The cross-lanelet arc-length is then
        ``spawn_lanelet_length + stop_s_on_following - offset``, clamped so
        the vehicle stays within the spawn lanelet.
        """
        lanelet = lanelet_map.laneletLayer[lanelet_id]

        # Collect stop lines from this lanelet's regulatory elements.
        seen_ids: set[int] = set()
        stop_lines = _collect_stop_lines_from_reg_elems(
            lanelet.regulatoryElements, seen_ids
        )

        if stop_lines:
            # Stop line found on the spawn lanelet itself.
            stop_s = self._project_stop_line(stop_lines[0], lanelet)
            result = max(0.0, stop_s - self.offset)
            logger.debug(
                "Lanelet %d: stop_line_s=%.2f, offset=%.2f -> spawn_s=%.2f",
                lanelet_id,
                stop_s,
                self.offset,
                result,
            )
            return result

        # -- Fallback: search following lanelets for a stop line. --
        if routing_graph is None:
            from .constraints import create_routing_graph

            routing_graph = create_routing_graph(lanelet_map)
        following = routing_graph.following(lanelet)
        spawn_lanelet_length = lanelet2.geometry.length2d(lanelet)

        for fll in following:
            fll_stop_lines = _collect_stop_lines_from_reg_elems(
                fll.regulatoryElements, seen_ids
            )
            if not fll_stop_lines:
                continue

            stop_s_on_following = self._project_stop_line(fll_stop_lines[0], fll)
            total_arc = spawn_lanelet_length + stop_s_on_following
            result = max(0.0, min(total_arc - self.offset, spawn_lanelet_length))
            logger.debug(
                "Lanelet %d: stop line on following lanelet %d "
                "(stop_s_following=%.2f, spawn_len=%.2f, total_arc=%.2f, "
                "offset=%.2f) -> spawn_s=%.2f",
                lanelet_id,
                fll.id,
                stop_s_on_following,
                spawn_lanelet_length,
                total_arc,
                self.offset,
                result,
            )
            return result

        raise ValueError(
            f"Lanelet {lanelet_id} and its following lanelets have no stop line; "
            "cannot resolve stop_line_offset binding."
        )


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
