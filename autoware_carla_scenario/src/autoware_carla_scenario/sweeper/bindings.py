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

        Walks the spawn lanelet and its following lanelets (BFS via routing
        graph) searching for the nearest stop line.  The result is
        ``accumulated + stop_s - offset``, clamped to
        ``[0, spawn_lanelet_length]`` so the vehicle stays within the spawn
        lanelet.

        The search terminates when the accumulated distance from the spawn
        lanelet end exceeds the offset (further stop lines would only clamp
        the result to ``spawn_lanelet_length``).
        """
        lanelet = lanelet_map.laneletLayer[lanelet_id]
        spawn_lanelet_length = lanelet2.geometry.length2d(lanelet)

        if routing_graph is None:
            from .constraints import create_routing_graph

            routing_graph = create_routing_graph(lanelet_map)

        # BFS over the lanelet chain: spawn -> following -> following -> ...
        # Each entry is (lanelet_object, arc_length_from_spawn_lanelet_start).
        candidates: list[tuple[Any, float]] = [(lanelet, 0.0)]
        visited: set[int] = {lanelet_id}
        seen_ids: set[int] = set()

        while candidates:
            next_candidates: list[tuple[Any, float]] = []
            for current, accumulated in candidates:
                stop_lines = _collect_stop_lines_from_reg_elems(
                    current.regulatoryElements, seen_ids
                )
                if stop_lines:
                    stop_s = self._project_stop_line(stop_lines[0], current)
                    total_arc = accumulated + stop_s
                    result = max(
                        0.0, min(total_arc - self.offset, spawn_lanelet_length)
                    )
                    logger.debug(
                        "Lanelet %d: stop line found on lanelet %d "
                        "(stop_s=%.2f, accumulated=%.2f, offset=%.2f) "
                        "-> spawn_s=%.2f",
                        lanelet_id,
                        current.id,
                        stop_s,
                        accumulated,
                        self.offset,
                        result,
                    )
                    return result

                new_accumulated = accumulated + lanelet2.geometry.length2d(current)

                # Stop expanding beyond offset from spawn lanelet end.
                if new_accumulated - spawn_lanelet_length > self.offset:
                    continue

                for fll in routing_graph.following(current):
                    if fll.id not in visited:
                        visited.add(fll.id)
                        next_candidates.append((fll, new_accumulated))

            candidates = next_candidates

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
