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


@dataclass
class BindingResult:
    """Result of resolving a binding.

    Attributes:
        value: The computed parameter value (e.g. ``spawn_s``).
        lanelet_id_override: If set, the spawn lanelet should be changed
            to this lanelet because the original lanelet did not have
            enough distance to satisfy the offset.
    """

    value: float
    lanelet_id_override: int | None = None


class Binding(Protocol):
    """Interface that all parameter bindings must satisfy."""

    @property
    def target_key(self) -> str:
        """Hydra override key this binding writes to (e.g. ``ego.spawn_s``)."""
        ...

    def resolve(
        self, lanelet_id: int, lanelet_map: Any, routing_graph: Any | None = None
    ) -> BindingResult:
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

    def _walk_back_to_predecessor(
        self,
        shortfall: float,
        lanelet: Any,
        lanelet_map: Any,
        routing_graph: Any,
        original_lanelet_id: int,
    ) -> BindingResult:
        """Walk backwards through predecessor lanelets to satisfy the offset.

        When the distance from the spawn lanelet start to the stop line is
        less than the requested offset, we need to place the vehicle on an
        earlier (predecessor) lanelet.

        Args:
            shortfall: Remaining distance that could not be satisfied on the
                original spawn lanelet (always > 0).
            lanelet: The original spawn lanelet object.
            lanelet_map: The lanelet2 map.
            routing_graph: The routing graph for predecessor lookups.
            original_lanelet_id: The ID of the original spawn lanelet.

        Returns:
            A :class:`BindingResult` with the spawn position on a
            predecessor lanelet and the predecessor's ID as override.
        """
        current = lanelet
        remaining = shortfall

        while remaining > 0:
            predecessors = routing_graph.previous(current)
            if not predecessors:
                logger.warning(
                    "[%s] No more predecessors for lanelet %d; "
                    "%.2f m of offset could not be satisfied. "
                    "Placing at start of lanelet %d.",
                    self.target_key,
                    current.id,
                    remaining,
                    current.id,
                )
                result_lanelet = current
                result_s = 0.0
                break

            prev = predecessors[0]
            prev_length = lanelet2.geometry.length2d(prev)

            if prev_length >= remaining:
                result_s = prev_length - remaining
                result_lanelet = prev
                break

            remaining -= prev_length
            current = prev
        else:
            result_lanelet = current
            result_s = 0.0

        logger.info(
            "[%s] Offset not satisfiable on spawn lanelet %d "
            "(shortfall=%.2f m). Walked back to predecessor lanelet %d "
            "-> spawn_s=%.2f m (lanelet length=%.2f m).",
            self.target_key,
            original_lanelet_id,
            shortfall,
            result_lanelet.id,
            result_s,
            lanelet2.geometry.length2d(result_lanelet),
        )
        return BindingResult(
            value=result_s,
            lanelet_id_override=result_lanelet.id,
        )

    def resolve(
        self, lanelet_id: int, lanelet_map: Any, routing_graph: Any | None = None
    ) -> BindingResult:
        """Return the arc-length position *offset* metres before the stop line.

        Walks the spawn lanelet and its following lanelets (BFS via routing
        graph) searching for the nearest stop line.  The result is
        ``accumulated + stop_s - offset``.

        If the offset cannot be satisfied within the spawn lanelet (i.e. the
        result would be negative), the method walks backwards through
        predecessor lanelets to find a position that satisfies the full
        offset distance, returning a :class:`BindingResult` with the
        predecessor lanelet ID as ``lanelet_id_override``.

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
                    raw_result = total_arc - self.offset

                    if raw_result < 0:
                        logger.info(
                            "[%s] Stop line at %.2f m on lanelet %d "
                            "(accumulated=%.2f m from spawn lanelet %d). "
                            "%.2f m before stop line requires %.2f m "
                            "beyond spawn lanelet start; "
                            "walking back to predecessors.",
                            self.target_key,
                            stop_s,
                            current.id,
                            accumulated,
                            lanelet_id,
                            self.offset,
                            -raw_result,
                        )
                        return self._walk_back_to_predecessor(
                            shortfall=-raw_result,
                            lanelet=lanelet,
                            lanelet_map=lanelet_map,
                            routing_graph=routing_graph,
                            original_lanelet_id=lanelet_id,
                        )

                    result = min(raw_result, spawn_lanelet_length)
                    logger.info(
                        "[%s] Stop line at %.2f m on lanelet %d "
                        "(accumulated=%.2f m from spawn lanelet %d). "
                        "%.2f m before stop line -> spawn_s=%.2f m "
                        "on spawn lanelet.",
                        self.target_key,
                        stop_s,
                        current.id,
                        accumulated,
                        lanelet_id,
                        self.offset,
                        result,
                    )
                    return BindingResult(value=result)

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
