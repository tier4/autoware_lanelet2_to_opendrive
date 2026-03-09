"""Traffic light utility functions for CARLA scenarios."""

from __future__ import annotations

from typing import TYPE_CHECKING, Optional, Tuple

if TYPE_CHECKING:
    import carla


def find_nearest_traffic_light(
    traffic_lights: list["carla.TrafficLight"],
    location: "carla.Location",
    max_distance: float = 150.0,
) -> Tuple[Optional["carla.TrafficLight"], float]:
    """Return the nearest traffic light to *location* within *max_distance*.

    Args:
        traffic_lights: List of CARLA traffic light actors to search.
        location: The reference position to measure distances from.
        max_distance: Maximum search radius in metres.  Traffic lights
            farther than this are ignored.

    Returns:
        A ``(traffic_light, distance)`` tuple.  If no traffic light is found
        within *max_distance*, returns ``(None, float('inf'))``.
    """
    nearest: Optional["carla.TrafficLight"] = None
    nearest_dist = float("inf")

    for tl in traffic_lights:
        dist: float = tl.get_transform().location.distance(location)
        if dist < nearest_dist and dist < max_distance:
            nearest = tl
            nearest_dist = dist

    return nearest, nearest_dist


def set_group_traffic_light_state(
    traffic_light: "carla.TrafficLight",
    state: "carla.TrafficLightState",
    *,
    freeze: bool = True,
) -> None:
    """Set every traffic light in the same group to *state*.

    This applies the state to all traffic lights returned by
    :meth:`carla.TrafficLight.get_group_traffic_lights`, which includes
    the given *traffic_light* itself.

    Args:
        traffic_light: Any member of the traffic light group.
        state: The desired :class:`carla.TrafficLightState`
            (e.g. ``carla.TrafficLightState.Green``).
        freeze: If ``True`` (default), freeze every light so that the
            CARLA traffic manager does not override the state.
    """
    for tl in traffic_light.get_group_traffic_lights():
        tl.set_state(state)
        tl.freeze(freeze)
