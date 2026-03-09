"""Traffic light utility functions for CARLA scenarios."""

from __future__ import annotations

from typing import TYPE_CHECKING, Optional, Tuple, Union

from ..coordinate.poses import AnyPose, CarlaWorldPose, Lanelet2Pose, OpenDrivePose
from ..coordinate.transform import to_carla_world

if TYPE_CHECKING:
    import carla


def _to_carla_location(pose: Union[AnyPose, "carla.Location"]) -> "carla.Location":
    """Convert an AnyPose or carla.Location to a carla.Location."""
    if isinstance(pose, (Lanelet2Pose, OpenDrivePose)):
        pose = to_carla_world(pose)
    if isinstance(pose, CarlaWorldPose):
        import carla  # noqa: PLC0415

        return carla.Location(x=pose.x, y=pose.y, z=pose.z)
    # Assume carla.Location
    return pose


def find_nearest_traffic_light(
    traffic_lights: list["carla.TrafficLight"],
    location: Union[AnyPose, "carla.Location"],
    max_distance: float = 150.0,
) -> Tuple[Optional["carla.TrafficLight"], float]:
    """Return the nearest traffic light to *location* within *max_distance*.

    Args:
        traffic_lights: List of CARLA traffic light actors to search.
        location: The reference position to measure distances from.
            Accepts any pose type (``Lanelet2Pose``, ``OpenDrivePose``,
            ``CarlaWorldPose``) or a raw ``carla.Location``.
        max_distance: Maximum search radius in metres.  Traffic lights
            farther than this are ignored.

    Returns:
        A ``(traffic_light, distance)`` tuple.  If no traffic light is found
        within *max_distance*, returns ``(None, float('inf'))``.
    """
    loc = _to_carla_location(location)
    nearest: Optional["carla.TrafficLight"] = None
    nearest_dist = float("inf")

    for tl in traffic_lights:
        dist: float = tl.get_transform().location.distance(loc)
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
