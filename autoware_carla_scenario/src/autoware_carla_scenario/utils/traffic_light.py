"""Traffic light utility functions for CARLA scenarios."""

from __future__ import annotations

from typing import TYPE_CHECKING, Optional, Tuple, Union

from ..coordinate.map_manager import MapManager
from ..coordinate.poses import AnyPose
from ..coordinate.transform import to_carla_location

if TYPE_CHECKING:
    import carla


def find_nearest_traffic_light(
    world: "carla.World",
    location: Union[AnyPose, "carla.Location"],
    max_distance: float = 150.0,
) -> Tuple[Optional["carla.TrafficLight"], float]:
    """Return the nearest traffic light to *location* within *max_distance*.

    All ``traffic.traffic_light`` actors are retrieved from *world*
    automatically.

    Args:
        world: The CARLA world instance used to enumerate traffic lights.
        location: The reference position to measure distances from.
            Accepts any pose type (``Lanelet2Pose``, ``OpenDrivePose``,
            ``CarlaWorldPose``) or a raw ``carla.Location``.
        max_distance: Maximum search radius in metres.  Traffic lights
            farther than this are ignored.

    Returns:
        A ``(traffic_light, distance)`` tuple.  If no traffic light is found
        within *max_distance*, returns ``(None, float('inf'))``.
    """
    loc = to_carla_location(location)
    nearest: Optional["carla.TrafficLight"] = None
    nearest_dist = float("inf")

    for actor in world.get_actors():
        if not actor.type_id.startswith("traffic.traffic_light"):
            continue
        dist: float = actor.get_transform().location.distance(loc)
        if dist < nearest_dist and dist < max_distance:
            nearest = actor
            nearest_dist = dist

    return nearest, nearest_dist


def lanelet2_traffic_light_id_to_opendrive_controller_id(
    lanelet2_tl_id: int,
) -> Optional[int]:
    """Return the OpenDRIVE controller ID for a Lanelet2 traffic light ID.

    The mapping is derived from the ``<controller name="Controller_TL_{lanelet2_id}">``
    naming convention used during Lanelet2-to-OpenDRIVE conversion.  The XODR
    XML is read from the :class:`MapManager` singleton's loaded road network.

    Args:
        lanelet2_tl_id: Lanelet2 regulatory element ID of the traffic light.

    Returns:
        OpenDRIVE controller ID, or ``None`` if no matching controller is found.
    """
    mm = MapManager.get_instance()
    root = mm.road_network.root
    expected_name = f"Controller_TL_{lanelet2_tl_id}"

    for ctrl_elem in root.iter("controller"):
        if ctrl_elem.get("name") == expected_name:
            return int(ctrl_elem.get("id"))
    return None


def get_signal_ids_for_controller(controller_id: int) -> list[str]:
    """Return the signal IDs controlled by an OpenDRIVE controller.

    Parses ``<control signalId="...">`` children of the ``<controller>``
    element whose ``id`` matches *controller_id*.
    """
    mm = MapManager.get_instance()
    root = mm.road_network.root
    for ctrl_elem in root.iter("controller"):
        if ctrl_elem.get("id") == str(controller_id):
            return [
                c.get("signalId")
                for c in ctrl_elem.iter("control")
                if c.get("signalId") is not None
            ]
    return []
