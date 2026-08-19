"""Autoware ego vehicle — an ego vehicle not managed by TrafficManager.

This entity behaves identically to :class:`EgoVehicle` during spawn and
destroy, but signals to :class:`ScenarioRunner` that TrafficManager
autopilot must **not** be enabled on its actor.  Nothing drives the actor:
it is spawned and left for an external stack to control out of band.

For a closed loop against an external driving policy, use
:class:`~autoware_carla_scenario.entity.carla_driver_entity.CarlaDriverEntity`,
which drives the ego over the ``egodriver`` gRPC contract.  Autoware topic
I/O integration remains a separate, future addition.
"""

from __future__ import annotations

from .ego import EgoVehicle


class AutowareEntity(EgoVehicle):
    """Ego vehicle controlled by Autoware instead of TrafficManager.

    After spawning, the :class:`ScenarioRunner` reads
    :attr:`EgoVehicle.use_autopilot` and skips ``set_autopilot(True)``
    for this actor, leaving it free for external (Autoware) control.

    The lifecycle hooks inherited from :class:`EgoVehicle` stay no-ops, so the
    vehicle stands still unless something outside the scenario drives it.  See
    :class:`~autoware_carla_scenario.entity.carla_driver_entity.CarlaDriverEntity`
    for an ego that closes the loop itself.
    """

    use_autopilot: bool = False
