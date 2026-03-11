"""entity – vehicle entity types for CARLA scenarios.

Usage::

    from autoware_carla_scenario.entity import (
        EgoVehicle,
        SpawnPointIndex,
        SpawnTransform,
        VehicleEntity,
        VehicleEntityConfig,
    )

    # NPC vehicle at spawn-point index 5
    config = VehicleEntityConfig(
        role_name="npc_vehicle_1",
        spawn_location=SpawnPointIndex(5),
        vehicle_type="vehicle.mini.cooper",
    )
    npc = VehicleEntity(config)
    actor = npc.spawn(world)
    # Use BaseScenario.enable_autopilot_after() to enable autopilot safely
"""

from ._spawn import SpawnLocation, SpawnPointIndex, SpawnTransform
from .ego import EgoVehicle
from .vehicle_entity import VehicleEntity, VehicleEntityConfig

__all__ = [
    "EgoVehicle",
    "SpawnLocation",
    "SpawnPointIndex",
    "SpawnTransform",
    "VehicleEntity",
    "VehicleEntityConfig",
]
