"""entity – vehicle entity types for CARLA scenarios.

Usage::

    from autoware_carla_scenario.entity import (
        EgoVehicle,
        VehicleEntity,
        VehicleEntityConfig,
    )

    # NPC vehicle
    config = VehicleEntityConfig(
        role_name="npc_vehicle_1",
        vehicle_type="vehicle.tesla.model3",
        spawn_index=5,
        autopilot=True,
    )
    npc = VehicleEntity(config)
    actor = npc.spawn(world)
"""

from .ego import EgoVehicle
from .vehicle_entity import VehicleEntity, VehicleEntityConfig

__all__ = [
    "EgoVehicle",
    "VehicleEntity",
    "VehicleEntityConfig",
]
