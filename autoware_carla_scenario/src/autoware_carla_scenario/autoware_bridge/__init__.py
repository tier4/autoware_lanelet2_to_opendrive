"""autoware_bridge - initialization/monitoring contract with ``autoware_carla_interface``.

Distinct from the alpasim ``egodriver`` driving-policy contract in
:mod:`autoware_carla_scenario.driver`; see :class:`.base.AutowareBridge` for the
rationale.  The framework core depends only on the abstract
:class:`AutowareBridge`; the concrete gRPC transport (``GrpcAutowareBridge``) and
the wire contract in ``proto/autoware_bridge/v0/autoware_bridge.proto`` are
implemented separately so this package never imports ROS 2 / rclpy.

Usage::

    from autoware_carla_scenario.autoware_bridge import (
        AutowareBridge,
        AutowareBridgeConfig,
        AutowareInitSequence,
        BridgePose,
        FakeAutowareBridge,
    )
"""

from __future__ import annotations

from .base import (
    AutowareBridge,
    AutowareBridgeConfig,
    BridgePose,
    LocalizationState,
    OperationMode,
    Quaternion,
    RoutingState,
    Vector3,
    VehicleStatus,
)
from .fake import FakeAutowareBridge
from .init_sequence import AutowareInitSequence, InitState

__all__ = [
    "AutowareBridge",
    "AutowareBridgeConfig",
    "AutowareInitSequence",
    "BridgePose",
    "FakeAutowareBridge",
    "InitState",
    "LocalizationState",
    "OperationMode",
    "Quaternion",
    "RoutingState",
    "Vector3",
    "VehicleStatus",
]
