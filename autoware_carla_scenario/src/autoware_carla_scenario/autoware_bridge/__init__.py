"""autoware_bridge - minimal readiness contract with ``autoware_carla_interface``.

The Autoware side owns the whole startup sequence (localization init, routing,
engage); the framework only hands over the initial pose and goal and waits for a
single readiness flag.  See :class:`.base.AutowareBridge` for the rationale.

The framework core depends only on the abstract :class:`AutowareBridge`.  The
concrete gRPC transport hosts the ``AutowareBridge`` *server*; the
``autoware_carla_interface`` node is the *client* that dials it (splatsim-
consistent - see the topology note in
``proto/autoware_bridge/v0/autoware_bridge.proto``).  The transport and that wire
contract are implemented separately, so this package never imports ROS 2 / rclpy.

Usage::

    from autoware_carla_scenario.autoware_bridge import (
        AutowareBridge,
        AutowareBridgeConfig,
        BridgePose,
        FakeAutowareBridge,
    )
"""

from __future__ import annotations

from .base import (
    AutowareBridge,
    AutowareBridgeConfig,
    BridgePose,
    Quaternion,
    Vector3,
)
from .fake import FakeAutowareBridge

__all__ = [
    "AutowareBridge",
    "AutowareBridgeConfig",
    "BridgePose",
    "FakeAutowareBridge",
    "Quaternion",
    "Vector3",
]
