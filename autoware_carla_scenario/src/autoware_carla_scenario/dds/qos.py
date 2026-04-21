"""QoS profiles for Autoware DDS topics.

These profiles mirror the standard ROS 2 QoS settings used by
Autoware's ``simple_planning_simulator`` and related nodes.
"""

from cyclonedds.qos import Policy, Qos

#: Default QoS for most Autoware command / state topics.
DEFAULT_QOS = Qos(
    Policy.Durability.Volatile,
    Policy.Reliability.Reliable(max_blocking_time=1_000_000_000),
    Policy.History.KeepLast(1),
)

#: QoS for topics that use transient-local durability (e.g. ``vector_map``).
#: Late-joining readers will still receive the last published sample.
TRANSIENT_LOCAL_QOS = Qos(
    Policy.Durability.TransientLocal,
    Policy.Reliability.Reliable(max_blocking_time=1_000_000_000),
    Policy.History.KeepLast(1),
)
