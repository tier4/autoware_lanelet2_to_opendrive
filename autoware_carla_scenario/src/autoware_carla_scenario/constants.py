"""Package-wide constants for autoware_carla_scenario."""

from .entity_role import EntityRole

# CARLA role_name attribute value assigned to the ego vehicle actor.
EGO_ROLE_NAME: EntityRole = EntityRole.ego()

# Default port for the CARLA TrafficManager.
# The CARLA default (8000) often conflicts with other services (e.g. VS Code),
# so we use a different port to avoid ``RuntimeError: std::exception``.
# This value is used as a fallback when no Hydra config is available.
DEFAULT_TM_PORT: int = 8100
