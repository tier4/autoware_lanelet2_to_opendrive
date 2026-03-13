"""Hydra-discoverable sweeper plugin.

Hydra requires that plugin classes live under the ``hydra_plugins`` namespace
package (``cls.__module__`` must start with ``hydra_plugins.``).  This thin
subclass satisfies that requirement while delegating all logic to the
implementation in :mod:`autoware_carla_scenario.sweeper`.
"""

from autoware_carla_scenario.sweeper.lanelet_constraint_sweeper import (
    LaneletConstraintSweeper as _LaneletConstraintSweeperBase,
)


class LaneletConstraintSweeper(_LaneletConstraintSweeperBase):
    """Lanelet-constraint sweeper visible to Hydra's plugin discovery."""
