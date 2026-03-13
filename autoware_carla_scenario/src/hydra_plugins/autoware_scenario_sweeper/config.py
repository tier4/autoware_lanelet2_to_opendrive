"""Register the LaneletConstraintSweeper config with Hydra's ConfigStore.

This module is discovered automatically by Hydra's plugin mechanism because it
lives under the ``hydra_plugins`` namespace package (PEP 420).
"""

from dataclasses import dataclass

from hydra.core.config_store import ConfigStore


@dataclass
class LaneletConstraintSweeperConf:
    """Structured config for the lanelet-constraint sweeper."""

    _target_: str = (
        "hydra_plugins.autoware_scenario_sweeper.sweeper.LaneletConstraintSweeper"
    )


cs = ConfigStore.instance()
cs.store(
    group="hydra/sweeper",
    name="lanelet_constraint",
    node=LaneletConstraintSweeperConf,
    provider="autoware_scenario_sweeper",
)
