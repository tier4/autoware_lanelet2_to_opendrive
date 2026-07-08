"""IntersectionPassing scenario package for autoware_carla_scenario

Installing this package makes its transpiled scenario(s) available to
``uv run scenario`` via the ``autoware_carla_scenario.scenarios`` entry
point — no edits to ``autoware_carla_scenario`` required."""

from __future__ import annotations

from pathlib import Path

from autoware_carla_scenario import register_conf_dir, register_scenario

from .configs import IntersectionPassingConfig

__all__ = ["IntersectionPassingConfig", "register"]

#: Directory holding this package's Hydra config groups.
CONF_DIR = Path(__file__).resolve().parent / "conf"


def register() -> None:
    """Register this package's scenarios and config directory."""
    from .intersection_passing import (
        IntersectionPassingScenario,
    )

    register_scenario(
        "intersection_passing", IntersectionPassingScenario, IntersectionPassingConfig
    )
    register_conf_dir(CONF_DIR)
