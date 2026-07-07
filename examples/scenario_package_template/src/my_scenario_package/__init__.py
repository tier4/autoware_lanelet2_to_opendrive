"""Example standalone scenario package.

Installing this package and running the framework CLI (``uv run scenario``)
makes the ``reach_goal`` scenario available *without editing
``autoware_carla_scenario`` at all*::

    uv run scenario scenario=reach_goal/default map=nishishinjuku

The :func:`register` function below is the entry point advertised in
``pyproject.toml`` under the ``autoware_carla_scenario.scenarios`` group.  The
runner calls it once at start-up.
"""

from __future__ import annotations

from pathlib import Path

from autoware_carla_scenario import register_conf_dir, register_scenario

from .configs import ReachGoalConfig

__all__ = ["ReachGoalConfig", "register"]

#: Directory holding this package's Hydra config groups (scenario/, ...).
CONF_DIR = Path(__file__).resolve().parent / "conf"


def register() -> None:
    """Register this package's scenarios and config directory.

    Called automatically by the ``scenario`` CLI via the
    ``autoware_carla_scenario.scenarios`` entry point.  It can also be called
    explicitly from a custom runner.

    The scenario class is imported *inside* this function on purpose: importing
    it pulls in ``BaseScenario`` (and therefore CARLA), so deferring it keeps
    merely importing this package -- e.g. while enumerating entry points --
    free of the heavy CARLA import.
    """
    from .reach_goal import ReachGoalScenario

    register_scenario("reach_goal", ReachGoalScenario, ReachGoalConfig)
    register_conf_dir(CONF_DIR)
