"""Scaffolding for standalone scenario packages.

Public API::

    from autoware_carla_scenario.scaffold import create_scenario_package

    create_scenario_package("my_scenario_package", output_dir="~/pkgs")

Or via the ``scenario-new`` console script::

    scenario-new my_scenario_package --scenario reach_goal
"""

from __future__ import annotations

from .generator import (
    ScaffoldResult,
    create_scenario_package,
    main,
    resolve_names,
)

__all__ = ["ScaffoldResult", "create_scenario_package", "main", "resolve_names"]
