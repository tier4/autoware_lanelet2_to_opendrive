"""Pluggable registry for scenarios and their Hydra config directories.

This module is the extension point that lets scenarios live **outside** the
``autoware_carla_scenario`` package.  An external "scenario package" only needs
to:

1. Define its scenario class (a :class:`BaseScenario` subclass) and a config
   dataclass.
2. Ship its concrete YAML configs under a ``conf/`` directory.
3. Register both at import time::

       from pathlib import Path
       from autoware_carla_scenario import register_conf_dir, register_scenario

       from .my_scenario import MyScenario
       from .configs import MyScenarioConfig


       def register() -> None:
           register_scenario("my_scenario", MyScenario, MyScenarioConfig)
           register_conf_dir(Path(__file__).parent / "conf")

4. Advertise ``register`` via a ``autoware_carla_scenario.scenarios`` entry
   point in ``pyproject.toml`` so the ``scenario`` CLI discovers it
   automatically::

       [project.entry-points."autoware_carla_scenario.scenarios"]
       my_scenario = "my_scenario_package:register"

The module is deliberately free of heavy imports (no CARLA, no lanelet2) so it
can be imported cheaply from anywhere -- the scenario/config classes are only
referenced lazily inside the builder closures.
"""

from __future__ import annotations

import logging
from pathlib import Path
from typing import TYPE_CHECKING, Any, Callable

if TYPE_CHECKING:
    from .coordinate import GroundProjectionConfig, Lanelet2Pose
    from .scenario_base import BaseScenario, EgoConfig
    from omegaconf import DictConfig

logger = logging.getLogger(__name__)

__all__ = [
    "ScenarioBuilder",
    "BuildScenarioFn",
    "register_scenario",
    "register_scenario_builder",
    "unregister_scenario",
    "get_scenario_registry",
    "get_scenario_builder",
    "register_conf_dir",
    "unregister_conf_dir",
    "get_conf_dirs",
    "load_scenario_plugins",
    "SCENARIO_ENTRY_POINT_GROUP",
]

#: Entry-point group that external scenario packages advertise in their
#: ``pyproject.toml`` to be auto-discovered by :func:`load_scenario_plugins`.
SCENARIO_ENTRY_POINT_GROUP = "autoware_carla_scenario.scenarios"

#: Callable that creates a :class:`BaseScenario` from its decomposed parts.
#: Signature: ``(ego, scenario_dict, spawn_pose, ground_projection) -> scenario``
ScenarioBuilder = Callable[
    ["EgoConfig", dict[str, Any], "Lanelet2Pose", "GroundProjectionConfig"],
    "BaseScenario",
]

#: Callable that replaces the entire ``build_scenario`` logic.
#: Signature: ``(cfg) -> (ego, scenario)``
BuildScenarioFn = Callable[["DictConfig"], "tuple[EgoConfig, BaseScenario]"]


# ---------------------------------------------------------------------------
# Scenario registry
# ---------------------------------------------------------------------------

_SCENARIO_REGISTRY: dict[str, ScenarioBuilder] = {}


def register_scenario(
    name: str,
    scenario_cls: type[BaseScenario],
    config_cls: type,
) -> None:
    """Register a scenario class and its config class under *name*.

    This creates a standard builder that follows the convention used by all
    built-in scenarios::

        scenario_cls(ego, config=config_cls(**scenario_dict),
                     spawn_pose=spawn_pose, ground_projection=ground_projection)

    Downstream projects can call this at import time to make their custom
    scenarios available to the CLI runner::

        from autoware_carla_scenario import register_scenario
        register_scenario("my_scenario", MyScenario, MyScenarioConfig)
    """

    def _builder(
        ego: EgoConfig,
        scenario_dict: dict[str, Any],
        spawn_pose: Lanelet2Pose,
        ground_projection: GroundProjectionConfig,
    ) -> BaseScenario:
        config = config_cls(**scenario_dict)
        return scenario_cls(  # type: ignore[call-arg]
            ego,
            config=config,
            spawn_pose=spawn_pose,
            ground_projection=ground_projection,
        )

    _SCENARIO_REGISTRY[name] = _builder


def register_scenario_builder(name: str, builder: ScenarioBuilder) -> None:
    """Register a custom builder function under *name*.

    Use this instead of :func:`register_scenario` when the scenario
    constructor does not follow the standard ``(ego, config=..., ...)``
    pattern and you need full control over instantiation.
    """
    _SCENARIO_REGISTRY[name] = builder


def get_scenario_registry() -> dict[str, ScenarioBuilder]:
    """Return a **copy** of the current scenario registry.

    Useful for introspection (e.g. listing available scenarios).
    """
    return dict(_SCENARIO_REGISTRY)


def get_scenario_builder(name: str) -> ScenarioBuilder | None:
    """Return the builder registered under *name*, or ``None`` if absent.

    Public lookup helper so callers don't reach into the internal registry
    dict.  Use :func:`get_scenario_registry` when you need the full mapping
    (e.g. to list the registered names).
    """
    return _SCENARIO_REGISTRY.get(name)


def unregister_scenario(name: str) -> None:
    """Remove the scenario registered under *name*, if present.

    Inverse of :func:`register_scenario` / :func:`register_scenario_builder`.
    Unknown names are ignored, so this is safe to call unconditionally (e.g.
    for test cleanup or to swap a scenario at runtime).
    """
    _SCENARIO_REGISTRY.pop(name, None)


# ---------------------------------------------------------------------------
# Config directory registry
# ---------------------------------------------------------------------------
#
# Every scenario package may contribute a ``conf/`` directory containing Hydra
# config groups (``scenario/``, ``map/``, ``ego/`` ...).  The CLI runner adds
# every registered directory to Hydra's search path so that
# ``scenario=<group>/<name>`` resolves regardless of which package owns it.

_CONF_DIRS: list[Path] = []


def register_conf_dir(path: str | Path) -> None:
    """Register a Hydra config directory contributed by a scenario package.

    The directory is expected to contain config groups such as
    ``scenario/``, ``map/``, ``ego/`` (mirroring the built-in layout).  It is
    appended to Hydra's search path so its config groups become selectable from
    the ``scenario`` CLI.  Registering the same directory twice is a no-op.
    """
    resolved = Path(path).resolve()
    if resolved not in _CONF_DIRS:
        _CONF_DIRS.append(resolved)


def unregister_conf_dir(path: str | Path) -> None:
    """Remove a previously registered config directory, if present.

    Inverse of :func:`register_conf_dir`.  Unregistered paths are ignored.
    """
    resolved = Path(path).resolve()
    try:
        _CONF_DIRS.remove(resolved)
    except ValueError:
        pass


def get_conf_dirs() -> list[Path]:
    """Return a copy of the ordered list of registered config directories.

    The first entry is the built-in directory (registered by the runner); any
    directories contributed by external packages follow in registration order.
    """
    return list(_CONF_DIRS)


# ---------------------------------------------------------------------------
# Entry-point based plugin discovery
# ---------------------------------------------------------------------------

_plugins_loaded = False


def load_scenario_plugins() -> None:
    """Discover and load external scenario packages via entry points.

    Every installed distribution that advertises an entry point in the
    :data:`SCENARIO_ENTRY_POINT_GROUP` group is imported and its target called
    with no arguments.  The target is expected to register scenarios and/or
    config directories (see the module docstring).

    Loading is idempotent: subsequent calls are no-ops within a single process.
    A failing plugin is logged and skipped so that one broken package cannot
    prevent the others (or the built-in scenarios) from running.
    """
    global _plugins_loaded
    if _plugins_loaded:
        return
    _plugins_loaded = True

    import importlib.metadata as importlib_metadata  # noqa: PLC0415

    # The selectable ``group=`` API exists on all supported interpreters
    # (requires-python >= 3.10).
    entry_points = importlib_metadata.entry_points(group=SCENARIO_ENTRY_POINT_GROUP)

    for entry_point in entry_points:
        try:
            register_fn = entry_point.load()
            register_fn()
            logger.info(
                "Loaded scenario plugin %r from %s",
                entry_point.name,
                getattr(entry_point, "value", entry_point),
            )
        except Exception:  # noqa: BLE001 -- one bad plugin must not kill the CLI
            logger.exception(
                "Failed to load scenario plugin %r; skipping.",
                entry_point.name,
            )
