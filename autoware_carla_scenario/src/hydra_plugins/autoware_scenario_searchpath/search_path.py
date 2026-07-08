"""Add scenario-package config directories to Hydra's search path.

This module is discovered automatically by Hydra's plugin mechanism because it
lives under the ``hydra_plugins`` namespace package (PEP 420), the same way the
lanelet-constraint sweeper is discovered.

It bridges the framework's config-directory registry
(:func:`autoware_carla_scenario.get_conf_dirs`, populated by
``register_conf_dir`` — including by external scenario packages loaded via
:func:`autoware_carla_scenario.load_scenario_plugins`) into Hydra's search
path.  Because Hydra invokes :meth:`manipulate_search_path` from inside the
shared ``ConfigLoader``, a single plugin covers **both** the ``compose()`` API
path and the ``@hydra.main`` path, so the runner never has to thread search
paths through per-call overrides.

The import is intentionally lightweight: :mod:`autoware_carla_scenario.registry`
pulls in neither CARLA nor lanelet2, so importing this plugin during Hydra's
start-up plugin scan is cheap.
"""

from __future__ import annotations

from hydra.core.config_search_path import ConfigSearchPath
from hydra.plugins.search_path_plugin import SearchPathPlugin

from autoware_carla_scenario.registry import get_conf_dirs


class AutowareScenarioSearchPathPlugin(SearchPathPlugin):
    """Append every registered scenario config directory to the search path."""

    def manipulate_search_path(self, search_path: ConfigSearchPath) -> None:
        # ``manipulate_search_path`` runs at compose time, by which point the
        # runner has registered the built-in conf dir and loaded any external
        # scenario packages.  Re-adding the built-in dir (already the primary
        # config source) is inert -- Hydra tolerates overlapping search-path
        # entries -- so we simply append them all rather than tracking which is
        # primary.
        for conf_dir in get_conf_dirs():
            # ``get_conf_dirs`` returns resolved (absolute) paths, so ``as_uri``
            # is safe and yields a correct ``file:///...`` URI on every platform.
            search_path.append(
                provider="autoware-carla-scenario",
                path=conf_dir.as_uri(),
            )
