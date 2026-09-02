"""Verify a freshly built scenario image from the inside.

Run with the image's own interpreter::

    docker run --rm --entrypoint python \
        -v "$PWD/.github/actions/pack-scenario-image/smoke-test.py:/tmp/smoke-test.py:ro" \
        my-scenario:carla0.10.0 /tmp/smoke-test.py 0.10.0

It asserts the properties the image exists to guarantee: the CARLA client is
exactly the version pinned at build time, the scenario package's entry point
registers its scenario, and Hydra can reach that package's config directory.
"""

from __future__ import annotations

import sys
from importlib.metadata import version


def main(argv: list[str]) -> int:
    """Check the pinned CARLA client and the registered scenarios."""
    if len(argv) != 2:
        print(f"usage: {argv[0]} <expected-carla-version>", file=sys.stderr)
        return 2
    expected = argv[1]

    installed = version("carla")
    if installed != expected:
        print(
            f"CARLA client is {installed}, but the image was built for {expected}",
            file=sys.stderr,
        )
        return 1

    # Imported lazily so a CARLA mismatch is reported before the heavier import.
    from autoware_carla_scenario.registry import (
        get_conf_dirs,
        get_scenario_registry,
        load_scenario_plugins,
    )

    load_scenario_plugins()
    scenarios = sorted(get_scenario_registry())
    if not scenarios:
        print(
            "No scenario registered: the package's "
            "'autoware_carla_scenario.scenarios' entry point did not load.",
            file=sys.stderr,
        )
        return 1

    # The plugin lives in the `hydra_plugins` namespace package, which is easy
    # to drop from a wheel by accident; without it Hydra never sees the
    # scenario package's conf dir and every `scenario=<name>/...` fails.
    from hydra.core.plugins import Plugins
    from hydra.plugins.search_path_plugin import SearchPathPlugin

    search_path_plugins = {
        plugin.__name__ for plugin in Plugins.instance().discover(SearchPathPlugin)
    }
    if "AutowareScenarioSearchPathPlugin" not in search_path_plugins:
        print(
            "Hydra did not discover AutowareScenarioSearchPathPlugin: the "
            "hydra_plugins namespace package is missing from the image.",
            file=sys.stderr,
        )
        return 1

    conf_dirs = get_conf_dirs()
    if not any(dir_.is_dir() and any(dir_.rglob("*.yaml")) for dir_ in conf_dirs):
        print(
            f"None of the registered config directories holds a YAML: {conf_dirs}",
            file=sys.stderr,
        )
        return 1

    print(f"CARLA client: {installed}")
    print(f"Registered scenarios: {', '.join(scenarios)}")
    print(f"Config directories: {', '.join(str(d) for d in conf_dirs)}")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
