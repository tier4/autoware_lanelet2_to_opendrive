"""Declare and statically check OpenDRIVE usage in a scenario.

A map loaded Lanelet2-only (no ``.xodr`` and no CARLA-sourced OpenDRIVE; see
:class:`~autoware_carla_scenario.coordinate.map_manager.MapManager`) cannot serve
OpenDRIVE-based conditions or conversions.  Those only register at ``setup()``
time -- after CARLA is up -- so the framework scans the scenario's *source*
before running it and fails fast, rather than after spinning CARLA up.

Rather than maintain a hand-written list of OpenDRIVE-requiring names, each such
condition/function declares itself with the :func:`requires_opendrive` decorator.
The decorator is the single source of truth: it both documents the requirement at
the definition and registers the name the source scan looks for.  The
:class:`MapManager` accessors remain the hard guarantee for anything the scan
cannot see (fully dynamic construction).
"""

from __future__ import annotations

import ast
from typing import TypeVar

_T = TypeVar("_T")

#: Names declared with :func:`requires_opendrive`, populated at import time as the
#: decorated modules load.  The source scan matches against this registry, so
#: adding a decorator is all it takes to include a new symbol -- no list to keep
#: in sync.
_OPENDRIVE_SYMBOLS: set[str] = set()


def requires_opendrive(obj: _T) -> _T:
    """Mark a condition class or function as needing OpenDRIVE.

    Sets ``requires_opendrive = True`` on *obj* (self-documenting, and usable for
    a runtime check on registered conditions) and registers its name for the
    source scan (:func:`find_opendrive_symbols`).
    """
    obj.requires_opendrive = True  # type: ignore[attr-defined]
    _OPENDRIVE_SYMBOLS.add(obj.__name__)  # type: ignore[attr-defined]
    return obj


def opendrive_symbols() -> frozenset[str]:
    """Return the currently registered OpenDRIVE-requiring symbol names."""
    return frozenset(_OPENDRIVE_SYMBOLS)


def find_opendrive_symbols(source: str) -> set[str]:
    """Return the registered OpenDRIVE-requiring symbols referenced in *source*.

    Resolves ``import x as y`` / ``from m import x as y`` aliases so a renamed
    import is still matched by its original name.

    Args:
        source: Python source text of the scenario module.

    Returns:
        The subset of :func:`opendrive_symbols` referenced by name in *source*.
    """
    tree = ast.parse(source)

    alias_to_original: dict[str, str] = {}
    for node in ast.walk(tree):
        if isinstance(node, (ast.Import, ast.ImportFrom)):
            for alias in node.names:
                if alias.asname:
                    alias_to_original[alias.asname] = alias.name.split(".")[-1]

    symbols = _OPENDRIVE_SYMBOLS
    found: set[str] = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Name):
            name = node.id
        elif isinstance(node, ast.Attribute):
            name = node.attr
        else:
            continue
        original = alias_to_original.get(name, name)
        if original in symbols:
            found.add(original)
    return found


def check_scenario_source(source: str, scenario_name: str) -> None:
    """Raise if *source* uses OpenDRIVE symbols (for a Lanelet2-only map).

    Args:
        source: Python source text of the scenario module.
        scenario_name: Name used in the error message.

    Raises:
        ValueError: If the scenario references any OpenDRIVE-requiring symbol.
    """
    used = find_opendrive_symbols(source)
    if used:
        raise ValueError(
            f"Scenario {scenario_name!r} uses OpenDRIVE-based symbols "
            f"{sorted(used)} but the map is loaded Lanelet2-only (no OpenDRIVE). "
            "Rewrite these in Lanelet2/map coordinates (e.g. EntityInAreaCondition "
            "or WaypointCondition), or provide an OpenDRIVE (an .xodr file or a "
            "CARLA level whose map carries a geoReference)."
        )
