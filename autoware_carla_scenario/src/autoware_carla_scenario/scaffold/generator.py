"""Generate a standalone scenario package from Jinja2 templates.

Exposes both a programmatic function (:func:`create_scenario_package`) and a
CLI entry point (:func:`main`, wired to the ``scenario-new`` console script).

The generated package is a normal installable Python package that depends on
``autoware-carla-scenario`` and plugs into the framework via the
``autoware_carla_scenario.scenarios`` entry point -- see
:mod:`autoware_carla_scenario.registry` for how discovery works.

This module has no CARLA/lanelet2 dependency, so it can be run in any
environment (the *generated* scenario needs CARLA at run time, not at
generation time).
"""

from __future__ import annotations

import argparse
import keyword
import re
import sys
from pathlib import Path
from typing import NamedTuple

from jinja2 import Environment, FileSystemLoader, StrictUndefined


class ScaffoldResult(NamedTuple):
    """Result of :func:`create_scenario_package`."""

    #: The created package root (``output_dir / package_name``).
    root: Path
    #: The resolved template variables (see :func:`resolve_names`).
    names: dict[str, str]


#: Directory holding the ``*.jinja`` templates shipped with this package.
TEMPLATES_DIR = Path(__file__).resolve().parent / "templates"

#: Mapping of ``template name -> output path template``.  Output paths are
#: formatted with the resolved names (``package_name`` / ``scenario_name``).
_FILE_MANIFEST: dict[str, str] = {
    "pyproject.toml.jinja": "pyproject.toml",
    "README.md.jinja": "README.md",
    "package__init__.py.jinja": "src/{package_name}/__init__.py",
    "scenario.py.jinja": "src/{package_name}/{scenario_name}.py",
    "configs.py.jinja": "src/{package_name}/configs.py",
    "default.yaml.jinja": (
        "src/{package_name}/conf/scenario/{scenario_name}/default.yaml"
    ),
}


# ---------------------------------------------------------------------------
# Name handling
# ---------------------------------------------------------------------------


def _to_snake(raw: str) -> str:
    """Normalise *raw* to a ``snake_case`` Python identifier fragment."""
    # Split camelCase / PascalCase boundaries, then unify separators.
    spaced = re.sub(r"(?<=[a-z0-9])(?=[A-Z])", "_", raw)
    snake = re.sub(r"[\s\-.]+", "_", spaced).strip("_").lower()
    return re.sub(r"__+", "_", snake)


def _to_pascal(snake: str) -> str:
    """Convert a ``snake_case`` name to ``PascalCase``."""
    return "".join(part.capitalize() for part in snake.split("_") if part)


def _to_hyphen(snake: str) -> str:
    """Convert a ``snake_case`` name to a ``hyphen-case`` distribution name."""
    return snake.replace("_", "-")


def _validate_identifier(name: str, *, kind: str) -> str:
    """Return *name* if it is a valid, non-keyword Python identifier, else raise."""
    if not name:
        msg = f"{kind} is empty after normalisation; please pass an explicit value."
        raise ValueError(msg)
    if not name.isidentifier() or keyword.iskeyword(name):
        msg = f"{kind} {name!r} is not a valid Python identifier."
        raise ValueError(msg)
    return name


def resolve_names(
    name: str,
    scenario_name: str | None = None,
    description: str | None = None,
) -> dict[str, str]:
    """Resolve all template variables from the raw package *name*.

    Returns a dict with ``package_name`` (import/snake), ``distribution_name``
    (hyphen), ``scenario_name`` (snake), ``scenario_class`` /  ``config_class``
    (Pascal), and ``description``.
    """
    package_name = _validate_identifier(_to_snake(name), kind="package name")

    if scenario_name is None:
        # Default: strip a trailing package-ish suffix so
        # ``my_scenario_package`` yields the scenario ``my_scenario``.
        stripped = re.sub(r"_(package|pkg|scenarios?)$", "", package_name)
        scenario = stripped or package_name
    else:
        scenario = _to_snake(scenario_name)
    scenario = _validate_identifier(scenario, kind="scenario name")

    pascal = _to_pascal(scenario)
    # Avoid awkward doubling when the name already ends in "scenario".
    scenario_class = pascal if pascal.endswith("Scenario") else f"{pascal}Scenario"
    return {
        "package_name": package_name,
        "distribution_name": _to_hyphen(package_name),
        "scenario_name": scenario,
        "scenario_class": scenario_class,
        "config_class": f"{pascal}Config",
        "description": description
        or f"{pascal} scenario package for autoware_carla_scenario",
    }


# ---------------------------------------------------------------------------
# Generation
# ---------------------------------------------------------------------------


def _environment() -> Environment:
    """Build the Jinja2 environment used to render the templates."""
    return Environment(
        loader=FileSystemLoader(str(TEMPLATES_DIR)),
        autoescape=False,  # we render code/config, never HTML
        keep_trailing_newline=True,
        trim_blocks=True,
        lstrip_blocks=True,
        undefined=StrictUndefined,  # fail loudly on a missing variable
    )


def create_scenario_package(
    name: str,
    *,
    scenario_name: str | None = None,
    output_dir: str | Path = ".",
    description: str | None = None,
    force: bool = False,
) -> ScaffoldResult:
    """Render a new scenario package under *output_dir*.

    Parameters
    ----------
    name:
        Package name.  Accepts ``snake_case``, ``hyphen-case``, or ``PascalCase``
        -- it is normalised to a valid import name (e.g. ``my-pkg`` ->
        ``my_pkg``).
    scenario_name:
        Name of the scenario to scaffold.  Defaults to *name* with a trailing
        ``_package`` / ``_pkg`` / ``_scenario(s)`` suffix removed.
    output_dir:
        Parent directory in which the package directory is created.
    description:
        Optional package description.
    force:
        Overwrite existing files if the target directory already exists.

    Returns
    -------
    ScaffoldResult
        The created package ``root`` and the resolved ``names``.
    """
    names = resolve_names(name, scenario_name, description)
    root = Path(output_dir) / names["package_name"]

    if root.exists() and not force:
        msg = f"Target directory already exists: {root} (use force=True to overwrite)."
        raise FileExistsError(msg)

    env = _environment()
    for template_name, out_template in _FILE_MANIFEST.items():
        out_path = root / out_template.format(**names)
        out_path.parent.mkdir(parents=True, exist_ok=True)
        rendered = env.get_template(template_name).render(**names)
        out_path.write_text(rendered, encoding="utf-8")

    return ScaffoldResult(root=root, names=names)


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="scenario-new",
        description="Generate a standalone autoware_carla_scenario package.",
    )
    parser.add_argument(
        "name",
        help="Package name (snake_case, hyphen-case, or PascalCase).",
    )
    parser.add_argument(
        "-s",
        "--scenario",
        dest="scenario_name",
        default=None,
        help="Scenario name (default: derived from the package name).",
    )
    parser.add_argument(
        "-o",
        "--output-dir",
        default=".",
        help="Directory to create the package in (default: current directory).",
    )
    parser.add_argument(
        "-d",
        "--description",
        default=None,
        help="Package description.",
    )
    parser.add_argument(
        "-f",
        "--force",
        action="store_true",
        help="Overwrite files if the target directory already exists.",
    )
    return parser


def main(argv: list[str] | None = None) -> int:
    """CLI entry point for the ``scenario-new`` console script."""
    args = _build_parser().parse_args(argv)
    try:
        root, names = create_scenario_package(
            args.name,
            scenario_name=args.scenario_name,
            output_dir=args.output_dir,
            description=args.description,
            force=args.force,
        )
    except (ValueError, FileExistsError) as exc:
        print(f"Error: {exc}", file=sys.stderr)  # noqa: T201
        return 1

    print(f"Created scenario package at {root}")  # noqa: T201
    print("\nNext steps:")  # noqa: T201
    print(f"  uv pip install -e {root}")  # noqa: T201
    print(  # noqa: T201
        f"  uv run scenario scenario={names['scenario_name']}/default map=nishishinjuku"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
