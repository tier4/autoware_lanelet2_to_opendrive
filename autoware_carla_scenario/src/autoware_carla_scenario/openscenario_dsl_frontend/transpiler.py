"""High-level entry points that tie the frontend pipeline together.

The pipeline is::

    .osc source
        -> parser.parse_*          (py-osc2 / ANTLR parse tree)
        -> extractor.extract       (syntax IR: OscProgram)
        -> translator.translate    (semantic plan: ScenarioPlan)
        -> codegen.generate_module (readable Python source)
"""

from __future__ import annotations

from pathlib import Path

from .ast_model import OscProgram
from .codegen import generate_module
from .extractor import extract_program
from .package_codegen import generate_package_files
from .parser import parse_osc_file, parse_osc_string
from .plan import ScenarioPlan
from .translator import translate_program


def parse_program_from_file(path: str | Path) -> OscProgram:
    """Parse and extract a ``.osc`` file into an :class:`OscProgram`."""
    return extract_program(parse_osc_file(path))


def parse_program_from_string(
    text: str, *, source_name: str = "<string>"
) -> OscProgram:
    """Parse and extract DSL source text into an :class:`OscProgram`."""
    return extract_program(parse_osc_string(text, source_name=source_name))


def plans_from_file(path: str | Path) -> list[ScenarioPlan]:
    """Parse, extract and translate a ``.osc`` file into scenario variants.

    Returns one :class:`ScenarioPlan` per ``one_of`` branch combination (just
    one for a scenario without ``one_of``).
    """
    return translate_program(parse_program_from_file(path))


def plans_from_string(
    text: str, *, source_name: str = "<string>"
) -> list[ScenarioPlan]:
    """Parse, extract and translate DSL source text into scenario variants."""
    return translate_program(parse_program_from_string(text, source_name=source_name))


def transpile_file(path: str | Path) -> str:
    """Transpile a ``.osc`` file into readable Python scenario source.

    Args:
        path: Path to the ``.osc`` source.

    Returns:
        The generated Python module as a string (one class per ``one_of``
        variant).
    """
    plans = plans_from_file(path)
    return generate_module(plans, source_name=str(path))


def transpile_string(text: str, *, source_name: str = "<string>") -> str:
    """Transpile DSL source text into readable Python scenario source."""
    plans = plans_from_string(text, source_name=source_name)
    return generate_module(plans, source_name=source_name)


def transpile_to_file(source: str | Path, destination: str | Path) -> Path:
    """Transpile *source* (``.osc``) and write the result to *destination*.

    Args:
        source: Path to the ``.osc`` source.
        destination: Path to the ``.py`` file to write.

    Returns:
        The *destination* path.
    """
    code = transpile_file(source)
    dst = Path(destination)
    dst.parent.mkdir(parents=True, exist_ok=True)
    dst.write_text(code, encoding="utf-8")
    return dst


def transpile_to_package(
    source: str | Path,
    output_dir: str | Path = ".",
    *,
    package_name: str | None = None,
    description: str | None = None,
    force: bool = False,
) -> Path:
    """Transpile *source* into an installable scenario package under *output_dir*.

    Args:
        source: Path to the ``.osc`` source.
        output_dir: Parent directory to create the package directory in.
        package_name: Desired package name; defaults to ``<scenario>_package``.
        description: Optional package description.
        force: Overwrite existing files if the target directory exists.

    Returns:
        The created package root directory.

    Raises:
        FileExistsError: If the target directory exists and *force* is ``False``.
    """
    plans = plans_from_file(source)
    files = generate_package_files(
        plans,
        source_name=str(source),
        package_name=package_name,
        description=description,
    )
    pkg = next(key.split("/")[1] for key in files if key.startswith("src/"))
    root = Path(output_dir) / pkg
    if root.exists() and not force:
        raise FileExistsError(
            f"target directory already exists: {root} (use force=True to overwrite)"
        )
    for rel, content in files.items():
        out_path = root / rel
        out_path.parent.mkdir(parents=True, exist_ok=True)
        out_path.write_text(content, encoding="utf-8")
    return root
