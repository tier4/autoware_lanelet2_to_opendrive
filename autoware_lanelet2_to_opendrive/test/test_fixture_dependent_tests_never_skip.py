"""No test may make itself conditional on the generated XODR being present.

    Neither a pass nor a fail from a fixture-dependent test can be read
    without checking whether it ran.

That sentence came out of a real failure today, not a design review.  A run
that deleted ``test/data`` and re-ran the suite reported **1361 passed / 7
skipped / 0 failed** against a previous **1356 / 5 / 4** -- strictly
better-looking numbers.  Two of the five extra "passes" were tests that had
stopped running: the fixture was gone, their ``skipif(not path.exists())``
guards fired, and they left the count silently.  One of the two was the exact
test a live diagnosis then cited as agreeing with its conclusion.  Rebuilt and
re-run, the conclusion held; the evidence quoted for it had not executed.

The artifact has two quiet failure modes pointing in opposite directions:

===========================  =================================  ==============
state                        what happens                       how it reads
===========================  =================================  ==============
stale artifact present       a file from another base is read   false failure
deleted to avoid that        the dependent test skips           **false pass**
===========================  =================================  ==============

The ``nishishinjuku_xodr`` fixture closes the first, by converting into the
invocation's own temp directory so that no artifact from another base is ever
read.  This module closes the second, by refusing the construct that causes
it.  The contract is: **a test that needs the reference conversion runs, or it
fails.**  That fixture raises rather than skipping.

An import-time guard could not have worked even in principle -- marks are
evaluated during collection, before any fixture has run -- so the check below
is not merely stylistic.
"""

from __future__ import annotations

import ast
from pathlib import Path

import pytest

#: Repository root, two levels above this test package.
_PROJECT_ROOT = Path(__file__).resolve().parents[2]

#: The package's test tree. The original defect was cross-package -- the
#: guarded tests lived in one package and the fixture that would have
#: satisfied them in the other -- but the second package has since moved to
#: its own repository, so one root is now the whole of it. Written as a tuple
#: so adding a tree back is a one-line change.
_TEST_ROOTS = (_PROJECT_ROOT / "autoware_lanelet2_to_opendrive" / "test",)

#: Names that identify the generated artifact in a guard expression.
_ARTIFACT_TOKENS = ("nishishinjuku_carla.xodr", "NISHISHINJUKU_XODR", "_XODR_PATH")


def _skip_guards_mentioning_the_artifact(tree: ast.AST) -> list[str]:
    """Return a description of every ``skip``/``skipif`` naming the artifact.

    Matches decorators and bare calls alike, so that
    ``pytest.mark.skipif(not X.exists(), ...)`` and a ``pytest.skip(...)``
    inside a function body are both caught.
    """
    found: list[str] = []
    for node in ast.walk(tree):
        if not isinstance(node, ast.Call):
            continue
        func = node.func
        name = (
            func.attr
            if isinstance(func, ast.Attribute)
            else func.id
            if isinstance(func, ast.Name)
            else ""
        )
        if name not in ("skip", "skipif"):
            continue
        source = ast.dump(node)
        if any(token in source for token in _ARTIFACT_TOKENS):
            found.append(f"line {node.lineno}: {name}(...)")
    return found


@pytest.mark.parametrize("root", _TEST_ROOTS, ids=lambda p: p.parent.name)
def test_no_test_skips_itself_on_the_generated_xodr(root: Path) -> None:
    """No ``skip`` or ``skipif`` in either tree is conditioned on the artifact.

    Request the ``nishishinjuku_xodr`` fixture instead. It converts the map
    once per invocation and raises if it cannot, which is what makes the
    test's presence in the count independent of execution history.
    """
    assert root.is_dir(), f"test root moved: {root}"

    offenders: dict[str, list[str]] = {}
    for path in sorted(root.rglob("test_*.py")):
        if path.name == Path(__file__).name:
            continue  # this module names the tokens in order to forbid them
        guards = _skip_guards_mentioning_the_artifact(
            ast.parse(path.read_text(encoding="utf-8"))
        )
        if guards:
            offenders[str(path.relative_to(_PROJECT_ROOT))] = guards

    assert not offenders, (
        "these tests make themselves conditional on the generated XODR, so "
        "their presence in the count depends on what was run earlier:\n"
        + "\n".join(f"  {k}: {', '.join(v)}" for k, v in sorted(offenders.items()))
        + "\nRequest the `nishishinjuku_xodr` fixture instead."
    )


def test_the_guard_above_would_catch_the_construct_it_forbids() -> None:
    """Fault injection: the AST check fires on the exact removed pattern.

    Without this, ``test_no_test_skips_itself_on_the_generated_xodr`` passing
    would be indistinguishable from it matching nothing -- which is the same
    class of defect it exists to prevent.
    """
    removed = (
        "import pytest\n"
        "@pytest.mark.skipif(not _XODR_PATH.exists(), reason='not available')\n"
        "class TestX:\n"
        "    def test_y(self): pass\n"
    )
    assert _skip_guards_mentioning_the_artifact(ast.parse(removed))

    # No-op control: an unrelated skip must not be reported, or the check
    # would forbid every skip in the repository and get deleted.
    unrelated = (
        "import pytest\n"
        "@pytest.mark.skipif(not CARLA_AVAILABLE, reason='carla missing')\n"
        "def test_y(): pass\n"
    )
    assert not _skip_guards_mentioning_the_artifact(ast.parse(unrelated))


def test_the_fixture_itself_never_skips() -> None:
    """The fixture's own failure modes must fail, which is the whole point.

    ``pytest.skip`` raises ``Skipped``; a fixture that reaches for it takes
    every dependent test out of the count. Read statically rather than by
    invoking the fixture, because invoking it converts the map: the property
    under test is what the source says, and an edit that restores ``skip``
    here fails this test rather than quietly restoring the defect.
    """
    conftest = Path(__file__).parent / "conftest.py"
    tree = ast.parse(conftest.read_text(encoding="utf-8"))

    fixture = next(
        (
            node
            for node in tree.body
            if isinstance(node, ast.FunctionDef) and node.name == "nishishinjuku_xodr"
        ),
        None,
    )
    assert fixture is not None, f"nishishinjuku_xodr fixture not found in {conftest}"

    skips = [
        f"line {node.lineno}"
        for node in ast.walk(fixture)
        if isinstance(node, ast.Call)
        and isinstance(node.func, ast.Attribute)
        and node.func.attr == "skip"
    ]
    assert not skips, (
        "the nishishinjuku_xodr fixture skips at "
        + ", ".join(skips)
        + "; every test that requests it then leaves the count silently. "
        "Fail instead."
    )
