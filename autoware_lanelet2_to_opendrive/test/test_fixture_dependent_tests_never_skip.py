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
from typing import Iterator, List, Tuple

import pytest

#: Repository root, two levels above this test package.
_PROJECT_ROOT = Path(__file__).resolve().parents[2]

#: Every tree pytest collects from. ``scripts`` is in ``testpaths``
#: (``pyproject.toml``) and holds ``test_tdd.py``, so a guard added there
#: would otherwise be unchecked.
_TEST_ROOTS = (
    _PROJECT_ROOT / "autoware_lanelet2_to_opendrive" / "test",
    _PROJECT_ROOT / "scripts",
)

#: Both of pytest's default ``python_files`` patterns. ``rglob("test_*.py")``
#: alone misses ``*_test.py``, which pytest collects just the same.
_TEST_FILE_PATTERNS = ("test_*.py", "*_test.py")

#: Methods that ask the filesystem whether something is there. A skip whose
#: condition calls one of these is conditioned on execution history, whatever
#: the thing is called.
#:
#: Matching on the *condition's shape* rather than on the artifact's name is
#: the whole design. The previous version substring-matched a list of names
#: (``"nishishinjuku_carla.xodr"``, ``"NISHISHINJUKU_XODR"``, ``"_XODR_PATH"``)
#: against ``ast.dump`` of the entire call, and was wrong in both directions,
#: measured:
#:
#: * **it missed the construct it exists to forbid.** Renaming the constant to
#:   ``_REF_XODR`` and keeping ``skipif(not _REF_XODR.exists(), ...)`` passed;
#:   so did ``needs = pytest.mark.skipif`` with ``@needs(...)``, because the
#:   decorator's name no longer read ``skipif``.
#: * **it flagged a legitimate skip** whose ``reason=`` string merely
#:   mentioned the filename -- ``skipif(not CARLA_AVAILABLE, reason="needs
#:   carla to load nishishinjuku_carla.xodr")`` -- because ``ast.dump`` of a
#:   ``Call`` includes its keywords.
#:
#: Both are closed by looking only at positional arguments and only for a
#: filesystem probe: the receiver's name is irrelevant, and ``reason=`` is
#: never read.
_PROBE_METHODS = frozenset({"exists", "is_file", "is_dir", "isfile", "isdir"})

#: Mark and function names that take a test out of the count. ``xfail`` is
#: here because a *conditional* ``xfail`` reports ``xfailed`` instead of
#: ``passed`` and is excluded from the failure count just as effectively --
#: the unconditional ``xfail(strict=True)`` used to pin a known defect has no
#: condition and so is never matched.
_SKIPPING_NAMES = frozenset({"skip", "skipif", "xfail", "importorskip"})

#: A skip that is allowed to probe the filesystem, as ``(file name, reason)``.
#: Empty on purpose: there is no such case in the tree today. An entry here is
#: a deliberate, reviewed exception -- a fixture that cannot be committed, for
#: instance -- and adding one should be as visible as the construct it allows.
_ALLOWED: Tuple[Tuple[str, str], ...] = ()


def _probes_the_filesystem(node: ast.AST) -> bool:
    """True when *node* contains a call asking whether a path exists."""
    for inner in ast.walk(node):
        if not isinstance(inner, ast.Call):
            continue
        func = inner.func
        if isinstance(func, ast.Attribute) and func.attr in _PROBE_METHODS:
            return True
    return False


def _call_name(call: ast.Call) -> str:
    """The last name in a call's target, or ``""`` for anything else."""
    func = call.func
    if isinstance(func, ast.Attribute):
        return func.attr
    if isinstance(func, ast.Name):
        return func.id
    return ""


def _decorator_offences(tree: ast.AST) -> Iterator[Tuple[int, str]]:
    """Decorators whose *condition* probes the filesystem.

    Every decorator that is a call is examined, whatever it is named: that is
    what catches an aliased mark, which no name-based check can. Only
    positional arguments are read -- a mark's condition is positional and
    ``reason=`` is a keyword, so naming a file in the prose is not an offence.
    """
    for node in ast.walk(tree):
        if not isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef, ast.ClassDef)):
            continue
        for decorator in node.decorator_list:
            if not isinstance(decorator, ast.Call):
                continue
            if any(_probes_the_filesystem(arg) for arg in decorator.args):
                yield decorator.lineno, f"@{_call_name(decorator) or 'decorator'}"


def _guarded_call_offences(tree: ast.AST) -> Iterator[Tuple[int, str]]:
    """Bare skips reached only when a filesystem probe says so.

    A call like ``pytest.skip("...")`` carries no condition of its own -- the
    condition is the ``if`` around it:

        if not _XODR_PATH.is_file():
            pytest.skip("no artifact")

    So the decorator rule above cannot see it, and reading only the call's
    arguments would report nothing. This walks ``if`` statements whose test
    probes the filesystem and reports any skipping call inside. Found by the
    fault-injection case below, which the first version of this rewrite
    failed.
    """
    for node in ast.walk(tree):
        if not isinstance(node, ast.If) or not _probes_the_filesystem(node.test):
            continue
        for branch in (node.body, node.orelse):
            for statement in branch:
                for inner in ast.walk(statement):
                    if (
                        isinstance(inner, ast.Call)
                        and _call_name(inner) in _SKIPPING_NAMES
                    ):
                        yield inner.lineno, f"{_call_name(inner)}() under an if"


def _skips_conditioned_on_the_filesystem(tree: ast.AST) -> List[str]:
    """Describe every skip whose condition asks the filesystem a question."""
    offences = {
        (lineno, label)
        for lineno, label in list(_decorator_offences(tree))
        + list(_guarded_call_offences(tree))
    }
    # One line can carry only one offence worth reporting; keep the first
    # label so a decorator is not also listed under its own name.
    by_line: dict = {}
    for lineno, label in sorted(offences):
        by_line.setdefault(lineno, label)
    return [f"line {lineno}: {label}" for lineno, label in sorted(by_line.items())]


def _collected_test_files(root: Path) -> List[Path]:
    """Every file pytest would collect from *root*."""
    files: set = set()
    for pattern in _TEST_FILE_PATTERNS:
        files.update(root.rglob(pattern))
    return sorted(files)


@pytest.mark.parametrize("root", _TEST_ROOTS, ids=lambda p: p.name)
def test_no_test_skips_itself_on_whether_a_file_exists(root: Path) -> None:
    """A skip conditioned on the filesystem makes a pass depend on history.

    Request the ``nishishinjuku_xodr`` fixture instead. It converts the map
    once per invocation and raises if it cannot, which is what makes a test's
    presence in the count independent of what was run earlier.
    """
    assert root.is_dir(), f"test root moved: {root}"

    allowed = {name for name, _ in _ALLOWED}
    offenders: dict = {}
    for path in _collected_test_files(root):
        if path.name in allowed:
            continue
        guards = _skips_conditioned_on_the_filesystem(
            ast.parse(path.read_text(encoding="utf-8"))
        )
        if guards:
            offenders[str(path.relative_to(_PROJECT_ROOT))] = guards

    assert not offenders, (
        "these tests condition themselves on whether a file is present, so "
        "their presence in the count depends on what was run earlier:"
        + "".join(f" [{k}: {', '.join(v)}]" for k, v in sorted(offenders.items()))
        + " Request the `nishishinjuku_xodr` fixture instead."
    )


class TestTheGuardWorksInBothDirections:
    """Fault injection, because a check that matches nothing looks identical.

    Each case below was measured against the previous implementation: the
    first three passed it (evasions) and the last one failed it (a false
    positive). They are the reason this module was rewritten.
    """

    @pytest.mark.parametrize(
        "name, source",
        [
            (
                "the construct that was removed",
                "import pytest\n"
                "@pytest.mark.skipif(not _XODR_PATH.exists(), reason='x')\n"
                "class TestX:\n"
                "    def test_y(self): pass\n",
            ),
            (
                "the same thing with the constant renamed",
                "import pytest\n"
                "@pytest.mark.skipif(not _REF_XODR.exists(), reason='x')\n"
                "def test_y(): pass\n",
            ),
            (
                "an aliased mark, so the decorator is not named skipif",
                "import pytest\n"
                "needs = pytest.mark.skipif\n"
                "@needs(not _XODR_PATH.exists(), reason='x')\n"
                "def test_y(): pass\n",
            ),
            (
                "a conditional xfail, which leaves the pass count too",
                "import pytest\n"
                "@pytest.mark.xfail(not _XODR_PATH.exists(), reason='x', run=False)\n"
                "def test_y(): pass\n",
            ),
            (
                "a bare skip in a function body",
                "import pytest\n"
                "def test_y():\n"
                "    if not _XODR_PATH.is_file(): pytest.skip('x')\n",
            ),
        ],
    )
    def test_it_catches(self, name: str, source: str) -> None:
        assert _skips_conditioned_on_the_filesystem(ast.parse(source)), name

    @pytest.mark.parametrize(
        "name, source",
        [
            (
                "a skip on an unrelated condition",
                "import pytest\n"
                "@pytest.mark.skipif(not CARLA_AVAILABLE, reason='carla missing')\n"
                "def test_y(): pass\n",
            ),
            (
                "the same, whose reason merely names the artifact",
                "import pytest\n"
                "@pytest.mark.skipif(not CARLA_AVAILABLE,\n"
                "    reason='needs carla to load nishishinjuku_carla.xodr')\n"
                "def test_y(): pass\n",
            ),
            (
                "the unconditional xfail used to pin a known defect",
                "import pytest\n"
                "@pytest.mark.xfail(strict=True, reason='known gap')\n"
                "def test_y(): pass\n",
            ),
            (
                "parametrising over files, which is not a skip",
                "import pytest\n"
                "@pytest.mark.parametrize('p', sorted(DATA.glob('*.osm')))\n"
                "def test_y(p): pass\n",
            ),
        ],
    )
    def test_it_does_not_catch(self, name: str, source: str) -> None:
        assert not _skips_conditioned_on_the_filesystem(ast.parse(source)), name


def test_the_fixture_module_never_skips() -> None:
    """Nothing in ``conftest.py`` may skip, not just the fixture's own body.

    Scoped to the whole module rather than to ``nishishinjuku_xodr`` because
    a fixture can reach a skip through a helper defined beside it, which a
    walk of the fixture's body cannot see. ``importorskip`` is included: it
    raises ``Skipped`` too and is the most idiomatic way anyone would
    reintroduce one.

    Read statically rather than by invoking the fixture, because invoking it
    converts the map -- the property under test is what the source says.
    """
    conftest = Path(__file__).parent / "conftest.py"
    tree = ast.parse(conftest.read_text(encoding="utf-8"))

    skips = [
        f"line {node.lineno}: {_call_name(node)}()"
        for node in ast.walk(tree)
        if isinstance(node, ast.Call) and _call_name(node) in {"skip", "importorskip"}
    ]
    assert not skips, (
        "conftest.py skips at "
        + ", ".join(skips)
        + "; every test that requests a fixture reaching that takes itself "
        "out of the count. Fail instead."
    )
