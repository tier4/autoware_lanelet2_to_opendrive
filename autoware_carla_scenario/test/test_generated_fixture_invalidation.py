"""Fault-injection tests for the generated-XODR cache key.

``autoware_carla_scenario/test/conftest.py`` regenerates
``nishishinjuku_carla.xodr`` when the inputs that produce it change, keyed on
``_input_fingerprint()``. Before that key existed the fixture's only condition
was ``if _XODR_PATH.exists(): return`` -- a cache with no invalidation -- and a
run on a new base silently inspected an artifact built by an old one.

A cache key is worth exactly as much as the evidence that it actually changes
when it should. So every test here *injects* a specific change and asserts the
fingerprint moved, and each one is paired with a no-op control asserting the
fingerprint is stable when nothing changed. A test that only checked "the
fingerprint is a 64-character string" would pass against a constant.

The injections run against a synthetic tree in ``tmp_path``, not the real
checkout: mutating the converter's source directory would be visible to every
other ``-n`` worker in the same invocation.
"""

from __future__ import annotations

from pathlib import Path
from typing import Any

import pytest


def _conftest(request: pytest.FixtureRequest) -> Any:
    """Return the already-loaded root conftest module.

    Looked up through the plugin manager rather than re-imported: importing
    ``conftest.py`` by path a second time would create a distinct module
    object with its own constants, so the test would no longer be exercising
    the code the fixture actually runs.
    """
    for plugin in request.config.pluginmanager.get_plugins():
        if hasattr(plugin, "_input_fingerprint") and hasattr(plugin, "_STAMP_PATH"):
            return plugin
    raise AssertionError(
        "root conftest with _input_fingerprint not found among loaded plugins"
    )


@pytest.fixture
def synthetic_inputs(tmp_path: Path) -> dict[str, Any]:
    """A miniature stand-in for (source OSM, converter source tree, argv)."""
    osm = tmp_path / "map.osm"
    osm.write_bytes(b"<osm/>")

    src = tmp_path / "src"
    (src / "conf" / "map").mkdir(parents=True)
    (src / "main.py").write_text("x = 1\n", encoding="utf-8")
    (src / "geometry.py").write_text("y = 2\n", encoding="utf-8")
    (src / "conf" / "map" / "nishishinjuku.yaml").write_text(
        "parampoly3:\n  default_segment_length: 3.0\n", encoding="utf-8"
    )
    # Not a fingerprinted suffix, and inside a directory that is skipped.
    (src / "notes.txt").write_text("ignored\n", encoding="utf-8")
    (src / "__pycache__").mkdir()
    (src / "__pycache__" / "main.py").write_text("stale bytecode stand-in\n")

    return {"osm_path": osm, "src_root": src, "argv": ("convert", "target=carla")}


def _fp(request: pytest.FixtureRequest, inputs: dict[str, Any]) -> str:
    return _conftest(request)._input_fingerprint(**inputs)


def test_identical_inputs_give_identical_fingerprint(
    request: pytest.FixtureRequest, synthetic_inputs: dict[str, Any]
) -> None:
    """No-op control: without a change the key must not move.

    This is what makes every other assertion in this module meaningful. If the
    fingerprint were time- or order-dependent, it would invalidate constantly
    and the "it changed" assertions below would pass for the wrong reason.
    """
    assert _fp(request, synthetic_inputs) == _fp(request, synthetic_inputs)


def test_changing_converter_source_changes_fingerprint(
    request: pytest.FixtureRequest, synthetic_inputs: dict[str, Any]
) -> None:
    before = _fp(request, synthetic_inputs)
    target = synthetic_inputs["src_root"] / "geometry.py"
    original = target.read_text(encoding="utf-8")

    target.write_text("y = 3\n", encoding="utf-8")
    assert _fp(request, synthetic_inputs) != before

    # Restoring must restore the key exactly -- otherwise the mechanism would
    # force a reconversion on every run and nobody would keep it.
    target.write_text(original, encoding="utf-8")
    assert _fp(request, synthetic_inputs) == before


def test_changing_hydra_config_changes_fingerprint(
    request: pytest.FixtureRequest, synthetic_inputs: dict[str, Any]
) -> None:
    """The YAML under ``conf/`` decides the emitted geometry, so it counts."""
    before = _fp(request, synthetic_inputs)
    config = synthetic_inputs["src_root"] / "conf" / "map" / "nishishinjuku.yaml"
    config.write_text("parampoly3:\n  default_segment_length: 1.0\n", encoding="utf-8")
    assert _fp(request, synthetic_inputs) != before


def test_changing_source_osm_changes_fingerprint(
    request: pytest.FixtureRequest, synthetic_inputs: dict[str, Any]
) -> None:
    before = _fp(request, synthetic_inputs)
    synthetic_inputs["osm_path"].write_bytes(b"<osm><node/></osm>")
    assert _fp(request, synthetic_inputs) != before


def test_adding_a_source_file_changes_fingerprint(
    request: pytest.FixtureRequest, synthetic_inputs: dict[str, Any]
) -> None:
    """Paths are hashed, not just contents, so a new module invalidates."""
    before = _fp(request, synthetic_inputs)
    (synthetic_inputs["src_root"] / "new_module.py").write_text("", encoding="utf-8")
    assert _fp(request, synthetic_inputs) != before


def test_deleting_a_source_file_changes_fingerprint(
    request: pytest.FixtureRequest, synthetic_inputs: dict[str, Any]
) -> None:
    before = _fp(request, synthetic_inputs)
    (synthetic_inputs["src_root"] / "geometry.py").unlink()
    assert _fp(request, synthetic_inputs) != before


def test_renaming_a_source_file_changes_fingerprint(
    request: pytest.FixtureRequest, synthetic_inputs: dict[str, Any]
) -> None:
    """A pure rename leaves the multiset of contents identical.

    Hashing contents alone would miss it; the path is folded in for this case.
    """
    before = _fp(request, synthetic_inputs)
    src = synthetic_inputs["src_root"]
    (src / "geometry.py").rename(src / "geometry_renamed.py")
    assert _fp(request, synthetic_inputs) != before


def test_changing_convert_argv_changes_fingerprint(
    request: pytest.FixtureRequest, synthetic_inputs: dict[str, Any]
) -> None:
    before = _fp(request, synthetic_inputs)
    synthetic_inputs["argv"] = ("convert", "target=default")
    assert _fp(request, synthetic_inputs) != before


def test_unfingerprinted_files_do_not_invalidate(
    request: pytest.FixtureRequest, synthetic_inputs: dict[str, Any]
) -> None:
    """Second no-op control: the filter is a filter, not a blanket.

    ``__pycache__`` churns on every interpreter run and a stray ``.txt`` does
    not change the emitted map. If either invalidated, the fixture would
    reconvert constantly and the cost would get the mechanism deleted.
    """
    before = _fp(request, synthetic_inputs)
    src = synthetic_inputs["src_root"]
    (src / "notes.txt").write_text("edited\n", encoding="utf-8")
    (src / "__pycache__" / "main.py").write_text("different bytecode\n")
    (src / "__pycache__" / "extra.py").write_text("more\n")
    assert _fp(request, synthetic_inputs) == before


def test_stamp_is_current_rejects_missing_and_mismatched(
    request: pytest.FixtureRequest, tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    """A missing stamp is stale, so a pre-existing artifact is not trusted."""
    conftest = _conftest(request)
    stamp = tmp_path / "nishishinjuku_carla.xodr.inputs"
    monkeypatch.setattr(conftest, "_STAMP_PATH", stamp)

    assert conftest._stamp_is_current("abc") is False  # missing

    stamp.write_text("def", encoding="utf-8")
    assert conftest._stamp_is_current("abc") is False  # mismatched

    stamp.write_text("abc\n", encoding="utf-8")
    assert conftest._stamp_is_current("abc") is True  # trailing newline tolerated


def test_real_tree_fingerprint_is_stable_and_nonconstant(
    request: pytest.FixtureRequest,
) -> None:
    """Sanity check against the actual checkout, not the synthetic tree.

    Two calls must agree (no time or ordering dependence), and the result must
    differ from the fingerprint of an empty tree -- which is what would come
    back if ``rglob`` were silently matching nothing because the source root
    moved.
    """
    conftest = _conftest(request)
    first = conftest._input_fingerprint()
    assert first == conftest._input_fingerprint()
    assert len(first) == 64

    empty = conftest._input_fingerprint(src_root=conftest._PROJECT_ROOT / "nonexistent")
    assert first != empty
