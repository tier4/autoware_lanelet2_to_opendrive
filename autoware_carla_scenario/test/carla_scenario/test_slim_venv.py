"""Unit tests for the image-slimming helper shipped with the packing action.

The helper strips the bundled shared objects of third-party manylinux wheels,
which is only safe because it checks the result: GNU binutils up to 2.40 (the
version in Debian bookworm, the image's base) rewrites some objects -- scipy's
bundled OpenBLAS among them -- into a layout the kernel's ELF loader rejects
with "ELF load command address/offset not page-aligned".  These tests cover the
check that spots that, since a false negative silently ships a broken image.
"""

from __future__ import annotations

import importlib.util
import struct
from pathlib import Path
from types import ModuleType

import pytest

_SCRIPT = (
    Path(__file__).resolve().parents[3]
    / ".github"
    / "actions"
    / "pack-scenario-image"
    / "slim-venv.py"
)

_PT_LOAD = 1
_PT_NOTE = 4
_EHDR_SIZE = 64
_PHDR_SIZE = 56


@pytest.fixture(scope="module")
def slim() -> ModuleType:
    """Import the helper by path -- it ships with the action, not the package."""
    spec = importlib.util.spec_from_file_location("slim_venv", _SCRIPT)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _elf64(segments: list[tuple[int, int, int]]) -> bytes:
    """Build a minimal little-endian ELF64 object with the given program headers.

    Each segment is ``(p_type, p_offset, p_vaddr)``; nothing else is filled in,
    because nothing else is read.
    """
    header = bytearray(_EHDR_SIZE)
    header[0:4] = b"\x7fELF"
    header[4] = 2  # ELFCLASS64
    header[5] = 1  # ELFDATA2LSB
    struct.pack_into("<Q", header, 0x20, _EHDR_SIZE)  # e_phoff
    struct.pack_into("<HH", header, 0x36, _PHDR_SIZE, len(segments))

    body = bytearray()
    for p_type, p_offset, p_vaddr in segments:
        phdr = bytearray(_PHDR_SIZE)
        struct.pack_into("<I", phdr, 0, p_type)
        struct.pack_into("<QQ", phdr, 8, p_offset, p_vaddr)
        body += phdr
    return bytes(header + body)


def test_congruent_segments_are_loadable(slim: ModuleType, tmp_path: Path) -> None:
    """Offset and address agreeing modulo the page size is what the loader wants."""
    path = tmp_path / "good.so"
    path.write_bytes(_elf64([(_PT_LOAD, 0x1000, 0x1000), (_PT_LOAD, 0x2AE0, 0x3AE0)]))
    assert slim.is_loadable(path)


def test_misaligned_segment_is_rejected(slim: ModuleType, tmp_path: Path) -> None:
    """This is exactly what a bad strip produces, and what the loader refuses."""
    path = tmp_path / "bad.so"
    path.write_bytes(_elf64([(_PT_LOAD, 0x1000, 0x1000), (_PT_LOAD, 0x2AE0, 0x3B00)]))
    assert not slim.is_loadable(path)


def test_non_load_segments_are_ignored(slim: ModuleType, tmp_path: Path) -> None:
    """Only PT_LOAD is mapped, so only PT_LOAD has to be congruent."""
    path = tmp_path / "note.so"
    path.write_bytes(_elf64([(_PT_LOAD, 0x1000, 0x1000), (_PT_NOTE, 0x2AE0, 0x3B00)]))
    assert slim.is_loadable(path)


def test_non_elf_file_is_left_alone(slim: ModuleType, tmp_path: Path) -> None:
    """An unreadable layout means "do not touch", not "assume it is fine"."""
    path = tmp_path / "not-an-object.so"
    path.write_bytes(b"#!/bin/sh\necho hello\n")
    assert slim.load_segments(path) is None
    assert not slim.is_loadable(path)


def test_object_without_load_segments_is_left_alone(
    slim: ModuleType, tmp_path: Path
) -> None:
    """Nothing to verify means nothing to strip."""
    path = tmp_path / "empty.so"
    path.write_bytes(_elf64([(_PT_NOTE, 0x1000, 0x1000)]))
    assert not slim.is_loadable(path)
