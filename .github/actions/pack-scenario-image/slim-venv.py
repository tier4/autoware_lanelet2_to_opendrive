#!/usr/bin/env python3
"""Shrink an installed virtualenv down to what running the code needs.

Two passes, both aimed at third-party manylinux wheels rather than at anything
this repository controls:

* delete what only a test runner, compiler or type checker reads -- byte-code
  caches, vendored test suites, C headers, Cython sources and type stubs;
* run ``strip --strip-unneeded`` over the bundled shared objects, which the
  wheels for scipy, numpy and OpenCV ship unstripped.

Stripping is verified rather than trusted.  GNU binutils up to and including
2.40 (the version in Debian bookworm) rewrites some shared objects into a
layout the kernel's ELF loader rejects with

    ELF load command address/offset not page-aligned

-- scipy's bundled OpenBLAS, whose LOAD segments leave large gaps between file
offset and virtual address, is one such object.  So every stripped file is
re-read and restored from a backup unless its LOAD segments still satisfy the
loader's requirement.  A newer binutils simply strips everything and restores
nothing.

Usage: slim-venv.py <venv-dir>
"""

from __future__ import annotations

import shutil
import struct
import subprocess
import sys
from pathlib import Path

#: Directories that only a test runner, compiler or type checker reads.
PRUNE_DIRS = frozenset({"__pycache__", "tests", "include"})
#: Files in the same category.
PRUNE_SUFFIXES = (".pyi", ".pyx", ".pxd", ".a")
#: x86-64 Linux maps segments at this granularity, whatever p_align claims.
PAGE_SIZE = 4096

_PT_LOAD = 1


def load_segments(path: Path) -> list[tuple[int, int]] | None:
    """Return ``(p_offset, p_vaddr)`` for each PT_LOAD segment.

    ``None`` means the file could not be read as a 64-bit ELF object, in which
    case the caller should leave it alone rather than guess.
    """
    try:
        data = path.read_bytes()
    except OSError:
        return None
    if len(data) < 64 or data[:4] != b"\x7fELF" or data[4] != 2:
        return None  # not ELF, or not ELFCLASS64

    endian = "<" if data[5] == 1 else ">"
    (e_phoff,) = struct.unpack_from(f"{endian}Q", data, 0x20)
    e_phentsize, e_phnum = struct.unpack_from(f"{endian}HH", data, 0x36)
    if e_phentsize < 56 or e_phoff + e_phnum * e_phentsize > len(data):
        return None

    segments = []
    for index in range(e_phnum):
        offset = e_phoff + index * e_phentsize
        p_type = struct.unpack_from(f"{endian}I", data, offset)[0]
        if p_type != _PT_LOAD:
            continue
        p_offset, p_vaddr = struct.unpack_from(f"{endian}QQ", data, offset + 8)
        segments.append((p_offset, p_vaddr))
    return segments


def is_loadable(path: Path) -> bool:
    """Whether the ELF loader will accept *path*'s segment layout."""
    segments = load_segments(path)
    if not segments:
        return False
    return all(
        p_offset % PAGE_SIZE == p_vaddr % PAGE_SIZE for p_offset, p_vaddr in segments
    )


def prune(site: Path) -> None:
    """Delete the build-time-only files under *site*."""
    for directory in sorted(
        (p for p in site.rglob("*") if p.is_dir() and p.name in PRUNE_DIRS),
        key=lambda p: len(p.parts),
        reverse=True,
    ):
        shutil.rmtree(directory, ignore_errors=True)
    # Materialised before deleting: unlinking while the walk is still open is
    # asking the directory iterator to cope with entries vanishing under it.
    for path in list(site.rglob("*")):
        if path.is_file() and path.name.endswith(PRUNE_SUFFIXES):
            path.unlink(missing_ok=True)


def shared_objects(site: Path) -> list[Path]:
    """Every shared object under *site*, in a stable order."""
    return sorted(
        path
        for path in site.rglob("*")
        if path.is_file() and not path.is_symlink() and ".so" in path.suffixes
    )


def strip_all(site: Path) -> list[Path]:
    """Strip every shared object, restoring any that strip made unloadable.

    Returns the objects that were restored.
    """
    restored = []
    for path in shared_objects(site):
        if not is_loadable(path):
            continue  # not ours to judge -- leave it exactly as it came
        backup = path.with_name(path.name + ".prestrip")
        shutil.copy2(path, backup)
        result = subprocess.run(
            ["strip", "--strip-unneeded", str(path)],
            capture_output=True,
            text=True,
        )
        if result.returncode != 0 or not is_loadable(path):
            backup.replace(path)
            restored.append(path)
        else:
            backup.unlink()
    return restored


def directory_size(path: Path) -> int:
    """Total size of every regular file under *path*."""
    return sum(p.stat().st_size for p in path.rglob("*") if p.is_file())


def main(argv: list[str]) -> int:
    """Slim the virtualenv named on the command line."""
    if len(argv) != 2:
        print(f"usage: {argv[0]} <venv-dir>", file=sys.stderr)
        return 2

    venv = Path(argv[1])
    (site,) = venv.glob("lib/python*/site-packages")

    before = directory_size(site)
    prune(site)
    after_prune = directory_size(site)
    restored = strip_all(site)
    after_strip = directory_size(site)

    mib = 1024 * 1024
    print(f"site-packages: {before / mib:.0f} MiB")
    print(f"  after pruning build-time files: {after_prune / mib:.0f} MiB")
    print(f"  after stripping shared objects: {after_strip / mib:.0f} MiB")
    for path in restored:
        print(f"  kept unstripped (strip produced an unloadable object): {path.name}")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
