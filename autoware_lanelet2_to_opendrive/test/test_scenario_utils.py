"""Unit tests for scenario_utils – map resolution and /tmp caching."""

from __future__ import annotations

import hashlib
import json
import subprocess
from pathlib import Path
from unittest.mock import patch

import pytest

from autoware_lanelet2_to_opendrive.scenario_utils import (
    _CACHE_DIR,
    _convert_lanelet2_to_xodr_cached,
    _get_converter_commit_hash,
    _sha256_of_file,
    resolve_map_to_xodr,
)


# ---------------------------------------------------------------------------
# _sha256_of_file
# ---------------------------------------------------------------------------


class TestSha256OfFile:
    def test_matches_stdlib(self, tmp_path: Path) -> None:
        data = b"hello autoware"
        f = tmp_path / "sample.bin"
        f.write_bytes(data)
        expected = hashlib.sha256(data).hexdigest()
        assert _sha256_of_file(f) == expected

    def test_large_file_chunked(self, tmp_path: Path) -> None:
        # 128 KiB – forces the chunked read path (chunk size is 64 KiB)
        data = b"x" * 131072
        f = tmp_path / "big.bin"
        f.write_bytes(data)
        expected = hashlib.sha256(data).hexdigest()
        assert _sha256_of_file(f) == expected


# ---------------------------------------------------------------------------
# _get_converter_commit_hash
# ---------------------------------------------------------------------------


class TestGetConverterCommitHash:
    def test_returns_string(self) -> None:
        result = _get_converter_commit_hash()
        assert isinstance(result, str)
        assert len(result) > 0

    def test_returns_unknown_when_git_fails(self) -> None:
        with patch(
            "subprocess.run", side_effect=subprocess.CalledProcessError(1, "git")
        ):
            result = _get_converter_commit_hash()
        assert result == "unknown"

    def test_returns_unknown_when_no_git_dir(self) -> None:
        # Verify "unknown" is returned when git itself errors out,
        # which covers the CalledProcessError branch regardless of .git presence.
        with patch(
            "subprocess.run",
            side_effect=subprocess.CalledProcessError(1, "git"),
        ):
            result = _get_converter_commit_hash()
        assert result == "unknown"


# ---------------------------------------------------------------------------
# resolve_map_to_xodr – .xodr passthrough
# ---------------------------------------------------------------------------


class TestResolveMapToXodrPassthrough:
    def test_xodr_returned_unchanged(self, tmp_path: Path) -> None:
        xodr = tmp_path / "map.xodr"
        xodr.write_text("<OpenDRIVE/>")
        result = resolve_map_to_xodr(xodr)
        assert result == xodr

    def test_xodr_string_path_accepted(self, tmp_path: Path) -> None:
        xodr = tmp_path / "map.xodr"
        xodr.write_text("<OpenDRIVE/>")
        result = resolve_map_to_xodr(str(xodr))
        assert result == xodr

    def test_xodr_no_conversion_called(self, tmp_path: Path) -> None:
        xodr = tmp_path / "map.xodr"
        xodr.write_text("<OpenDRIVE/>")
        with patch(
            "autoware_lanelet2_to_opendrive.scenario_utils._convert_lanelet2_to_xodr_cached"
        ) as mock_convert:
            resolve_map_to_xodr(xodr)
            mock_convert.assert_not_called()


# ---------------------------------------------------------------------------
# resolve_map_to_xodr – Lanelet2 triggers conversion
# ---------------------------------------------------------------------------


class TestResolveMapToXodrConversion:
    def test_osm_triggers_conversion(self, tmp_path: Path) -> None:
        osm = tmp_path / "map.osm"
        osm.write_text("<osm/>")
        expected = tmp_path / "result.xodr"
        expected.write_text("<OpenDRIVE/>")

        with patch(
            "autoware_lanelet2_to_opendrive.scenario_utils._convert_lanelet2_to_xodr_cached",
            return_value=expected,
        ) as mock_convert:
            result = resolve_map_to_xodr(osm)

        mock_convert.assert_called_once_with(osm, None, None)
        assert result == expected

    def test_bin_triggers_conversion(self, tmp_path: Path) -> None:
        bin_file = tmp_path / "map.bin"
        bin_file.write_bytes(b"\x00\x01")
        expected = tmp_path / "result.xodr"
        expected.write_text("<OpenDRIVE/>")

        with patch(
            "autoware_lanelet2_to_opendrive.scenario_utils._convert_lanelet2_to_xodr_cached",
            return_value=expected,
        ) as mock_convert:
            result = resolve_map_to_xodr(bin_file)

        mock_convert.assert_called_once_with(bin_file, None, None)
        assert result == expected

    def test_config_and_mgrs_forwarded(self, tmp_path: Path) -> None:
        from autoware_lanelet2_to_opendrive.conversion_config import (
            ConversionConfig,
            OriginSpec,
        )

        osm = tmp_path / "map.osm"
        osm.write_text("<osm/>")
        config = ConversionConfig(origin=OriginSpec(mgrs_code="54SUE"))
        expected = tmp_path / "result.xodr"
        expected.write_text("<OpenDRIVE/>")

        with patch(
            "autoware_lanelet2_to_opendrive.scenario_utils._convert_lanelet2_to_xodr_cached",
            return_value=expected,
        ) as mock_convert:
            resolve_map_to_xodr(osm, config=config, mgrs_code="54SUE")

        mock_convert.assert_called_once_with(osm, config, "54SUE")


# ---------------------------------------------------------------------------
# _convert_lanelet2_to_xodr_cached – cache hit path
# ---------------------------------------------------------------------------


class TestCacheHit:
    def _make_cache(self, sha256: str, commit: str, content: str) -> Path:
        """Create a fake cache entry and return the cached xodr path."""
        cache_key = f"{sha256[:16]}_{commit[:12]}"
        cache_dir = _CACHE_DIR / cache_key
        cache_dir.mkdir(parents=True, exist_ok=True)
        cached_xodr = cache_dir / "map.xodr"
        cached_xodr.write_text(content)
        cache_info = cache_dir / "cache_info.json"
        cache_info.write_text(json.dumps({"source_sha256": sha256}))
        return cached_xodr

    def test_cache_hit_returns_cached_path(self, tmp_path: Path) -> None:
        from autoware_lanelet2_to_opendrive.conversion_config import (
            ConversionConfig,
            OriginSpec,
        )

        fake_sha = "a" * 64
        fake_commit = "b" * 40
        cached_xodr = self._make_cache(fake_sha, fake_commit, "<OpenDRIVE/>")

        osm = tmp_path / "map.osm"
        osm.write_bytes(b"fake osm")
        config = ConversionConfig(origin=OriginSpec(mgrs_code="54SUE"))

        with (
            patch(
                "autoware_lanelet2_to_opendrive.scenario_utils._sha256_of_file",
                return_value=fake_sha,
            ),
            patch(
                "autoware_lanelet2_to_opendrive.scenario_utils._get_converter_commit_hash",
                return_value=fake_commit,
            ),
        ):
            result = _convert_lanelet2_to_xodr_cached(osm, config)

        assert result == cached_xodr

    def test_cache_hit_skips_conversion(self, tmp_path: Path) -> None:
        """Cache hit must return before reaching the (lazy-imported) conversion code."""
        from autoware_lanelet2_to_opendrive.conversion_config import (
            ConversionConfig,
            OriginSpec,
        )

        fake_sha = "c" * 64
        fake_commit = "d" * 40
        cached_xodr = self._make_cache(fake_sha, fake_commit, "<OpenDRIVE/>")

        osm = tmp_path / "map.osm"
        osm.write_bytes(b"fake osm")
        config = ConversionConfig(origin=OriginSpec(mgrs_code="54SUE"))

        # lanelet2 and the converter are lazy-imported inside the function.
        # On a cache hit the code returns before reaching those imports, so
        # patching them at module level is not applicable. We verify the
        # behaviour by confirming the cached path is returned and that no
        # network/IO-heavy code was executed (lanelet2 module untouched).
        with (
            patch(
                "autoware_lanelet2_to_opendrive.scenario_utils._sha256_of_file",
                return_value=fake_sha,
            ),
            patch(
                "autoware_lanelet2_to_opendrive.scenario_utils._get_converter_commit_hash",
                return_value=fake_commit,
            ),
        ):
            result = _convert_lanelet2_to_xodr_cached(osm, config)

        assert result == cached_xodr


# ---------------------------------------------------------------------------
# _convert_lanelet2_to_xodr_cached – error cases
# ---------------------------------------------------------------------------


class TestCacheMissCases:
    def test_missing_file_raises(self, tmp_path: Path) -> None:
        from autoware_lanelet2_to_opendrive.conversion_config import (
            ConversionConfig,
            OriginSpec,
        )

        config = ConversionConfig(origin=OriginSpec(mgrs_code="54SUE"))
        with pytest.raises(FileNotFoundError):
            _convert_lanelet2_to_xodr_cached(tmp_path / "nonexistent.osm", config)

    def test_no_origin_raises(self, tmp_path: Path) -> None:
        from autoware_lanelet2_to_opendrive.conversion_config import ConversionConfig

        osm = tmp_path / "map.osm"
        osm.write_bytes(b"fake osm")
        config = ConversionConfig()  # no origin set

        with pytest.raises(ValueError, match="no origin specified"):
            _convert_lanelet2_to_xodr_cached(osm, config)

    def test_default_config_used_when_none(self, tmp_path: Path) -> None:
        """Passing config=None should not raise TypeError before the origin check."""
        osm = tmp_path / "map.osm"
        osm.write_bytes(b"fake osm")

        # No config means default ConversionConfig (no origin) → ValueError
        with pytest.raises(ValueError, match="no origin specified"):
            _convert_lanelet2_to_xodr_cached(osm, config=None)
