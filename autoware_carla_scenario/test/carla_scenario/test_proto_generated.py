"""Guards on the vendored alpasim protobuf definitions and their generated modules."""

from __future__ import annotations

import importlib.util
import sys
from pathlib import Path

import pytest

from autoware_carla_scenario.driver._proto import egodriver_pb2, egodriver_pb2_grpc


_PACKAGE_ROOT = Path(__file__).resolve().parents[2]
_COMPILE_SCRIPT = _PACKAGE_ROOT / "scripts" / "compile_protos.py"


def _load_compiler():
    """Import ``scripts/compile_protos.py`` as a module."""
    spec = importlib.util.spec_from_file_location("_compile_protos", _COMPILE_SCRIPT)
    assert spec is not None and spec.loader is not None  # noqa: S101
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


# ---------------------------------------------------------------------------
# Wire identity
# ---------------------------------------------------------------------------


def test_service_keeps_its_upstream_name() -> None:
    """The full service name is the contract; renaming it breaks every peer."""
    service = egodriver_pb2.DESCRIPTOR.services_by_name["EgodriverService"]
    assert service.full_name == "egodriver.EgodriverService"


def test_descriptor_keeps_its_upstream_path() -> None:
    """Descriptor names must match alpasim's so registries agree."""
    assert egodriver_pb2.DESCRIPTOR.name == "alpasim_grpc/v0/egodriver.proto"


def test_every_contract_rpc_is_generated() -> None:
    expected = {
        "start_session",
        "close_session",
        "submit_image_observation",
        "submit_egomotion_observation",
        "submit_route",
        "submit_recording_ground_truth",
        "drive",
        "get_version",
    }
    service = egodriver_pb2.DESCRIPTOR.services_by_name["EgodriverService"]
    assert {method.name for method in service.methods} == expected


def test_stub_and_servicer_are_available() -> None:
    assert hasattr(egodriver_pb2_grpc, "EgodriverServiceStub")
    assert hasattr(egodriver_pb2_grpc, "EgodriverServiceServicer")
    assert hasattr(egodriver_pb2_grpc, "add_EgodriverServiceServicer_to_server")


# ---------------------------------------------------------------------------
# Provenance and freshness
# ---------------------------------------------------------------------------


def test_vendored_protos_are_present() -> None:
    proto_dir = _PACKAGE_ROOT / "proto" / "alpasim_grpc" / "v0"
    names = {path.name for path in proto_dir.glob("*.proto")}
    assert names == {"common.proto", "sensorsim.proto", "egodriver.proto"}


def test_vendored_protos_keep_their_licence_header() -> None:
    proto_dir = _PACKAGE_ROOT / "proto" / "alpasim_grpc" / "v0"
    for path in proto_dir.glob("*.proto"):
        assert "SPDX-License-Identifier: Apache-2.0" in path.read_text()


def test_generated_modules_are_up_to_date() -> None:
    """Regenerating from the vendored protos must not change the committed output."""
    pytest.importorskip(
        "grpc_tools", reason="grpcio-tools is only installed with the dev dependencies"
    )
    compiler = _load_compiler()
    assert compiler._check() == 0, (  # noqa: SLF001
        "Generated protobuf modules are stale. Run: "
        "uv run python autoware_carla_scenario/scripts/compile_protos.py"
    )
