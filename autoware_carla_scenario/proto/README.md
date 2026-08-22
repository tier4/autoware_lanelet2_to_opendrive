# Vendored protobuf definitions

The `.proto` files here are copied **verbatim** from two upstreams, so that this package
can speak the `egodriver.EgodriverService` gRPC contract — and the CARLA ground-truth
extension a policy reads alongside it — without depending on either distribution.

| Directory | Upstream | Pinned revision | Upstream path |
| --- | --- | --- | --- |
| `alpasim_grpc/` | [NVlabs/alpasim](https://github.com/NVlabs/alpasim) | `68709245a5dc0f2eda4f8cb2c3aa8cbdfa913043` | `src/grpc/alpasim_grpc/v0/` |
| `carla_driver/` | [hakuturu583/carla_driver_interface](https://github.com/hakuturu583/carla_driver_interface) | `af1dcd3d3ddae7739f811e414a642d72d0440386` | `proto/carla_driver/v0/` |

Both are Apache-2.0; see `LICENSE.alpasim`.

## Why vendor instead of depend

`alpasim-grpc` declares `requires-python = ">=3.11,<3.13"`, while this package is pinned to
`>=3.10,<3.11` because the CARLA 0.10.0 Python API wheel is CPython-3.10 only. The two cannot
coexist in one environment, so the wire contract is vendored and compiled locally instead.

Only the transitive closure of the two entry points is vendored:

```
egodriver.proto              carla_driver.proto
├── common.proto             ├── common.proto
└── sensorsim.proto          ├── egodriver.proto
    └── common.proto         └── sensorsim.proto
```

`carla_driver.proto` defines the CARLA-specific messages that ride inside the two `bytes`
extension points alpasim already provides — `DriveRequest.renderer_data` and
`DriveResponse.DebugInfo.unstructured_debug_info` — so nothing about the alpasim wire
format changes.

The files are **never edited**. Field numbers, package names, and the
`alpasim_grpc/v0/...` import paths are preserved exactly, which is what keeps the generated
messages wire-compatible with an upstream alpasim runtime or with
[hakuturu583/carla_driver_interface](https://github.com/hakuturu583/carla_driver_interface).

## Regenerating the Python stubs

Generated modules are committed under
`src/autoware_carla_scenario/driver/_proto/` so that the runtime never needs `grpcio-tools`.
Regenerate them with:

```bash
uv run python autoware_carla_scenario/scripts/compile_protos.py
```

`--check` verifies the committed output is up to date without writing anything; the same check
runs in `test_proto_generated.py`.

## Updating to a newer upstream revision

1. Copy the `.proto` files from the new revision.
2. Update the pinned revision in this file and in `scripts/compile_protos.py`
   (`ALPASIM_REV` / `CARLA_DRIVER_INTERFACE_REV`).
3. Re-run the generator and the test suite.

The `carla_driver` pin matters for behaviour, not just for messages: `af1dcd3` is the
revision that made the runtime report a traffic light *before* the ego reaches its stop
line, and `driver/renderer.py` reproduces that logic. A policy pinned to the same
revision — as [stl_driver](https://github.com/hakuturu583/stl_driver) is — expects it.
