# Vendored alpasim protobuf definitions

The `.proto` files under `alpasim_grpc/` are copied **verbatim** from
[NVlabs/alpasim](https://github.com/NVlabs/alpasim) so that this package can speak the
`egodriver.EgodriverService` gRPC contract without depending on the `alpasim-grpc`
distribution.

| Field | Value |
| --- | --- |
| Upstream repository | `https://github.com/NVlabs/alpasim` |
| Pinned revision | `68709245a5dc0f2eda4f8cb2c3aa8cbdfa913043` |
| Upstream path | `src/grpc/alpasim_grpc/v0/` |
| License | Apache-2.0 (see `LICENSE.alpasim`) |

## Why vendor instead of depend

`alpasim-grpc` declares `requires-python = ">=3.11,<3.13"`, while this package is pinned to
`>=3.10,<3.11` because the CARLA 0.10.0 Python API wheel is CPython-3.10 only. The two cannot
coexist in one environment, so the wire contract is vendored and compiled locally instead.

Only the transitive closure of `egodriver.proto` is vendored:

```
egodriver.proto
├── common.proto
└── sensorsim.proto
    └── common.proto
```

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

## Updating to a newer alpasim revision

1. Copy the three `.proto` files from the new revision.
2. Update the pinned revision in this file and in `scripts/compile_protos.py`.
3. Re-run the generator and the test suite.
