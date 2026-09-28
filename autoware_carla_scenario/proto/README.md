# Vendored egodriver wire contract

These `.proto` files are verbatim copies. They let the ego driver policy
(`autoware_carla_scenario.driver_policy`) speak `egodriver.EgodriverService`
without depending on `alpasim-grpc`. That package requires Python >= 3.11, and
this workspace is pinned to 3.10.

| Files | Source | Revision |
| --- | --- | --- |
| `alpasim_grpc/v0/common.proto`, `sensorsim.proto`, `egodriver.proto` | [NVlabs/alpasim](https://github.com/NVlabs/alpasim) `src/grpc/alpasim_grpc/v0/` | `68709245a5dc0f2eda4f8cb2c3aa8cbdfa913043` |
| `carla_driver/v0/carla_driver.proto` | [hakuturu583/carla_driver_interface](https://github.com/hakuturu583/carla_driver_interface) `proto/carla_driver/v0/` | `ccc84acc1b2547055885239ec8b85f450d5992ad` |

The alpasim revision is the one `carla_driver_interface` pins its
`alpasim-grpc` dependency to. The two sides of the gRPC connection therefore
compile the same package names, message layouts and field numbers. The wire
format depends only on those, so a driver served by `carla_driver_interface`
decodes what this package sends without change.

The alpasim files are Apache-2.0, Copyright (c) 2025 NVIDIA Corporation. Their
headers are kept as they are.

## Updating

1. Copy the new files in from the revisions you are moving to, and update the
   table above.
2. Regenerate the Python modules:

   ```bash
   uvx --python 3.10 --from grpcio-tools==1.62.3 \
       python autoware_carla_scenario/scripts/compile_driver_protos.py
   ```

3. Update `ALPASIM_GRPC_REV` and `CARLA_DRIVER_INTERFACE_REV` in
   `driver_policy/wire.py`.
