# Ego Driver Policy

`EgoDriverEntity` drives the ego from an external driver process. The driver
serves `egodriver.EgodriverService`, the driver contract of
[NVlabs/alpasim](https://github.com/NVlabs/alpasim). Any driver written against
that contract works unmodified, including the policies of
[carla_driver_interface](https://github.com/hakuturu583/carla_driver_interface).

## How it fits together

alpasim puts a *Runtime* at the centre of the loop. The Runtime owns the world
and queries the driver once per policy step. `carla_driver_interface` ships a
Runtime of its own (`CarlaRuntime`), but that one connects its own client and
loads the map again. `ScenarioRunner` already owns the world, so here it plays
the Runtime role itself, and only the driver runs in a separate process:

```
┌──────────── ScenarioRunner process (Python 3.10) ───────────┐   ┌─── Driver process ───┐
│                                                             │   │                      │
│  ScenarioRunner tick loop                                   │   │  any egodriver       │
│   └ EgoDriverEntity                                         │   │  server, e.g.        │
│       └ EgoDriverPolicy ......... alpasim Runtime role      │   │  carla-driver-       │
│           ├ RouteProvider ....... route ahead, rig frame    │   │  interface serve     │
│           ├ CarlaGroundTruth .... CarlaRendererData         │   │  (Python 3.11+)      │
│           └ TrajectoryFollower .. pure pursuit + speed PID  │   │                      │
└──────────────────────────────┬──────────────────────────────┘   └──────────▲───────────┘
                               └──── egodriver.EgodriverService (gRPC) ──────┘
```

The two processes need different Python versions. This workspace is pinned to
Python 3.10 (by lanelet2 and the CARLA 0.10.0 wheel), while `alpasim-grpc` and
therefore `carla_driver_interface` require 3.11 or newer. The gRPC boundary is
what lets them work together. The scenario side does not import either package.
It uses a vendored copy of the same `.proto` files, pinned to the same alpasim
revision, so the bytes on the wire are identical
(see `autoware_carla_scenario/proto/README.md`).

## One policy step

With the defaults (`policy_timestep_s=0.1`, 20 Hz ticks), a step runs on every
other tick, in the order of alpasim's `PolicyEvent`:

1. `submit_image_observation`: the newest frame from each configured camera.
2. `submit_egomotion_observation`: every ego pose since the previous step, with
   velocities, in the rig frame.
3. `submit_route`: the route ahead (80 m at 2 m spacing), in the rig frame.
4. `submit_recording_ground_truth`: only if `send_ground_truth=True`.
5. `drive`: with `CarlaRendererData` in `renderer_data`. This carries the
   governing traffic light and its stop-line distance, the speed limit,
   weather, and nearby vehicles and pedestrians.
6. The returned plan is tracked with pure pursuit and a speed PID, and the
   control is applied to the ego.

When the driver sets `terminate_session`, or the route is complete, the policy
stops querying and holds the brake. The scenario's own pass and fail
conditions still decide the result.

## Usage

Start a driver:

```bash
# In a carla_driver_interface checkout (Python 3.11 or 3.12)
uv run carla-driver-interface serve --policy route_follower --port 50051
```

Pass the entity factory as the scenario's `ego_type`:

```python
from autoware_carla_scenario import (
    BaseScenario,
    EgoConfig,
    EgoDriverEntity,
    EgoDriverPolicyConfig,
)


class MyScenario(BaseScenario):
    def __init__(self, ego_config: EgoConfig) -> None:
        super().__init__(
            ego_config,
            ego_type=EgoDriverEntity.factory(
                EgoDriverPolicyConfig(driver_address="localhost:50051")
            ),
        )
```

`ScenarioRunner` then skips TrafficManager autopilot for the ego and opens the
driver session after the warm-up ticks. The session is closed when the ego is
destroyed.

## Configuration

`EgoDriverPolicyConfig` mirrors `carla_driver_interface.runtime.RuntimeConfig`
wherever the same setting exists, so a driver receives the same inputs under
either runtime.

| Field | Default | Meaning |
|-------|---------|---------|
| `driver_address` | `"localhost:50051"` | `host:port` of the driver. |
| `driver_timeout_s` | `60.0` | Per-RPC timeout. |
| `scene_id` | `None` | Reported to the driver; defaults to `"<map>:<scenario class>"`. |
| `policy_timestep_s` | `0.1` | Policy period; must be a multiple of the tick (0.05 s). |
| `rear_axle_offset_m` | `None` | Actor origin to rig origin (rear axle); derived from wheel physics when `None`. |
| `cameras` | `()` | Cameras streamed to the driver; `default_camera_rig()` gives the upstream front camera. |
| `image_format`, `image_quality` | JPEG, 90 | Camera encoding (PNG or JPEG). |
| `route` | `None` | CARLA world `(x, y, z)` points to follow; `None` walks the lane graph from the ego, choosing junction exits with the scenario's random seed. |
| `route_length_m` | `500.0` | Length of the generated route. |
| `route_horizon_m`, `route_resolution_m` | `80.0`, `2.0` | Route window sent each step. |
| `send_renderer_data` | `True` | Send `CarlaRendererData`. |
| `traffic_light_sight_distance_m` | `60.0` | How far down the lane a governing light is looked for. |
| `send_actor_ground_truth`, `actor_horizon_m` | `True`, `150.0` | Report surrounding actors, and how far out. |
| `control` | `ControlConfig()` | Pure pursuit and PID tuning. |

## Differences from `CarlaRuntime`

- The scenario owns the world, the map, the ego spawn and background traffic.
  `EgoDriverPolicy` only reads the world and actuates the ego.
- Egomotion noise is not modelled: the pose the driver is told is the true
  pose.
- No rollout metrics are computed. Pass and fail are decided by the scenario's
  conditions. `EgoDriverPolicy.route_completion`, `terminated_by_driver`,
  `last_plan` and `last_command` are available to write conditions against.
