# Tutorial: Writing Your First Scenario

This tutorial walks you through creating an abstract scenario, pairing it
with a concrete YAML config, and verifying the result in the viewer — all
in under 15 minutes.

---

## Key Concepts

| Term | What it is | File type |
|---|---|---|
| **Abstract scenario** | Reusable Python class that defines *what to test* (conditions, actions, entities). | `*.py` |
| **Config dataclass** | Typed parameters the scenario accepts (timeout, speed, lanelet IDs, ...). | `*.py` |
| **Concrete scenario** | YAML file that binds an abstract scenario to *specific* parameter values. | `*.yaml` |

One abstract scenario can have many concrete variants (straight, left turn, right turn, etc.).

---

## Step 1 — Create the Config Dataclass

Add your config to `src/autoware_carla_scenario/examples/configs.py`:

```python
@dataclass
class MyScenarioConfig:
    """Parameters for my custom scenario."""

    name: str = "my_scenario"

    #: Lanelet IDs the ego should visit.
    expected_lanelet_ids: list[int] = field(default_factory=lambda: [460])

    #: Fail-safe timeout in seconds.
    timeout_seconds: float = 10.0
```

Every field becomes overridable from the YAML side via Hydra.

---

## Step 2 — Create the Abstract Scenario

Create `src/autoware_carla_scenario/examples/my_scenario.py`:

```python
from __future__ import annotations

from autoware_carla_scenario import (
    EGO_ROLE_NAME,
    AndCondition,
    BaseScenario,
    EgoConfig,
    EntityLanePositionCondition,
    GroundProjectionConfig,
    Lanelet2Pose,
    StickyCondition,
    TimeoutCondition,
    to_opendrive,
)

from .configs import MyScenarioConfig


class MyScenario(BaseScenario):
    """Verify the ego visits every expected road."""

    def __init__(
        self,
        ego_config: EgoConfig,
        spawn_pose: Lanelet2Pose,
        config: MyScenarioConfig | None = None,
        ground_projection: GroundProjectionConfig | None = None,
    ) -> None:
        super().__init__(
            ego_config, spawn_pose=spawn_pose,
            ground_projection=ground_projection,
        )
        self._config = config or MyScenarioConfig()

    def setup(self) -> None:
        # 1. Snap ego spawn to CARLA road surface
        self._setup_ego_spawn()

        cfg = self._config

        # 2. Build pass condition — "ego visited all expected roads"
        stickies = []
        for ll_id in cfg.expected_lanelet_ids:
            od = to_opendrive(Lanelet2Pose(lanelet_id=ll_id, s=0.0))
            stickies.append(
                StickyCondition(
                    EntityLanePositionCondition(
                        entity_name=EGO_ROLE_NAME,
                        road_id=od.road_id,
                    )
                )
            )
        self.register_pass_condition(AndCondition(stickies))

        # 3. Fail-safe timeout
        self.register_fail_condition(
            TimeoutCondition(cfg.timeout_seconds, label="timeout")
        )

    def is_done(self) -> bool:
        return False
```

### Anatomy of `setup()`

Every scenario follows the same structure:

1. **Spawn** — call `self._setup_ego_spawn()` (snaps the Lanelet2 pose to the
   CARLA road surface).
2. **Pass conditions** — register one or more conditions via
   `self.register_pass_condition()`. When all pass conditions are met the
   scenario succeeds.
3. **Fail conditions** — register via `self.register_fail_condition()`.
   If any fail condition triggers first, the scenario fails.
4. **Actions** (optional) — e.g. `TurnAction`, `TrafficSignalAction`.

!!! tip "Discovering available Conditions and Actions"
    Run `grep -r "class.*Condition" src/autoware_carla_scenario/conditions/` and
    `grep -r "class.*Action" src/autoware_carla_scenario/actions/` to see
    everything the framework provides.

---

## Step 3 — Register the Scenario

In `src/autoware_carla_scenario/examples/run.py`, add the import and
registration:

```python
from .my_scenario import MyScenario
from .configs import MyScenarioConfig

register_scenario("my_scenario", MyScenario, MyScenarioConfig)
```

The first argument (`"my_scenario"`) is the name referenced in YAML's
`scenario.name` field.

---

## Step 4 — Write a Concrete Scenario (YAML)

Create `src/autoware_carla_scenario/examples/conf/scenario/my_scenario/default.yaml`:

```yaml
# @package _global_
scenario:
  name: my_scenario
  expected_lanelet_ids: [460, 265]
  timeout_seconds: 8.0

ego:
  initial_speed_kmh: 5.0
  spawn_lanelet_id: 242
  spawn_s: 25.0
```

!!! note "`# @package _global_`"
    This Hydra directive is **required** on the first line. It tells Hydra to
    merge the file into the root config, not into a nested key.

### Minimal checklist

- [x] `scenario.name` matches the registered name (`"my_scenario"`)
- [x] `ego.spawn_lanelet_id` is a valid lanelet ID in the target map
- [x] `ego.spawn_s` is within the lanelet length

---

## Step 5 — Run It

```bash
# Single run
uv run scenario scenario=my_scenario/default

# Batch — run all variants under my_scenario/
uv run scenario scenario='my_scenario/*'

# Override a parameter on the fly
uv run scenario scenario=my_scenario/default scenario.timeout_seconds=20.0
```

---

## Step 6 — Check Results in the Viewer

```bash
# Start the viewer (defaults to port 9000)
uv run viewer

# Or point at a specific output directory
VIEWER_BASE_PATH=outputs uv run viewer
```

Open <http://localhost:9000> in your browser:

1. **Session list** — shows each run with pass/fail summary.
2. **Session detail** — click a session to see every scenario's condition tree
   and recorded video.
3. **Trigger a new run** — use the "Run" button in the UI to launch a scenario
   directly from the browser.

The viewer scans `outputs/` (single/batch runs) and `multirun/` (sweep runs)
for `*_result.json` files. Click **Refresh** if you ran a scenario while the
viewer was open.

---

## Adding Sweep (Parametric Testing)

To test the same scenario across many lanelets automatically, add a `sweep`
section to the YAML:

```yaml
# @package _global_
scenario:
  name: my_scenario
  expected_lanelet_ids: [460, 265]
  timeout_seconds: 8.0

ego:
  initial_speed_kmh: 5.0
  spawn_lanelet_id: 242
  spawn_s: 25.0

sweep:
  constraints:
    ego.spawn_lanelet_id:
      - type: and
        constraints:
          - type: lanelet_length
            rule: greater_than_or_equal
            value: 10.0
          - type: not
            constraint:
              type: is_junction
```

Run as a multirun:

```bash
uv run scenario --multirun scenario=my_scenario/default \
  hydra/sweeper=lanelet_constraint
```

The sweeper evaluates constraints against the Lanelet2 map and generates one
job per matching lanelet. Results appear under `multirun/` in the viewer.

---

## Writing Scenarios with Claude Code

This repository ships a **`write-scenario` Claude Code plugin** that
streamlines the entire workflow. If you use
[Claude Code](https://claude.ai/code), the plugin is automatically
available.

### Slash Commands

| Command | Description |
|---|---|
| `/write-scenario` | Interactive 7-step guide: concept, discover conditions/actions, write code, debug, add sweep, run |
| `/debug-concrete-scenario` | Execute a scenario, analyze logs, and iterate until it passes |
| `/review-scenario` | Review parameter design for orthogonality and sweeper compatibility |

### Example Session

```
You: /write-scenario

Claude: What scenario do you want to create?

You: I want to test that the ego stops at a stop sign,
     waits 3 seconds, then proceeds.

Claude: (walks you through each step — discovering StandstillCondition,
        writing the Python class, generating the YAML, registering it,
        running it, and debugging any failures)
```

The plugin automatically discovers available **Conditions**, **Actions**,
**Constraints**, and **Bindings** from the codebase — you do not need to
memorize them.

### Tips

- Start with `/write-scenario` for new scenarios — it handles all four files
  (config, class, YAML, registration) in one session.
- Use `/debug-concrete-scenario` when a scenario fails — it reads CARLA logs
  and suggests targeted fixes.
- Use `/review-scenario` before adding sweep — it checks that parameters are
  orthogonal and sweeper-compatible.
