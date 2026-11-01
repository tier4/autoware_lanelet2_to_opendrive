# Docker-Based Build & Test Environment

This repository ships a multi-stage `Dockerfile` and a `docker-compose.yml` so
that developers can reproduce the GitHub Actions CI jobs locally and so that
end users can run the `convert` tool without installing Python, `uv`, or any
system libraries directly on their host.

## Prerequisites

- Docker 23.0 or newer (BuildKit must be available; it is enabled by default
  on supported daemons).
- Docker Compose v2 (`docker compose ...`, with a space — not the legacy
  `docker-compose` binary).
## Image targets

| Target | Image tag (compose) | Purpose |
| --- | --- | --- |
| `dev` | `l2o-dev:local` | Full development environment on CI's OS and Boost versions, including pytest, mypy, pre-commit, and the CARLA 0.10.0 wheel. |
| `dev-nocarla` | `l2o-dev-nocarla:local` | CARLA-free. Builds and runs for the host's native architecture, avoiding QEMU emulation on Apple Silicon. |
| `convert` | `l2o-convert:local` | Slim runtime image whose entrypoint is the `convert` CLI. Reuses the same `.venv` as `dev` and is intended for end users who only need conversion. |

Both `dev` and `dev-nocarla` install the workspace via a single `uv sync --dev`
invocation (the latter without the `carla` extra). Splitting that into
separate runtime and dev syncs was attempted but produced non-deterministic
builds of `lanelet2-python-api-for-autoware`'s C++ wrapper shared libraries —
the single-sync approach is slightly larger but reliable.

## Architecture and CARLA

The `carla` extra (`typesafe-carla`, which provides the CARLA 0.10.0 client)
publishes only an x86_64 wheel, so every service built from the `dev` or
`convert` image (`dev`, `pytest`, `lint`, `qc-validate`, `carla-import-test`,
`convert`) pins `platform: linux/amd64` explicitly in `docker-compose.yml`. On
Apple Silicon this necessarily runs under QEMU emulation.

`dev-nocarla` and its compose services (`dev-native`, `pytest-native`) carry
no such pin: they build and run for the host's native architecture. On Apple
Silicon that means arm64, with no emulation overhead.

The difference is measured, not theoretical. The same 245-test subset of the
suite takes 2095.42s under amd64 emulation and 108.52s natively on arm64. Most
of that remaining 108.52s is one single slow test
(`test_issue_291_diverging_roads_have_no_lane_drop`, ~103s, run serially), so
further reductions need separate caching work rather than more native
throughput. **Apple Silicon developers should default to `--profile
test-native`** for day-to-day iteration, and fall back to `--profile test`
(the CARLA-inclusive, amd64 path) before opening a PR or after
touching the CARLA code path.

No test module imports the CARLA client — only the `carla-import-test`
entrypoint does — so `pytest-native` collects and runs the same tests as
`pytest`. What it cannot exercise is `carla-import-test` itself: use
`--profile carla` to verify any change to that code path.

## Local jobs covering CI's checks

Each CI job has a matching compose service with the same command. Profiles
prevent accidental `docker compose up` from starting anything.

```bash
# Day-to-day test loop. Native architecture, no QEMU emulation, no `carla`
# extra installed.
docker compose --profile test-native run --rm pytest-native

# Full pytest suite with the `carla` extra, in the image matching CI's OS and
# Boost versions (the same pytest flags as CI's `test` job, though CI
# invokes them through `uv run` and this does not -- see docs/docker.md). Pinned to linux/amd64, so
# this runs under emulation -- and is much slower -- on Apple Silicon. Use
# before opening a PR.
docker compose --profile test run --rm pytest

# Run pre-commit on all files (same as CI's `lint-and-format` job)
docker compose --profile lint run --rm lint

# Run qc-validate against the bundled nishishinjuku fixture
docker compose --profile qc run --rm qc-validate

# Run the CARLA import test (convert -> carla-import-test -> analyze)
docker compose --profile carla run --rm carla-import-test

# Open an interactive shell with the workspace bind-mounted (CARLA-enabled,
# CARLA-enabled, linux/amd64)
docker compose --profile dev run --rm dev

# Open a native interactive shell (CARLA-free, host architecture)
docker compose --profile dev-native run --rm dev-native
```

### Running from a git worktree

If `.` is a [`git worktree`](https://git-scm.com/docs/git-worktree) rather
than the main checkout, its `.git` is a text file pointing at an absolute
host path inside the *main* repository's `.git/worktrees/<name>` directory —
a location outside the `.:/workspace` bind mount, so it's invisible to the
container. `docker-compose.yml` mounts an extra volume at
`${L2O_GIT_COMMON_DIR}` on both the carla-base and native-base anchors to
make that path resolve; export it before running any compose command from a
worktree:

```bash
export L2O_GIT_COMMON_DIR=$(git rev-parse --path-format=absolute --git-common-dir)
docker compose --profile lint run --rm lint
docker compose --profile test-native run --rm pytest-native
```

When `L2O_GIT_COMMON_DIR` is unset (the normal, non-worktree case) the
volume falls back to a harmless empty directory and every command above
works exactly as documented, with no extra step.

The `L2O_` prefix is not decoration. **`GIT_COMMON_DIR` is a name git
itself reads from the environment**, so exporting it as above would point
every later git command in that shell at the same directory, whichever
repository they were run in. The variable was renamed for that reason, and
the reason is recorded at both mount sites in `docker-compose.yml` as well. The image also bakes in
`git config --system --add safe.directory '*'` so that the bind-mounted
`.git` (owned by the host UID) isn't rejected by git's "dubious ownership"
check inside the container (which runs as root).

The `dev` and `dev-native` profiles bind-mount the repository root at
`/workspace`, so source edits on the host are immediately visible inside the
container. Named volumes isolate state from the host, and the two
architectures never share one:

- `venv-cache` is mounted over `/workspace/.venv` for the CARLA-enabled
  services (`dev`, `pytest`, `lint`, `qc-validate`, `carla-import-test`) so
  the container's interpreter-specific virtualenv never leaks onto the host
  (and vice versa); `uv-cache` mounts `/root/.cache/uv` so `uv`'s download
  cache persists across runs.
- `venv-nocarla-cache` / `uv-nocarla-cache` do the same for the native
  services (`dev-native`, `pytest-native`).

**Never point a CARLA-enabled service and a native service at the same
volume** — the two venvs differ in both dependency set and architecture, and
sharing one named volume between them reproduces the `SystemError:
initialization` failure documented below.

If you switch base images or the lock file changes substantially, delete
`venv-cache` to force a clean reinstall. The simplest option is to let
Compose remove its own volumes:

```bash
docker compose --profile dev down -v
```

If you prefer to remove the volume directly, quote the name (the directory
may contain spaces) and resolve the actual volume — Compose prefixes volume
names with the project name, which defaults to the working directory but
can be overridden with `-p <name>` or `COMPOSE_PROJECT_NAME`:

```bash
docker volume ls --filter name=venv-cache
docker volume rm "$(basename "$PWD")_venv-cache"
```

The compose services invoke entrypoints directly (e.g. `convert`, `pytest`)
rather than wrapping them in `uv run`. This is intentional: `uv run` triggers
an implicit sync against the bind-mounted source on every invocation, which
can rebuild the workspace packages and destabilize the carefully-built native
dependencies that the image already contains.

## Red/green runs for test-driven development

`scripts/tdd.py` runs selected tests in the container and says whether the
result is a genuine red or green. It needs only a host Python 3.10+ (standard
library) and Docker.

```bash
python scripts/tdd.py red   autoware_lanelet2_to_opendrive/test/test_x.py::test_y
python scripts/tdd.py green autoware_lanelet2_to_opendrive/test/test_x.py::test_y
python scripts/tdd.py run   autoware_lanelet2_to_opendrive/test/test_x.py   # no verdict
python scripts/tdd.py stop                                                   # stop the container
```

Everything after the subcommand is passed to pytest.

| | accepted only when | rejected, with the reason |
| --- | --- | --- |
| `red` | every selected test ran and **failed** | passes already; error in setup/teardown; skipped; collection error (e.g. an import failure at module level); nothing selected |
| `green` | every selected test **passed** | failed; error; skipped; still marked `xfail`; nothing selected |

A `red` verdict lists each failure message. A failure inside the test body can
still be the wrong one -- an `ImportError` of the function under test fails
too -- so read the reasons, not only the verdict.

`red` passes `--runxfail`, so a test committed as red under
`@pytest.mark.xfail(strict=True)` is still checked as failing; `green` does
not, so a leftover marker shows up as `XPASS(strict)`.

Both clear `addopts` and disable pytest-testmon (`-p no:pytest-testmon`), so an
`addopts` added to the project later, or a stray `--testmon`, cannot change
what runs: testmon deselects tests it considers unaffected by the last change,
which would turn a red into "nothing ran". Tests run serially; pass `-n N`
yourself to parallelise.

The tests run in a resident `tdd` compose service (`sleep infinity`, profile
`tdd`), started on first use, so each run is a `docker compose exec` of a few
seconds rather than a fresh container. Stop it with `python scripts/tdd.py stop`.

## Using the `convert` distribution image

The `convert` image is intended to be used standalone. Build it once, then
invoke it from any directory containing your `.osm` map:

```bash
# Build (only needed once, or when dependencies change)
docker compose --profile convert build convert

# Run the conversion. Mount the directory holding your map at /io.
docker run --rm -v "$PWD:/io" l2o-convert:local \
  map=nishishinjuku target=carla \
  input_map_path=/io/your-map.osm \
  output_map_path=/io/your-map.xodr
```

Arguments are passed verbatim to the underlying `convert` CLI (Hydra syntax).
Use `docker run --rm l2o-convert:local --help` to see all supported keys.

Output files inside the mounted volume will be owned by `root` because the
container runs as root by default; on Linux you can run with `--user
"$(id -u):$(id -g)"` to retain host ownership.

## Troubleshooting

### `the --frozen flag was used but the lockfile is out of date`

`pyproject.toml` was modified without regenerating `uv.lock`. Run
`uv lock` on the host, commit the updated `uv.lock`, then rebuild.

### `import lanelet2 ... cannot open shared object file: liblanelet2_*.so`

The dynamic loader couldn't find lanelet2's bundled libraries. The base image
sets `LD_LIBRARY_PATH=/workspace/.venv/lib/python3.10/site-packages/lanelet2/lib`
to fix this — if you derived a custom image, make sure that env is preserved.

### `import autoware_lanelet2_extension_python ... SystemError: initialization`

Your `venv-cache` named volume contains a `.venv` from an earlier image build
whose native bindings were broken. Delete the volume and re-run:

```bash
docker volume rm "$(basename "$PWD")_venv-cache"
docker compose --profile <whatever> run --rm <service>
```

### `DOCKER_DEFAULT_PLATFORM` slows down the native profiles too

If this environment variable is set to `linux/amd64` (e.g. left over from a
shell history entry), it forces emulation onto `dev-native` and
`pytest-native` as well, even though they declare no `platform:` of their
own — losing the whole point of the native target. Diagnose with:

```bash
echo $DOCKER_DEFAULT_PLATFORM
docker compose --profile test-native run --rm pytest-native uname -m   # expect aarch64 on Apple Silicon
```

If it is set, `unset DOCKER_DEFAULT_PLATFORM` in the current shell (and check
your shell's rc files and history for where it came from).

**Unsetting it is not enough on its own if a native profile already ran with
it set.** That run installed an amd64 `.venv` into the `venv-nocarla-cache`
named volume, which is mounted over `/workspace/.venv` on every subsequent
run — so the rebuilt arm64 image still starts against the stale amd64
virtualenv. Delete the volume too:

```bash
unset DOCKER_DEFAULT_PLATFORM
docker volume rm "$(basename "$PWD")_venv-nocarla-cache"
docker compose --profile test-native run --rm pytest-native
```

The same applies in reverse to `venv-cache` if a CARLA-enabled service was
ever run with the wrong architecture. `docker compose --profile <name> down
-v` removes all of the project's named volumes at once and is simpler when a
full reinstall is acceptable.

### `uv sync --extra carla` fails building `typesafe-carla` on arm64

`typesafe-carla` publishes only an x86_64 wheel, so on an arm64 host `uv`
has no wheel to install and falls back to the source distribution. This means a
CARLA-requiring service was built without its `linux/amd64` pin. Check that `docker-compose.yml` still has
`platform: linux/amd64` on the affected service — it must not be removed.

### Fewer `pytest-xdist` workers than expected

Worker count follows the container's `nproc`, which is capped by your Docker
runtime's VM CPU allocation (e.g. Rancher Desktop: `rdctl set
--virtual-machine.number-cpus 8` — a host setting, not a repository change).
In this repository's suite the effect is limited: the current bottleneck is
one long-running test executed serially, not worker count (see "Architecture
and CARLA" above).

### Pre-commit fails with `Is it installed, and are you in a Git repository directory?`

You are running from a `git worktree` without exporting
`L2O_GIT_COMMON_DIR` first. See [Running from a git worktree](#running-from-a-git-worktree)
above.

### `docker compose ... config` reports a YAML error

Ensure you are using Compose v2 (`docker compose version` should print
something like `Docker Compose version v2.x.x`). The legacy `docker-compose`
binary does not understand `secrets:` in `build:` blocks.
