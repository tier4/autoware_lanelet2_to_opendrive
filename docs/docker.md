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
| `dev` | `l2o-dev:local` | Full development environment matching CI, including pytest, mypy, pre-commit, and the CARLA 0.10.0 wheel. |
| `convert` | `l2o-convert:local` | Slim runtime image whose entrypoint is the `convert` CLI. Reuses the same `.venv` as `dev` and is intended for end users who only need conversion. |

Both images install the workspace via a single `uv sync --dev` invocation.
Splitting that into separate runtime and dev syncs was attempted but produced
non-deterministic builds of `lanelet2-python-api-for-autoware`'s C++ wrapper
shared libraries — the single-sync approach is slightly larger but reliable.

## CI-equivalent local jobs

Each CI job has a matching compose service with the same command. Profiles
prevent accidental `docker compose up` from starting anything.

```bash
# Run the full pytest suite (same as CI's `test` job, default carla extra)
docker compose --profile test run --rm pytest

# Run pre-commit on all files (same as CI's `lint-and-format` job)
docker compose --profile lint run --rm lint

# Run qc-validate against the bundled nishishinjuku fixture
docker compose --profile qc run --rm qc-validate

# Run the CARLA import test (convert -> carla-import-test -> analyze)
docker compose --profile carla run --rm carla-import-test

# Open an interactive shell with the workspace bind-mounted
docker compose --profile dev run --rm dev
```

### Running from a git worktree

If `.` is a [`git worktree`](https://git-scm.com/docs/git-worktree) rather
than the main checkout, its `.git` is a text file pointing at an absolute
host path inside the *main* repository's `.git/worktrees/<name>` directory —
a location outside the `.:/workspace` bind mount, so it's invisible to the
container. `docker-compose.yml` mounts an extra volume at
`${GIT_COMMON_DIR}` to make that path resolve; export it before running any
compose command from a worktree:

```bash
export GIT_COMMON_DIR=$(git rev-parse --path-format=absolute --git-common-dir)
docker compose --profile lint run --rm lint
```

When `GIT_COMMON_DIR` is unset (the normal, non-worktree case) the volume
falls back to a harmless empty directory and every command above works
exactly as documented, with no extra step. The image also bakes in
`git config --system --add safe.directory '*'` so that the bind-mounted
`.git` (owned by the host UID) isn't rejected by git's "dubious ownership"
check inside the container (which runs as root).

The `dev` profile bind-mounts the repository root at `/workspace`, so source
edits on the host are immediately visible inside the container. Two named
volumes isolate state from the host:

- `venv-cache` is mounted over `/workspace/.venv` so the container's
  interpreter-specific virtualenv never leaks onto the host (and vice versa).
- `uv-cache` mounts `/root/.cache/uv` so `uv`'s download cache persists
  across runs.

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

### Pre-commit fails with `Is it installed, and are you in a Git repository directory?`

You are running from a `git worktree` without exporting `GIT_COMMON_DIR`
first. See [Running from a git worktree](#running-from-a-git-worktree)
above.

### `docker compose ... config` reports a YAML error

Ensure you are using Compose v2 (`docker compose version` should print
something like `Docker Compose version v2.x.x`). The legacy `docker-compose`
binary does not understand `secrets:` in `build:` blocks.
