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
- A GitHub Personal Access Token with `repo` scope, exported as `GH_PAT`:

  ```bash
  export GH_PAT=ghp_xxx
  ```

  This token is required because the project depends on the private repository
  `tier4/lanelet2_python_api_for_autoware`. The token is passed to the Docker
  build as a BuildKit secret and is **not** persisted in the resulting image.

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

The `dev` profile bind-mounts the repository root at `/workspace`, so source
edits on the host are immediately visible inside the container. Two named
volumes isolate state from the host:

- `venv-cache` is mounted over `/workspace/.venv` so the container's
  interpreter-specific virtualenv never leaks onto the host (and vice versa).
- `uv-cache` mounts `/root/.cache/uv` so `uv`'s download cache persists
  across runs.

If you switch base images or the lock file changes substantially, delete
`venv-cache` to force a clean reinstall:

```bash
docker volume rm $(basename "$PWD")_venv-cache
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

### `Failed to download and build lanelet2-python-api-for-autoware`

`GH_PAT` is unset or invalid. Verify with `echo "${GH_PAT:0:8}"` — you should
see `ghp_...` or `gho_...`. The token must have `repo` scope.

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
docker volume rm $(basename "$PWD")_venv-cache
docker compose --profile <whatever> run --rm <service>
```

### Pre-commit fails with `Is it installed, and are you in a Git repository directory?`

Hooks run inside a container that has `git` installed but cannot resolve the
host's `.git` if you are working from a `git worktree`. The bind-mounted
`.git` file inside a worktree points to a path under the parent repository
that the container does not see. Run the lint profile from a regular checkout
of the repository, or run pre-commit directly on the host.

### `docker compose ... config` reports a YAML error

Ensure you are using Compose v2 (`docker compose version` should print
something like `Docker Compose version v2.x.x`). The legacy `docker-compose`
binary does not understand `secrets:` in `build:` blocks.
