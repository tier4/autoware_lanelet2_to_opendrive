# Development Guide

This guide is for developers who contribute to the
`autoware-lanelet2-to-opendrive` project. It mirrors the ground rules
codified in the repo-root [`CLAUDE.md`](https://github.com/tier4/autoware_lanelet2_to_opendrive/blob/master/CLAUDE.md).

## Development Environment Setup

### Prerequisites

- **Python 3.10** (exactly 3.10 — see [Installation](installation.md) for
  the rationale)
- **uv** 0.9.7 or newer
- **Docker** with Compose (only required for end-to-end test verification;
  see "Local test verification" below)
- **Git**, plus a GitHub Personal Access Token (`GH_PAT`) with `repo`
  scope when building the Docker image

### Cloning the repository

```bash
git clone https://github.com/tier4/autoware_lanelet2_to_opendrive.git
cd autoware_lanelet2_to_opendrive
```

The repository is a `uv` workspace with two members:
[`autoware_lanelet2_to_opendrive/`](https://github.com/tier4/autoware_lanelet2_to_opendrive/tree/master/autoware_lanelet2_to_opendrive)
(the converter, this package) and
[`autoware_carla_scenario/`](https://github.com/tier4/autoware_lanelet2_to_opendrive/tree/master/autoware_carla_scenario)
(scenario / CARLA integration). A single `uv sync` installs both.

### Installing dependencies

```bash
# Sync the workspace from uv.lock (frozen, reproducible)
uv sync --dev

# Optional: also pull in the CARLA Python wheel for the autoware_carla_scenario package
uv sync --dev --extra carla
```

### Pre-commit hooks

```bash
uv run pre-commit install
```

The configured hooks (see [`.pre-commit-config.yaml`](https://github.com/tier4/autoware_lanelet2_to_opendrive/blob/master/.pre-commit-config.yaml))
run automatically on every commit:

- `pre-commit-hooks` v4.6.0: `trailing-whitespace`, `end-of-file-fixer`,
  `check-yaml` (excluding `mkdocs.yml`), `check-added-large-files`,
  `check-merge-conflict`, `check-toml`, `debug-statements`,
  `mixed-line-ending`
- `ruff` v0.7.4 (`--fix`) and `ruff-format`
- Local `mypy --ignore-missing-imports` over both workspace `src/` and
  `test/` trees
- Local `pytest --no-testmon` runs for each workspace member, executed in
  separate processes to dodge a known shutdown-time crash in the
  lanelet2 C++ bindings when both suites share an interpreter

Never bypass hooks with `--no-verify`. If a hook auto-fixes a file,
re-stage with `git add -u` and commit again.

## Project Structure

```
autoware_lanelet2_to_opendrive/                 # repo root (uv workspace)
├── pyproject.toml                              # workspace + tool config
├── uv.lock                                     # locked dependencies
├── Dockerfile                                  # multi-stage CI / convert image
├── docker-compose.yml                          # dev / test / lint / qc / carla / convert profiles
├── .pre-commit-config.yaml
├── autoware_lanelet2_to_opendrive/             # this package
│   ├── pyproject.toml                          # package metadata & console scripts
│   ├── mkdocs.yml                              # docs site config
│   ├── docs/                                   # docs sources (this site)
│   ├── overrides/                              # mkdocs-material overrides
│   ├── src/autoware_lanelet2_to_opendrive/     # package source
│   │   ├── main.py                             # convert CLI + pipeline orchestrator
│   │   ├── conversion_config.py                # ConversionConfig dataclass tree
│   │   ├── config.py                           # numerical constants (DEFAULT_CONFIG)
│   │   ├── projection.py                       # MGRS / lat-lon helpers
│   │   ├── preprocess_lanelet.py               # preprocess-lanelet CLI + ops
│   │   ├── lanelet.py / junction.py / centerline.py
│   │   ├── geometry.py / spline.py / cubic_spline_1d.py
│   │   ├── divergence.py                       # synthetic divergence/merge junctions
│   │   ├── road_lanelet_geo_mapping.py         # mapping persistence + cross-validation
│   │   ├── map_resolver.py                     # resolve_map_to_xodr helper
│   │   ├── analyze_xodr.py / qc_validate.py    # ASAM QC entry points
│   │   ├── carla_import_test.py                # CARLA smoke test
│   │   ├── opendrive/                          # OpenDRIVE 1.4 dataclasses + builders
│   │   ├── conf/                               # Hydra config (config.yaml, map/, target/)
│   │   ├── types.py / util.py / py.typed
│   └── test/                                   # pytest suite (testpaths in workspace pyproject)
└── autoware_carla_scenario/                    # sister package
```

## Development Workflow

### Common uv commands

```bash
# Add a runtime dependency to this package
uv add --package autoware-lanelet2-to-opendrive <package_name>

# Add a workspace-level dev dependency
uv add --dev <package_name>

# Resync (e.g. after pulling main)
uv sync --dev

# Run a script via the venv's interpreter
uv run python <script.py>
```

### Running the converter during development

```bash
# Console-script form
uv run convert input_map_path=/path/to/map.osm

# Module form (equivalent)
uv run python -m autoware_lanelet2_to_opendrive.main \
    input_map_path=/path/to/map.osm
```

### Coding standards

- **Python 3.10** syntax and features only (the package is pinned to
  `>=3.10,<3.11`).
- Type hints on every public function and method (`py.typed` marker).
- Google-style docstrings on public modules, classes, and functions.
- Naming: `snake_case` for functions and variables, `PascalCase` for
  classes; the distribution name uses hyphens
  (`autoware-lanelet2-to-opendrive`), the import name uses underscores
  (`autoware_lanelet2_to_opendrive`).
- All numerical constants must live in `config.py`'s frozen dataclasses
  (accessed via `DEFAULT_CONFIG`) — see the "Constants Configuration"
  section in `CLAUDE.md` for the rationale.
- Formatting / linting via Ruff; static type checking via `mypy
  --ignore-missing-imports`.
- All new docs, comments, and PR descriptions must be in English (the
  project's [Language Policy](https://github.com/tier4/autoware_lanelet2_to_opendrive/blob/master/CLAUDE.md#language-policy)).

## Local Test Verification (Container-Based)

!!! warning "Run dynamic checks inside the container"
    The runtime dependency `lanelet2-python-api-for-autoware` is built
    from source against the system Boost. Hosts whose Boost ABI does not
    match (e.g. Ubuntu 24.04 with Boost 1.83) will fail the wheel build
    with `RuntimeError: Command failed: make -jN`. The Docker image
    pins Ubuntu 22.04 with Boost 1.74, matching CI exactly.

The repository ships a multi-stage `Dockerfile` and `docker-compose.yml`
with profiles that mirror each CI job. Build once with
`GH_PAT=<your-token>` exported, then run a profile:

```bash
# Full pytest suite (matches CI's `test` job)
docker compose --profile test run --rm pytest

# Pre-commit on all files (matches CI's `lint-and-format` job)
docker compose --profile lint run --rm lint

# QC: convert nishishinjuku and run qc-validate on the result
docker compose --profile qc run --rm qc-validate

# CARLA: convert + carla-import-test + analyze
docker compose --profile carla run --rm carla-import-test

# Interactive workspace shell with the repo bind-mounted
docker compose --profile dev run --rm dev

# Slim convert image (uses the `convert` console script as its entrypoint)
docker compose --profile convert run --rm convert input_map_path=/io/map.osm
```

The static checks listed below do **not** import the package and can be
run directly on the host without Docker:

```bash
uv run ruff check .
uv run ruff format --check .
uv run mypy --ignore-missing-imports \
    autoware_lanelet2_to_opendrive/src \
    autoware_lanelet2_to_opendrive/test
```

If you need to run unit tests without Docker (e.g. for a single test
file) and your host happens to have a compatible Boost, you can fall
back to:

```bash
uv run pytest autoware_lanelet2_to_opendrive/test/<file>.py
```

— but if the build fails with the `make -jN` error, defer to the
container or to CI.

## Contributing

1. Open an issue first for non-trivial changes
2. Fork the repo and create a feature branch (do **not** push to
   `master`)
3. Add tests for any new behaviour
4. Run `uv run pre-commit run --all-files` before committing; stage any
   formatter modifications and commit again
5. Open a PR using the
   [`PULL_REQUEST_TEMPLATE.md`](https://github.com/tier4/autoware_lanelet2_to_opendrive/blob/master/.github/PULL_REQUEST_TEMPLATE.md)
   structure and add **one** version-bump label
   (`bump patch` / `bump minor` / `bump major`) — the
   `Check Version Bump Label` GitHub Action enforces this

### Commit message convention

Follow conventional-commit prefixes:

- `feat:` — new feature
- `fix:` — bug fix
- `docs:` — documentation only
- `test:` — test additions / changes
- `refactor:` — internal refactor
- `chore:` — maintenance / tooling

### Git Safety

!!! warning "Forbidden git operations"
    The following are prohibited in this repo (and pre-configured as
    `permissions.deny` in `.claude/settings.local.json`):

    - `git push --force` / `git push -f` / `git push --force-with-lease`
    - `git push origin master` / `git push origin main` (use a PR)
    - `git commit --no-verify` / `git push --no-verify`
    - `git rebase` / `git pull --rebase`

    Use plain `git push`, plain `git pull`, and `git merge origin/master`
    if you need to integrate upstream changes.

## Documentation

### Building docs locally

```bash
# Serve at http://127.0.0.1:8000/
uv run mkdocs serve --config-file autoware_lanelet2_to_opendrive/mkdocs.yml

# One-shot build to ./site
uv run mkdocs build --config-file autoware_lanelet2_to_opendrive/mkdocs.yml
```

### Documentation structure

- `docs/index.md` — landing page
- `docs/installation.md`, `docs/usage.md`, `docs/development.md`
- `docs/api.md` — auto-generated via `mkdocstrings`
- `docs/conversion-process.md` — pipeline internals
- `docs/carla_opendrive_lanelet2_mapping.md` — OpenDRIVE-tag ↔ CARLA-parser
  reference table
- `docs/signals.md`, `docs/crosswalk_objects.md`, `docs/stop_line_objects.md`
- `docs/limitations/` — known limitation pages, plus an overview
- `docs/image/`, `docs/stylesheets/` — assets (do not edit casually)

When adding a new page, also register it in
[`mkdocs.yml`'s `nav` block](https://github.com/tier4/autoware_lanelet2_to_opendrive/blob/master/autoware_lanelet2_to_opendrive/mkdocs.yml).

## Architecture notes

The conversion pipeline is orchestrated by
[`_Lanelet2ToOpenDRIVEConverter.convert()`](https://github.com/tier4/autoware_lanelet2_to_opendrive/blob/master/autoware_lanelet2_to_opendrive/src/autoware_lanelet2_to_opendrive/main.py)
in `main.py`. See [Conversion Process](conversion-process.md) for the
step-by-step trace and [API Reference](api.md) for module-level
documentation.

Design principles followed across the codebase:

- **Type safety**: every public surface is annotated and `py.typed` is
  shipped
- **Modularity**: parsing, preprocessing, geometry fitting, OpenDRIVE
  emission, and validation each live in their own module
- **Configuration via dataclasses**: dynamic knobs in
  `conversion_config.py`, numerical tolerances in `config.py`'s
  `DEFAULT_CONFIG`
- **Testability**: pure helper functions are kept free of lanelet2
  imports where practical; integration tests use the small fixtures in
  `autoware_lanelet2_to_opendrive/test/data/`

## Testing strategy

- **Unit tests** — isolated geometry, spline, projection, util tests
- **Lane / road / junction integration tests** — exercise
  `Road.construct_*`, `Junction.construct_*`, and the lane-link
  resolution against canned fixtures
- **End-to-end tests** — `test_main.py` runs the full conversion on
  small `.osm` fixtures
- **Fixture locking** — `test_xodr_fixture_locking.py` guards golden
  outputs against accidental drift

## Release process

Versioning is driven by the version-bump label on each merged PR. The
`Check Version Bump Label` action enforces that exactly one of
`bump patch` / `bump minor` / `bump major` is set; CI uses that label
to bump the version in
`autoware_lanelet2_to_opendrive/pyproject.toml`. The package is published
through the standard GitHub release workflow associated with each tag.

## Resources

### Related projects

- [Lanelet2](https://github.com/fzi-forschungszentrum-informatik/Lanelet2)
  — source format library
- [Autoware](https://github.com/autowarefoundation/autoware) — target
  platform
- [tier4/lanelet2_python_api_for_autoware](https://github.com/tier4/lanelet2_python_api_for_autoware)
  — Lanelet2 Python wrapper used at runtime
- [CARLA](https://carla.org/) — primary OpenDRIVE consumer

### Standards and specifications

- [OpenDRIVE 1.4 specification](https://www.asam.net/standards/detail/opendrive/)
- [PEP 561](https://peps.python.org/pep-0561/) — type hints in packages
- [Google Python Style Guide](https://google.github.io/styleguide/pyguide.html)
