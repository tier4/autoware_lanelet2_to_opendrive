# syntax=docker/dockerfile:1.7
#
# Container image for a scenario package generated with `scenario-new`.
# Build context: the trimmed tree that assemble-context.sh next to this file
# produces (`framework/` = the uv workspace, `scenario/` = the package).
#
# Three stages, each handing the next strictly less than it was given: wheels
# are built on Alpine (everything here is pure Python), installed into a
# throwaway Debian stage, and only the finished virtualenv reaches the runtime.
# The runtime cannot be Alpine -- the CARLA client, opencv-python-headless and
# simple-lanelet2 publish manylinux wheels only, and the CARLA extension module
# links glibc 2.34+ symbols directly.
#
# See autoware_carla_scenario/docs/docker.md for the reasoning and the sizes.

ARG PYTHON_VERSION=3.10
ARG UV_VERSION=0.9.7
ARG RUNTIME_BASE=python:${PYTHON_VERSION}-slim-bookworm


# ---------------------------------------------------------------------------
# Stage 1: wheelhouse -- Alpine, pure Python, no native toolchain
# ---------------------------------------------------------------------------
FROM ghcr.io/astral-sh/uv:${UV_VERSION}-python${PYTHON_VERSION}-alpine AS wheelhouse

ENV UV_LINK_MODE=copy \
    UV_PYTHON_DOWNLOADS=never

WORKDIR /build

# The framework is pinned by the action ref, so it changes far less often than
# the scenario package; building it first keeps its layer cached across
# scenario edits. `--all-packages` builds whatever workspace members the
# assembler copied, so the member list lives in exactly one place.
COPY framework/ framework/
RUN --mount=type=cache,target=/root/.cache/uv \
    uv build --wheel --all-packages --out-dir /wheels framework \
    && uv export --directory framework --frozen --no-dev --no-emit-workspace \
        --no-hashes -o /wheels/constraints.txt

COPY scenario/ scenario/
RUN --mount=type=cache,target=/root/.cache/uv \
    uv build --wheel --out-dir /wheels scenario


# The uv binary is lifted out of its own distroless image rather than
# installed, so nothing about it lingers in a layer of the runtime.
FROM ghcr.io/astral-sh/uv:${UV_VERSION} AS uv


# ---------------------------------------------------------------------------
# Stage 2: venv -- resolve, install, and slim /opt/venv
# ---------------------------------------------------------------------------
FROM ${RUNTIME_BASE} AS venv

# The CARLA client version is fixed here, at build time, and never resolved
# again: the runtime image contains exactly this client and no way to change it.
ARG CARLA_VERSION=0.10.0
# Set to anything but "true" to keep debug symbols and vendored test suites --
# useful when debugging a native crash inside the image with gdb.
ARG SLIM=true

# `strip` comes from binutils. Installed before anything volatile is copied in
# so the apt layer caches across every rebuild; the stage is discarded, so
# installing it even when SLIM is off costs the image nothing.
RUN apt-get update \
    && apt-get install -y --no-install-recommends binutils \
    && rm -rf /var/lib/apt/lists/*

COPY --from=uv /uv /usr/local/bin/uv
COPY --from=wheelhouse /wheels /wheels
# The vendored CARLA wheels stay in their own --find-links directory: merged
# into /wheels they would be installed by the glob below even when
# CARLA_VERSION names a different release.
COPY framework/carla_wheels /carla-wheels

ENV UV_LINK_MODE=copy \
    UV_PYTHON_DOWNLOADS=never

# The constraints file is the workspace lock, so every framework dependency is
# pinned to the version this repository tests -- not just the CARLA client.
# Constraints only bound what gets resolved, so a scenario package that pulls
# in extra dependencies of its own still installs.
RUN --mount=type=cache,target=/root/.cache/uv \
    uv venv --python /usr/local/bin/python3 /opt/venv \
    && uv pip install --python /opt/venv/bin/python \
        --constraint /wheels/constraints.txt \
        --find-links /wheels \
        --find-links /carla-wheels \
        "carla==${CARLA_VERSION}" \
        /wheels/*.whl

# Drops what only a test runner, compiler or type checker reads, then strips
# the bundled shared objects. The script verifies each stripped object and puts
# the original back if binutils produced a layout the ELF loader would reject
# -- see its docstring; scipy's bundled OpenBLAS is one such object under the
# binutils in Debian bookworm.
COPY slim-venv.py /tmp/slim-venv.py
RUN set -eu; \
    if [ "${SLIM}" = "true" ]; then \
        /opt/venv/bin/python /tmp/slim-venv.py /opt/venv; \
    fi

# Fail the build here rather than at run time if the client is not the version
# that was asked for, or if slimming broke an extension module.
RUN /opt/venv/bin/python -c "import carla, cv2, lanelet2, numpy, scipy.interpolate, shapely; from importlib.metadata import version; got = version('carla'); assert got == '${CARLA_VERSION}', 'expected CARLA ${CARLA_VERSION}, installed ' + got; print('CARLA ' + got + ' pinned; native extension modules import cleanly')"


# ---------------------------------------------------------------------------
# Stage 3: runtime -- the venv and nothing else
# ---------------------------------------------------------------------------
FROM ${RUNTIME_BASE} AS runtime

ARG CARLA_VERSION=0.10.0
# ffmpeg is only invoked by CameraRecorder, and it is the largest thing that
# can be added to this image, so it stays opt-in. With it off the runtime
# installs no packages and carries no apt layer: after the move to
# opencv-python-headless the venv needs nothing beyond the glibc, libgcc_s,
# libstdc++ and libz the base image already has.
ARG WITH_FFMPEG=false

RUN set -eu; \
    if [ "${WITH_FFMPEG}" = "true" ]; then \
        apt-get update; \
        apt-get install -y --no-install-recommends ffmpeg; \
        rm -rf /var/lib/apt/lists/*; \
    fi

# --user-group makes the matching group explicit rather than relying on
# Debian's USERGROUPS_ENAB default, which `install -o/-g` below depends on.
RUN useradd --create-home --user-group --uid 1000 scenario \
    && install -d -o scenario -g scenario /work

COPY --from=venv /opt/venv /opt/venv

ENV PATH=/opt/venv/bin:$PATH \
    VIRTUAL_ENV=/opt/venv \
    PYTHONDONTWRITEBYTECODE=1 \
    PYTHONUNBUFFERED=1 \
    HYDRA_FULL_ERROR=1

LABEL org.opencontainers.image.title="autoware_carla_scenario scenario package" \
      org.opencontainers.image.description="A generated scenario package with a pinned CARLA client" \
      org.opencontainers.image.source="https://github.com/tier4/autoware_lanelet2_to_opendrive" \
      io.autoware.carla-scenario.carla-version="${CARLA_VERSION}"

USER scenario
# Hydra writes its run directory relative to the working directory, so mount a
# volume here to keep scenario outputs after the container exits.
WORKDIR /work

ENTRYPOINT ["scenario"]
CMD ["--help"]
