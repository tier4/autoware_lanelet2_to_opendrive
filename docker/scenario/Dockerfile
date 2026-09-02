# syntax=docker/dockerfile:1.7
#
# Container image for a scenario package generated with `scenario-new`.
#
# The build context is the trimmed tree produced by
# docker/scenario/assemble-context.sh:
#
#     framework/   the uv workspace (autoware_carla_scenario,
#                  autoware_lanelet2_to_opendrive, carla_wheels/)
#     scenario/    the generated scenario package
#
# Stages, ordered so that each one hands the next strictly less than it was
# given -- that is what keeps the final image down to "the code that runs":
#
#   wheelhouse (Alpine)  turns both trees into wheels. Every package here is
#                        pure Python (`uv build` emits py3-none-any), so musl
#                        is enough: no compiler, no CARLA, no glibc. Only the
#                        wheels leave this stage -- no sources, no uv, no
#                        pyproject.toml, no build cache.
#   venv (Debian slim)   installs those wheels plus the pinned CARLA client
#                        into a self-contained /opt/venv.
#   runtime (Debian slim) copies /opt/venv and the handful of shared libraries
#                        OpenCV needs. Nothing else: no uv, no wheels, no
#                        sources, no toolchain.
#
# Why the runtime is not Alpine: the CARLA client (both the vendored 0.10.0
# wheel and the PyPI 0.9.16 one), opencv-python and simple-lanelet2 publish
# manylinux wheels only. The CARLA extension module links glibc 2.34+
# symbols directly, and there is no musllinux wheel to fall back to, so a musl
# runtime cannot even install -- let alone import -- the client. Alpine is
# therefore used where it is actually free, in the wheel-building stage.

ARG PYTHON_VERSION=3.10
ARG DEBIAN_SUITE=bookworm
ARG UV_VERSION=0.9.7
ARG RUNTIME_BASE=python:${PYTHON_VERSION}-slim-${DEBIAN_SUITE}


# ---------------------------------------------------------------------------
# Stage 1: wheelhouse -- Alpine, pure Python, no native toolchain
# ---------------------------------------------------------------------------
FROM ghcr.io/astral-sh/uv:${UV_VERSION}-python${PYTHON_VERSION}-alpine AS wheelhouse

ENV UV_LINK_MODE=copy \
    UV_PYTHON_DOWNLOADS=never

WORKDIR /build

COPY framework/ framework/
COPY scenario/ scenario/

# Build the three wheels, then gather any vendored CARLA wheels alongside them.
# The mkdir keeps the gather (and the next stage's COPY) valid whether or not
# the context carried a wheel for a CARLA release that is not on PyPI.
RUN --mount=type=cache,target=/root/.cache/uv \
    uv build --wheel --out-dir /wheels --package autoware-carla-scenario framework \
    && uv build --wheel --out-dir /wheels --package autoware-lanelet2-to-opendrive framework \
    && uv build --wheel --out-dir /wheels scenario \
    && mkdir -p /carla-wheels framework/carla_wheels \
    && find framework/carla_wheels -maxdepth 1 -name '*.whl' -exec cp {} /carla-wheels/ \;


# The uv binary is lifted out of its own distroless image rather than
# installed, so nothing about it lingers in a layer of the runtime.
FROM ghcr.io/astral-sh/uv:${UV_VERSION} AS uv


# ---------------------------------------------------------------------------
# Stage 2: venv -- resolve and install into a relocatable /opt/venv
# ---------------------------------------------------------------------------
FROM ${RUNTIME_BASE} AS venv

# The CARLA client version is fixed here, at build time, and never resolved
# again: the runtime image contains exactly this client and no way to change it.
ARG CARLA_VERSION=0.10.0

COPY --from=uv /uv /usr/local/bin/uv
COPY --from=wheelhouse /wheels /wheels
COPY --from=wheelhouse /carla-wheels /carla-wheels

ENV UV_LINK_MODE=copy \
    UV_PYTHON_DOWNLOADS=never

# Byte-code caches are dropped rather than kept: the runtime runs as a non-root
# user that cannot write to /opt/venv, so they would never be refreshed. The
# final check fails the build if the client is not the version asked for.
RUN --mount=type=cache,target=/root/.cache/uv \
    uv venv --python /usr/local/bin/python3 /opt/venv \
    && uv pip install --python /opt/venv/bin/python \
        --find-links /wheels \
        --find-links /carla-wheels \
        "carla==${CARLA_VERSION}" \
        /wheels/*.whl \
    && find /opt/venv -depth -type d -name __pycache__ -exec rm -rf {} + \
    && /opt/venv/bin/python -c "import importlib.metadata as m; got = m.version('carla'); print('CARLA client pinned at ' + got); assert got == '${CARLA_VERSION}', 'expected CARLA ${CARLA_VERSION}, installed ' + got"


# ---------------------------------------------------------------------------
# Stage 3: runtime -- the venv, its shared libraries, and nothing else
# ---------------------------------------------------------------------------
FROM ${RUNTIME_BASE} AS runtime

ARG CARLA_VERSION=0.10.0
# opencv-python links libGL/libglib even when only decoding frames in memory;
# opencv-python-headless would drop these, but the framework depends on the
# full build. ffmpeg is only needed by CameraRecorder, so it stays opt-in.
ARG WITH_FFMPEG=false

RUN apt-get update \
    && apt-get install -y --no-install-recommends libgl1 libglib2.0-0 \
    && if [ "${WITH_FFMPEG}" = "true" ]; then \
        apt-get install -y --no-install-recommends ffmpeg; \
    fi \
    && rm -rf /var/lib/apt/lists/*

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
