# syntax=docker/dockerfile:1.7
#
# Container image for a scenario package generated with `scenario-new`.
#
# The build context is the trimmed tree produced by the assemble-context.sh
# script next to this file:
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
#                        into a self-contained /opt/venv, then slims it: no
#                        byte-code caches, no vendored test suites, no C
#                        headers or type stubs, and every bundled shared
#                        object stripped of its symbol table.
#   runtime (Debian slim) copies /opt/venv and nothing else. After the switch
#                        to opencv-python-headless the only shared libraries
#                        the venv needs from outside are glibc, libgcc_s,
#                        libstdc++ and libz, all already in the base image, so
#                        the runtime installs no packages at all unless ffmpeg
#                        is asked for -- there is not even an apt layer.
#
# Why the runtime is not Alpine: the CARLA client (both the vendored 0.10.0
# wheel and the PyPI 0.9.16 one), opencv-python-headless and simple-lanelet2
# publish manylinux wheels only. The CARLA extension module links glibc 2.34+
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
# Stage 2: venv -- resolve, install, and slim /opt/venv
# ---------------------------------------------------------------------------
FROM ${RUNTIME_BASE} AS venv

# The CARLA client version is fixed here, at build time, and never resolved
# again: the runtime image contains exactly this client and no way to change it.
ARG CARLA_VERSION=0.10.0
# Set to anything but "true" to keep debug symbols and vendored test suites --
# useful when debugging a native crash inside the image with gdb.
ARG SLIM=true

COPY --from=uv /uv /usr/local/bin/uv
COPY --from=wheelhouse /wheels /wheels
COPY --from=wheelhouse /carla-wheels /carla-wheels

ENV UV_LINK_MODE=copy \
    UV_PYTHON_DOWNLOADS=never

RUN --mount=type=cache,target=/root/.cache/uv \
    uv venv --python /usr/local/bin/python3 /opt/venv \
    && uv pip install --python /opt/venv/bin/python \
        --find-links /wheels \
        --find-links /carla-wheels \
        "carla==${CARLA_VERSION}" \
        /wheels/*.whl

# Everything removed here is dead weight at run time. Byte-code caches are
# never refreshed (the runtime user cannot write to /opt/venv); vendored test
# suites, C headers, Cython sources and type stubs are only read by test
# runners, compilers and type checkers. `strip --strip-unneeded` leaves the
# dynamic symbol table intact, so the extension modules keep working -- the
# manylinux wheels for scipy, numpy and OpenCV simply ship unstripped.
# Together this is worth roughly 100 MB.
RUN set -eu; \
    if [ "${SLIM}" = "true" ]; then \
        site="$(/opt/venv/bin/python -c 'import sysconfig; print(sysconfig.get_paths()["purelib"])')"; \
        find "${site}" -depth -type d \
            \( -name __pycache__ -o -name tests -o -name include \) \
            -exec rm -rf {} +; \
        find "${site}" \
            \( -name '*.pyi' -o -name '*.pyx' -o -name '*.pxd' -o -name '*.a' \) \
            -delete; \
        apt-get update; \
        apt-get install -y --no-install-recommends binutils; \
        find "${site}" \( -name '*.so' -o -name '*.so.*' \) -print0 \
            | xargs -0 -r strip --strip-unneeded; \
    fi

# Fail the build here rather than at run time if the client is not the version
# that was asked for, or if slimming broke an extension module.
RUN /opt/venv/bin/python -c "import importlib.metadata as m; got = m.version('carla'); print('CARLA client pinned at ' + got); assert got == '${CARLA_VERSION}', 'expected CARLA ${CARLA_VERSION}, installed ' + got" \
    && /opt/venv/bin/python -c "import carla, cv2, lanelet2, numpy, scipy.interpolate, shapely; print('native extension modules import cleanly')"


# ---------------------------------------------------------------------------
# Stage 3: runtime -- the venv and nothing else
# ---------------------------------------------------------------------------
FROM ${RUNTIME_BASE} AS runtime

ARG CARLA_VERSION=0.10.0
# ffmpeg is only invoked by CameraRecorder, and it is the largest thing that
# can be added to this image, so it stays opt-in. With it off the runtime
# installs no packages and carries no apt layer.
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
