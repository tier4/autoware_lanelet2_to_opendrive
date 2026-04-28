# syntax=docker/dockerfile:1.7
# Python 3.10 is hardcoded throughout this Dockerfile because the prebuilt
# lanelet2 wheel published by tier4/lanelet2_python_api_for_autoware is
# CPython-3.10 ABI-tagged; bumping requires rebuilding that wheel upstream
# and updating the apt package, the venv path, and LD_LIBRARY_PATH below.
FROM ubuntu:22.04 AS base
ENV DEBIAN_FRONTEND=noninteractive \
    PYTHONUNBUFFERED=1 \
    UV_LINK_MODE=copy \
    UV_PYTHON_DOWNLOADS=never \
    LD_LIBRARY_PATH=/workspace/.venv/lib/python3.10/site-packages/lanelet2/lib
RUN apt-get update && apt-get install -y --no-install-recommends \
      python3.10 python3.10-venv python3-pip \
      build-essential cmake \
      libboost-dev libeigen3-dev libpugixml-dev libgeographic-dev \
      libboost-python-dev libboost-serialization-dev librange-v3-dev \
      libboost-filesystem-dev libboost-program-options-dev \
      libgl1 \
      git ca-certificates curl \
    && rm -rf /var/lib/apt/lists/*
COPY --from=ghcr.io/astral-sh/uv:0.9.7 /uv /uvx /usr/local/bin/
WORKDIR /workspace

# Single deps stage: copy full workspace sources and run a single uv sync.
# Splitting into separate runtime/dev sync stages was attempted but produced
# non-deterministic builds of lanelet2-python-api-for-autoware (the wrapper
# core.so/io.so binaries differed between parallel stage builds, causing
# import-time SystemError in some images). Building once with --dev is
# slower but reliable; convert reuses the same install.
#
# LD_LIBRARY_PATH (set in `base`) is required because the wrapper .so files
# are linked with an absolute RUNPATH pointing inside uv's build cache, which
# doesn't survive a COPY --from across stages.
FROM base AS deps
COPY pyproject.toml uv.lock .python-version ./
COPY autoware_lanelet2_to_opendrive/ autoware_lanelet2_to_opendrive/
COPY autoware_carla_scenario/ autoware_carla_scenario/
COPY carla_wheels/ carla_wheels/
# The git config containing the PAT is written and removed inside the same
# RUN layer so the token is never committed to the image filesystem. Do not
# split this into separate RUN commands.
RUN --mount=type=secret,id=gh_pat,required=false \
    if [ -s /run/secrets/gh_pat ]; then \
      git config --global url."https://$(cat /run/secrets/gh_pat)@github.com/".insteadOf "https://github.com/"; \
    fi && \
    uv sync --frozen --dev --extra carla && \
    rm -f /root/.gitconfig

FROM deps AS dev
ENV PATH="/workspace/.venv/bin:${PATH}"
CMD ["bash"]

FROM base AS convert
COPY --from=deps /workspace /workspace
ENV PATH="/workspace/.venv/bin:${PATH}"
WORKDIR /io
ENTRYPOINT ["convert"]
CMD ["--help"]
