FROM python:3.11-slim-bookworm

ENV DEBIAN_FRONTEND=noninteractive \
    PIP_DISABLE_PIP_VERSION_CHECK=1 \
    PYTHONUNBUFFERED=1 \
    HOME=/tmp \
    XDG_CONFIG_HOME=/tmp/.config \
    XDG_CACHE_HOME=/tmp/.cache

RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        blender \
        ca-certificates \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /opt/ll2tofbx

COPY pyproject.toml README.md /opt/ll2tofbx/
COPY src /opt/ll2tofbx/src
COPY scripts /opt/ll2tofbx/scripts

RUN python -m pip install --upgrade pip setuptools wheel \
    && python -m pip install \
        lanelet2 \
        numpy \
        scipy \
        mapbox-earcut \
        shapely \
    && python -m pip install .

ENTRYPOINT ["ll2tofbx"]
CMD ["--help"]
