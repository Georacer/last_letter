# Checks that the Python distribution installs on a clean system.
#
# Build from the repository root, after `uv build --sdist`:
#
#   docker build -f lib/tools/Dockerfile -t last-letter-sdist .
#   docker build -f lib/tools/Dockerfile --build-arg BASE_IMAGE=debian:12 .
#
# A successful `docker build` is the pass condition and nothing has to be run
# afterwards.
ARG BASE_IMAGE=ubuntu:22.04
FROM ${BASE_IMAGE}

ARG DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
        build-essential \
        ca-certificates \
        git \
        libboost-dev \
        libeigen3-dev \
        liblz4-dev \
        libzstd-dev \
        python3 \
        python3-dev \
        python3-venv \
    && rm -rf /var/lib/apt/lists/*

# Install into a virtualenv rather than the system interpreter: it is what a
# user should do anyway, and newer Debian and Ubuntu images refuse the latter
# outright (PEP 668).
ENV VIRTUAL_ENV=/opt/venv
RUN python3 -m venv ${VIRTUAL_ENV}
ENV PATH=${VIRTUAL_ENV}/bin:${PATH}
RUN pip install --no-cache-dir --upgrade pip

# Overridable so a specific sdist can be picked when dist/ holds several.
ARG SDIST=dist/*.tar.gz
COPY ${SDIST} /tmp/

# The compile happens here, and takes a few minutes.
RUN pip install --no-cache-dir /tmp/*.tar.gz

COPY lib/tools/sdist_smoke_test.py /tmp/
RUN python /tmp/sdist_smoke_test.py

CMD ["python"]
