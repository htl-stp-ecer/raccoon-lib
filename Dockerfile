# Builder for the Raspberry Pi wheels (aarch64). Used by build.py, which picks the stage with
# `docker build --target`.
#
#   cross   x86_64 image with the aarch64 cross compiler (a PC). Nothing is emulated while compiling:
#           aarch64-linux-gnu-g++ runs on the host CPU. Only finished programs run under qemu-user: the
#           C++ tests (ctest) and the Python that imports the finished module to write the type stubs.
#           No binfmt, no --privileged.
#   native  image for the machine it is built on (an arm64 machine, e.g. the CI's ARM runner), with the
#           normal compiler. Everything runs natively, no qemu.
FROM debian:13-slim AS base

ARG DEBIAN_FRONTEND=noninteractive
ARG TZ=Europe/Vienna
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ >/etc/timezone

# Tools that do not depend on the mode. liblcm-dev provides lcm-gen for code generation at build time.
RUN apt-get update && apt-get install -y --no-install-recommends \
      ca-certificates git cmake ninja-build ccache pkg-config liblcm-dev \
      python3 python3-dev python3-venv python3-pip \
    && rm -rf /var/lib/apt/lists/*

# Python build tools (host).
ENV VIRTUAL_ENV=/opt/venv
RUN python3 -m venv $VIRTUAL_ENV
ENV PATH=/opt/venv/bin:$PATH
RUN pip install --no-cache-dir 'scikit-build-core>=0.10' pybind11 build wheel setuptools \
      pybind11-stubgen 'mypy>=1.15' ruff==0.7.3 pytest pytest-asyncio numpy

ENV CCACHE_DIR=/ccache \
    CCACHE_MAXSIZE=3G \
    CCACHE_COMPRESS=1 \
    CCACHE_BASEDIR=/src \
    CCACHE_NOHASHDIR=1 \
    CCACHE_COMPILERCHECK=content \
    PYTHONUNBUFFERED=1 \
    PYTHONDONTWRITEBYTECODE=1

RUN git config --global --add safe.directory '*'
WORKDIR /src


FROM base AS native
# libglib2.0-dev is needed by the LCM the transport builds from source.
RUN apt-get update && apt-get install -y --no-install-recommends \
      gcc g++ libglib2.0-dev \
    && rm -rf /var/lib/apt/lists/*
CMD ["/bin/bash"]


FROM base AS cross
RUN dpkg --add-architecture arm64

# Cross toolchain and qemu-user. libglib2.0-dev:arm64 is needed by the LCM the transport builds from source.
RUN apt-get update && apt-get install -y --no-install-recommends \
      gcc-aarch64-linux-gnu g++-aarch64-linux-gnu binutils-aarch64-linux-gnu \
      libglib2.0-dev:arm64 qemu-user \
    && rm -rf /var/lib/apt/lists/*

# aarch64 Python 3.13: the headers to compile against, and a runtime for qemu-user.
# python3.13-minimal:arm64 cannot be installed (its postinst would run an ARM binary), so the
# packages are only downloaded and unpacked into /opt/arm64-root.
RUN apt-get update \
    && apt-get install -y --no-install-recommends --download-only \
         libpython3.13-dev:arm64 python3.13-minimal:arm64 libpython3.13-stdlib:arm64 \
         libsqlite3-0:arm64 libstdc++6:arm64 libgcc-s1:arm64 libglib2.0-0t64:arm64 \
    && mkdir -p /opt/arm64-root \
    && for deb in /var/cache/apt/archives/*_arm64.deb; do dpkg-deb --extract "$deb" /opt/arm64-root; done \
    && mkdir -p /usr/include/aarch64-linux-gnu \
    && cp -r /opt/arm64-root/usr/include/aarch64-linux-gnu/python3.13 /usr/include/aarch64-linux-gnu/ \
    && rm -rf /var/lib/apt/lists/* /var/cache/apt/archives/*.deb
CMD ["/bin/bash"]
