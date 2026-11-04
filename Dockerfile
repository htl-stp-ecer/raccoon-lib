# Builder for the Raspberry Pi wheels (aarch64). Used by build.py.
#
# Cross compilation, no emulation of the compiler: aarch64-linux-gnu-g++ runs on the host CPU.
# Only the finished programs run under qemu-user: the C++ tests (ctest) and the Python that
# imports the finished module to write the type stubs. No binfmt, no --privileged.
FROM debian:13-slim

ARG DEBIAN_FRONTEND=noninteractive
ARG TZ=Europe/Vienna
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ >/etc/timezone

RUN dpkg --add-architecture arm64

# Host tools and cross toolchain. liblcm-dev provides /usr/bin/lcm-gen (x86_64) for code generation
# at build time; libglib2.0-dev:arm64 is needed by the LCM the transport builds from source.
RUN apt-get update && apt-get install -y --no-install-recommends \
      ca-certificates git cmake ninja-build ccache pkg-config \
      gcc-aarch64-linux-gnu g++-aarch64-linux-gnu binutils-aarch64-linux-gnu \
      liblcm-dev libglib2.0-dev:arm64 \
      python3 python3-dev python3-venv python3-pip \
      qemu-user \
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
CMD ["/bin/bash"]
