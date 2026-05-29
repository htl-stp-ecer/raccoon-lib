FROM python:3.13-trixie

ARG DEBIAN_FRONTEND=noninteractive
ARG TZ=Europe/Vienna

RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ >/etc/timezone

RUN apt-get update && apt-get install -y --no-install-recommends \
      build-essential \
      cargo \
      ccache \
      clang \
      cmake \
      git \
      libacl1-dev \
      libclang-19-dev \
      libffi-dev \
      libglib2.0-dev \
      liblcm-dev \
      libssl-dev \
      llvm-dev \
      ninja-build \
      pkg-config \
      rustc \
    && rm -rf /var/lib/apt/lists/*

# iceoryx2's build-time bindgen needs libclang.so (unversioned). Debian
# only ships libclang-NN.so — add the symlink that clang-sys looks for.
RUN ln -sf /usr/lib/aarch64-linux-gnu/libclang-19.so.19 \
           /usr/lib/aarch64-linux-gnu/libclang.so 2>/dev/null \
    || ln -sf /usr/lib/x86_64-linux-gnu/libclang-19.so.19 \
              /usr/lib/x86_64-linux-gnu/libclang.so 2>/dev/null \
    || true

RUN pip install --no-cache-dir --upgrade pip setuptools wheel

RUN pip install --no-cache-dir \
    build \
    "mypy>=1.15" \
    pybind11 \
    pybind11-stubgen \
    pytest \
    pytest-asyncio \
    ruff==0.7.3 \
    scikit-build-core[pyproject]

ENV CCACHE_DIR=/ccache \
    CCACHE_MAXSIZE=3G \
    CCACHE_COMPRESS=1 \
    PYTHONUNBUFFERED=1 \
    PYTHONDONTWRITEBYTECODE=1

RUN git config --global --add safe.directory '*'

WORKDIR /src
CMD ["/bin/bash"]
