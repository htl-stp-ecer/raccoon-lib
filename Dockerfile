FROM python:3.13-trixie
ARG DEBIAN_FRONTEND=noninteractive
ARG TZ=Europe/Vienna
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ >/etc/timezone
# Install system dependencies for building Python extensions
RUN apt-get update && apt-get install -y --no-install-recommends \
      build-essential cmake ninja-build git pkg-config ccache \
      libssl-dev libffi-dev \
      liblcm-dev \
    && rm -rf /var/lib/apt/lists/*
# Upgrade pip and install build tools
RUN pip install --no-cache-dir --upgrade pip setuptools wheel
# Install Python build dependencies
RUN pip install --no-cache-dir \
    scikit-build-core[pyproject] \
    pybind11 \
    pybind11-stubgen \
    "mypy>=1.15" \
    build \
    pytest \
    ruff==0.7.3
ENV CCACHE_DIR=/ccache \
    CCACHE_MAXSIZE=3G \
    CCACHE_COMPRESS=1 \
    PYTHONUNBUFFERED=1 \
    PYTHONDONTWRITEBYTECODE=1
# Trust bind-mounted source tree and its CMake FetchContent checkouts
RUN git config --global --add safe.directory '*'
WORKDIR /src
CMD ["/bin/bash"]
