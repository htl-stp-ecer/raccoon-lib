FROM --platform=linux/arm64/v8 python:3.12-slim-bookworm

ARG DEBIAN_FRONTEND=noninteractive
ARG TZ=Europe/Vienna
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ >/etc/timezone

# Install system dependencies for building Python extensions
RUN apt-get update && apt-get install -y --no-install-recommends \
      build-essential cmake ninja-build git pkg-config ccache \
      python3-dev python3-pip python3-venv \
      libssl-dev libffi-dev \
    && rm -rf /var/lib/apt/lists/*

# Upgrade pip and install build tools
RUN pip install --no-cache-dir --upgrade pip setuptools wheel

# Install Python build dependencies
RUN pip install --no-cache-dir \
    scikit-build-core[pyproject] \
    pybind11 \
    build \
    pytest

ENV CCACHE_DIR=/ccache \
    CCACHE_MAXSIZE=3G \
    CCACHE_COMPRESS=1 \
    PYTHONUNBUFFERED=1 \
    PYTHONDONTWRITEBYTECODE=1

WORKDIR /src

CMD ["/bin/bash"]