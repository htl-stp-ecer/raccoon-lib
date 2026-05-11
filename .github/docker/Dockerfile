FROM python:3.13-trixie

ARG DEBIAN_FRONTEND=noninteractive
ARG TZ=Europe/Vienna

RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ >/etc/timezone

RUN apt-get update && apt-get install -y --no-install-recommends \
      build-essential \
      ccache \
      cmake \
      git \
      libffi-dev \
      liblcm-dev \
      libssl-dev \
      ninja-build \
      pkg-config \
    && rm -rf /var/lib/apt/lists/*

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
