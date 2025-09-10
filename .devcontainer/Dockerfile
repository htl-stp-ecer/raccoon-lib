FROM mcr.microsoft.com/devcontainers/cpp:1-ubuntu-24.04

ARG REINSTALL_CMAKE_VERSION_FROM_SOURCE="none"

COPY ./reinstall-cmake.sh /tmp/
RUN if [ "${REINSTALL_CMAKE_VERSION_FROM_SOURCE}" != "none" ]; then \
        chmod +x /tmp/reinstall-cmake.sh && /tmp/reinstall-cmake.sh ${REINSTALL_CMAKE_VERSION_FROM_SOURCE}; \
    fi && rm -f /tmp/reinstall-cmake.sh

ENV DEBIAN_FRONTEND=noninteractive \
    PIP_DISABLE_PIP_VERSION_CHECK=1 \
    PIP_NO_CACHE_DIR=1 \
    VIRTUAL_ENV_DISABLE_PROMPT=1

RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        python3 \
        python3-pip \
        python3-venv \
        python3-dev \
        build-essential \
        ca-certificates \
        git && \
    update-alternatives --install /usr/bin/python python /usr/bin/python3 10 && \
    update-alternatives --install /usr/bin/pip pip /usr/bin/pip3 10 && \
    rm -rf /var/lib/apt/lists/*

RUN cat >/etc/profile.d/auto-venv.sh <<'EOF'
[ -n "$DISABLE_AUTO_VENV" ] && return 0
case $- in
  *i*) : ;;
  *) return 0 ;;
esac

WS="${WORKSPACE_FOLDER:-}"
if [ -z "$WS" ]; then
  if echo "$PWD" | grep -q "^/workspaces/"; then
    WS="/workspaces/$(echo "$PWD" | cut -d/ -f3)"
  fi
fi

[ -z "$WS" ] && WS="$PWD"

[ -w "$WS" ] || return 0

VENV_PATH="$WS/.venv"
if [ ! -d "$VENV_PATH" ]; then
  # Create venv quietly
  python -m venv "$VENV_PATH" >/dev/null 2>&1 || return 0
  "$VENV_PATH/bin/python" -m pip install --upgrade pip >/dev/null 2>&1 || true
fi

# Avoid re-sourcing if already active
if [ -z "$VIRTUAL_ENV" ] || [ "$VIRTUAL_ENV" != "$VENV_PATH" ]; then
  # shellcheck disable=SC1090
  . "$VENV_PATH/bin/activate"
fi
EOF