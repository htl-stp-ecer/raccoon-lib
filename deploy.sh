#!/usr/bin/env bash
set -euo pipefail

PROJECT_NAME="${PROJECT_NAME:-libstp}"
BUILD_DIR="${BUILD_DIR:-build}"

REMOTE_USER="${RPI_USER:-pi}"
REMOTE_HOST="${RPI_HOST:-10.142.98.232}"
REMOTE_DIR="${RPI_DIR:-/home/pi/python-libs}"
PYTHON_CMD="${PYTHON_CMD:-python3}"

RED='\033[0;31m'; GREEN='\033[0;32m'; YELLOW='\033[1;33m'; BLUE='\033[0;34m'; NC='\033[0m'

echo -e "${GREEN}Deploying ${PROJECT_NAME} to Raspberry Pi...${NC}"
echo -e "${BLUE}Target: ${REMOTE_USER}@${REMOTE_HOST}:${REMOTE_DIR}${NC}"

echo -e "${YELLOW}Building Python wheel (containerized ARM toolchain)...${NC}"
bash ./build.sh

WHEEL_FILE=$(find "$BUILD_DIR" -name "*.whl" -type f | head -1)
if [[ ! -f "$WHEEL_FILE" ]]; then
  echo -e "${RED}Error: Wheel file not found after build in ${BUILD_DIR}${NC}"
  exit 1
fi

echo -e "${BLUE}Found wheel: $(basename "$WHEEL_FILE")${NC}"

echo -e "${GREEN}Testing SSH connection...${NC}"
if ! ssh -o ConnectTimeout=5 "${REMOTE_USER}@${REMOTE_HOST}" "echo 'SSH OK'"; then
  echo -e "${RED}Error: Cannot connect to ${REMOTE_USER}@${REMOTE_HOST}${NC}"
  echo -e "${YELLOW}Make sure:${NC}"
  echo -e "${YELLOW}  1. SSH is enabled on the Pi${NC}"
  echo -e "${YELLOW}  2. You can reach the Pi at ${REMOTE_HOST}${NC}"
  echo -e "${YELLOW}  3. SSH keys are set up or password auth available${NC}"
  echo -e "${YELLOW}Set environment variables if needed:${NC}"
  echo -e "${YELLOW}  export RPI_USER=your_username${NC}"
  echo -e "${YELLOW}  export RPI_HOST=your_pi_ip_or_hostname${NC}"
  echo -e "${YELLOW}  export RPI_DIR=/path/to/deploy/directory${NC}"
  exit 1
fi

echo -e "${GREEN}Creating remote directory...${NC}"
ssh "${REMOTE_USER}@${REMOTE_HOST}" "mkdir -p '${REMOTE_DIR}'"

echo -e "${GREEN}Copying wheel file...${NC}"
scp "${WHEEL_FILE}" "${REMOTE_USER}@${REMOTE_HOST}:${REMOTE_DIR}/"

echo -e "${GREEN}Installing Python package on target...${NC}"
WHEEL_BASENAME=$(basename "$WHEEL_FILE")
ssh "${REMOTE_USER}@${REMOTE_HOST}" "${PYTHON_CMD} -m pip install --user --force-reinstall '${REMOTE_DIR}/${WHEEL_BASENAME}' --break-system-packages"

# Test the installation
echo -e "${GREEN}Testing installation...${NC}"
if ssh "${REMOTE_USER}@${REMOTE_HOST}" "${PYTHON_CMD} -c 'import ${PROJECT_NAME}; print(f\"${PROJECT_NAME} successfully installed\")'"; then
  echo -e "${GREEN}✓ Package installation verified!${NC}"
else
  echo -e "${YELLOW}⚠ Package installed but import test failed${NC}"
fi

echo -e "${GREEN}Deployment complete!${NC}"
echo -e "${BLUE}Python package deployed and installed on: ${REMOTE_USER}@${REMOTE_HOST}${NC}"
echo -e "${YELLOW}To use: ssh ${REMOTE_USER}@${REMOTE_HOST} '${PYTHON_CMD} -c \"import ${PROJECT_NAME}\"'${NC}"
echo -e "${YELLOW}Wheel location: ${REMOTE_USER}@${REMOTE_HOST}:${REMOTE_DIR}/${WHEEL_BASENAME}${NC}"
