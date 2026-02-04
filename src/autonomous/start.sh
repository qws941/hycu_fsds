#!/bin/bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$(dirname "$SCRIPT_DIR")")"
FSDS_DIR="$PROJECT_ROOT/src/simulator/fsds-linux"

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

cleanup() {
    echo -e "\n${YELLOW}Shutting down...${NC}"
    docker compose down 2>/dev/null || true
    pkill -f "FSOnline/Binaries/Linux/Blocks" 2>/dev/null || true
    exit 0
}
trap cleanup SIGINT SIGTERM

detect_ip() {
    ip -4 addr show | grep -oP '(?<=inet\s)\d+(\.\d+){3}' | grep -v '^127\.' | grep -v '^172\.' | head -1 || echo "127.0.0.1"
}

HOST_IP=$(detect_ip)

mkdir -p "$HOME/.airsim"
cp "$FSDS_DIR/settings.json" "$HOME/.airsim/settings.json"

cat > "$SCRIPT_DIR/.env" << EOF
FSDS_HOST_IP=$HOST_IP
FSDS_PORT=41451
EOF

# Map selection: default to TrainingMap, can override with argument
# Available maps: TrainingMap, CompetitionMap1, CompetitionMap2, CompetitionMap3, Skidpad
MAP_NAME="${1:-TrainingMap}"

echo "=== FSDS Autonomous Driving (Linux) ==="
echo "Host IP: $HOST_IP"
echo "Map: $MAP_NAME"
echo ""

# Auto-detect display (prefer existing DISPLAY, fallback to :99, then :0)
if [ -z "$DISPLAY" ]; then
    if [ -e /tmp/.X11-unix/X99 ]; then
        export DISPLAY=:99
    else
        export DISPLAY=:0
    fi
fi
echo "Using DISPLAY=$DISPLAY"
sudo cat /var/run/lightdm/root/:0 2>/dev/null | xauth merge - 2>/dev/null || true
xhost +local:root 2>/dev/null || true

# Force NVIDIA GPU rendering (prevents llvmpipe/CPU fallback)
export __NV_PRIME_RENDER_OFFLOAD=1
export __GLX_VENDOR_LIBRARY_NAME=nvidia
export __VK_LAYER_NV_optimus=NVIDIA_only
export VK_ICD_FILENAMES=/usr/share/vulkan/icd.d/nvidia_icd.json

echo -e "${GREEN}[1/3]${NC} Starting FSDS simulator (GPU: NVIDIA forced)..."
chmod +x "$FSDS_DIR/FSDS.sh"
cd "$FSDS_DIR" && ./FSDS.sh "/Game/$MAP_NAME" -vulkan &
FSDS_PID=$!
sleep 20

echo -e "${GREEN}[2/3]${NC} Starting ROS containers..."
cd "$SCRIPT_DIR"
docker compose up -d --build

echo -e "${GREEN}[3/3]${NC} Waiting for driver to start..."
sleep 5

echo ""
echo -e "${GREEN}=== All systems running ===${NC}"
echo "  FSDS PID: $FSDS_PID"
echo "  Logs: docker compose logs -f autonomous"
echo "  Stop: Ctrl+C"
echo ""

docker compose logs -f autonomous
