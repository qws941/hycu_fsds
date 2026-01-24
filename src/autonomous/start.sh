#!/bin/bash
set -e

# Load environment
if [ -f .env ]; then
    export $(grep -v '^#' .env | xargs)
fi

# Validate IP
if [ -z "$FSDS_HOST_IP" ] || [ "$FSDS_HOST_IP" = "YOUR_WINDOWS_IP_HERE" ]; then
    echo "ERROR: .env 파일에서 FSDS_HOST_IP를 설정하세요"
    echo "예: FSDS_HOST_IP=192.168.0.100"
    exit 1
fi

echo "=== FSDS Autonomous Driving ==="
echo "Windows IP: $FSDS_HOST_IP"
echo "Port: ${FSDS_PORT:-41451}"
echo ""

# Start containers
docker compose up -d
echo ""
docker compose ps
