#!/bin/bash
# FSDS Race Recording Script
# Records the race from X11 display with proper GPU rendering

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$(dirname "$SCRIPT_DIR")")"
RECORDINGS_DIR="$PROJECT_ROOT/recordings"

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

# Create recordings directory
mkdir -p "$RECORDINGS_DIR"

# Output filename with timestamp
TIMESTAMP=$(date +%Y%m%d_%H%M%S)
OUTPUT_FILE="${1:-$RECORDINGS_DIR/fsds_race_$TIMESTAMP.mp4}"

# Recording settings - auto-detect display
if [ -z "$DISPLAY" ]; then
    if [ -e /tmp/.X11-unix/X99 ]; then
        DISPLAY_NUM=":99"
    else
        DISPLAY_NUM=":0"
    fi
else
    DISPLAY_NUM="$DISPLAY"
fi
VIDEO_SIZE="1280x720"
FRAMERATE="30"
OFFSET="0,40"  # Offset to skip window decorations

echo -e "${GREEN}=== FSDS Race Recorder ===${NC}"
echo "Output: $OUTPUT_FILE"
echo "Display: $DISPLAY_NUM"
echo "Resolution: $VIDEO_SIZE @ ${FRAMERATE}fps"
echo ""

# Check if FSDS is running
if ! pgrep -f "FSOnline/Binaries/Linux/Blocks" > /dev/null; then
    echo -e "${RED}Error: FSDS is not running!${NC}"
    echo "Start FSDS first with: ./start.sh"
    exit 1
fi

echo -e "${YELLOW}Recording started. Press Ctrl+C to stop.${NC}"
echo ""

# Start recording
# Uses x11grab for screen capture, libx264 for encoding
ffmpeg -y \
    -f x11grab \
    -video_size "$VIDEO_SIZE" \
    -framerate "$FRAMERATE" \
    -i "${DISPLAY_NUM}.0+${OFFSET}" \
    -c:v libx264 \
    -preset ultrafast \
    -crf 23 \
    -pix_fmt yuv420p \
    "$OUTPUT_FILE"

echo ""
echo -e "${GREEN}Recording saved: $OUTPUT_FILE${NC}"
