#!/usr/bin/env bash
set -euo pipefail

# Build helper for the thermalcam sketch using Arduino CLI.
# Usage: ./scripts/arduino-build.sh [--fqbn <fqbn>] [--port <port>] [--flash]
# Defaults target an ESP32-S3 with PSRAM enabled; adjust as needed.

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export PATH="$REPO_ROOT/bin:$PATH"

FQBN="esp32:esp32:esp32s3"
PORT=""
DO_FLASH=false

while [[ $# -gt 0 ]]; do
  case "$1" in
    --fqbn)
      FQBN="$2"; shift 2;;
    --port)
      PORT="$2"; shift 2;;
    --flash)
      DO_FLASH=true; shift;;
    *)
      echo "Unknown arg: $1" >&2; exit 1;;
  esac
done

BUILD_PROPS=(
  --build-property build.psram=enabled
)
SKETCH_DIR="$REPO_ROOT"
CMD=(arduino-cli compile --fqbn "$FQBN" "${BUILD_PROPS[@]}" "$SKETCH_DIR")
SKETCH="$REPO_ROOT/sketch_dec6a.ino"
CMD=(arduino-cli compile --fqbn "$FQBN" "${BUILD_PROPS[@]}" "$SKETCH")

echo "Building with FQBN=$FQBN" >&2
if $DO_FLASH; then
  if [[ -z "$PORT" ]]; then
    echo "--flash requires --port <serialport>" >&2
    exit 1
  fi
  CMD=(arduino-cli compile --fqbn "$FQBN" "${BUILD_PROPS[@]}" --upload -p "$PORT" "$SKETCH_DIR")
fi

"${CMD[@]}"