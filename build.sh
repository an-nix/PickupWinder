#!/bin/bash
set -e

echo "Building PickupWinder for esp32dev..."
platformio run -e esp32dev -v
echo "Build complete."
