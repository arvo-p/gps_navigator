#!/usr/bin/env bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

echo "==================================================="
echo " Building GPS Embedded Simulation (Linux / Raylib)"
echo "==================================================="

make

echo ""
echo "==================================================="
echo " Build successful! Executable: ./gps_simulation"
echo " Run simulation with:  ./gps_simulation"
echo " Run unit tests with:  make test"
echo "==================================================="
