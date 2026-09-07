#!/usr/bin/env bash
set -euo pipefail

# Go2 demo: sim + fusion + viz + bridge -> Foxglove on :8765
# Run from repo root with:
#   pixi run bash ./scripts/go2_demo.sh
#
# Requires: Pixi environment, built ROS workspace, and Foxglove Studio on Windows.

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$ROOT"

echo "Building ROS workspace (if needed)..."
export LIBRARY_PATH="$CONDA_PREFIX/lib${LIBRARY_PATH:+:$LIBRARY_PATH}"
export LD_LIBRARY_PATH="$CONDA_PREFIX/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"

colcon build \
  --packages-select otolith_description otolith_fusion otolith_viz \
  --cmake-args "-DCMAKE_PREFIX_PATH=$CONDA_PREFIX" 2>&1 | tail -5

echo "Sourcing install..."
set +u
source install/setup.bash
set -u

echo "Cleaning up previous processes..."
pkill -f otolith_sim.sim_node || true
pkill -f fusion_node || true
pkill -f viz_node || true
pkill -f foxglove_bridge || true
sleep 1

echo "Starting sim (puppet)..."
PYTHONPATH=sim python -m otolith_sim.sim_node &
SIM_PID=$!

echo "Starting viz stack (robot_state_publisher + fusion + viz + bridge)..."
ros2 launch otolith_viz viz_launch.py &
LAUNCH_PID=$!

trap '
  echo "Tearing down..."
  kill "$SIM_PID" "$LAUNCH_PID" 2>/dev/null || true
  wait "$SIM_PID" 2>/dev/null || true
  wait "$LAUNCH_PID" 2>/dev/null || true
' EXIT INT TERM

echo "Waiting for nodes and bridge to start..."
sleep 3

echo "Up. Topics:"
ros2 topic list | grep otolith || true
echo ""

echo "Open Foxglove Studio (Windows) -> ws://localhost:8765"
echo "Load layout: foxglove/go2_demo.json"
echo "3D: URDF via /robot_description + TF world->base (est) + world->base_gt"
echo "Plots: /otolith/gt_path vs /otolith/est_path, /otolith/markers"
echo "Press Ctrl+C to stop (or wait 30s for auto-demo)..."

if [[ "${1:-}" == "--auto" ]]; then
  sleep 30
else
  wait "$SIM_PID"
fi