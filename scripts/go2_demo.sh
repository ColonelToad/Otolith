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
pkill -f "http.server 8000" || true
sleep 1

# Fail loudly if the ports we need are still bound (stale orphans).
for port in 8000 8765; do
  if python3 -c "import socket,sys; s=socket.socket(); s.settimeout(1); sys.exit(0 if s.connect_ex(('127.0.0.1',$port))==0 else 1)"; then
    echo "ERROR: port $port still in use after cleanup. Kill the stale process holding it, then re-run." >&2
    echo "Hint: ps aux | grep -E 'http.server 8000|foxglove_bridge|sim_node' | grep -v grep" >&2
    exit 1
  fi
done

echo "Starting sim (puppet)..."
PYTHONPATH=sim python -m otolith_sim.sim_node &
SIM_PID=$!

echo "Starting viz stack (robot_state_publisher + fusion + bridge + mesh_server)..."
ros2 launch otolith_viz viz_launch.py &
LAUNCH_PID=$!

trap '
  echo "Tearing down..."
  kill "$SIM_PID" "$LAUNCH_PID" 2>/dev/null || true
  wait "$SIM_PID" 2>/dev/null || true
  wait "$LAUNCH_PID" 2>/dev/null || true
  pkill -f "http.server 8000" 2>/dev/null || true
' EXIT INT TERM

echo "Waiting for nodes and bridge to start..."
sleep 3

echo "Up. Topics:"
ros2 topic list | grep otolith || true
echo ""

echo "Open Foxglove Studio (Windows) -> ws://localhost:8765"
echo "Load layout: foxglove/go2_demo.json"
echo "3D: add URDF layer, Source=Topic /robot_description, frame base; Scene mesh-up-axis=Z-up, then restart Studio"
echo "3D paths: /otolith/gt_path (green) vs /otolith/est_path (orange) — enable both in 3D > Topics; markers silent unless fusion publish_covariance:=true"
echo "Press Ctrl+C to stop (or wait 30s for auto-demo)..."

if [[ "${1:-}" == "--auto" ]]; then
  sleep 30
else
  wait "$SIM_PID"
fi