#!/usr/bin/env bash
# Humanoid demo: biped sim + fusion + viz + bridge -> Foxglove, with a split-screen
# utilisation layout.
#
#   pixi run bash ./scripts/humanoid_demo.sh                 # g1 (default)
#   pixi run bash ./scripts/humanoid_demo.sh op3
#   pixi run bash ./scripts/humanoid_demo.sh apollo 45        # 45 s, then exit
#
# Requires: Pixi environment, built ROS workspace, Foxglove Studio on Windows.
#
# Loads foxglove/humanoid_demo.json: 3D on the left, software utilisation top-right,
# hardware utilisation bottom-right. See foxglove/README.md for panel setup notes.
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$ROOT"

ROBOT="${1:-g1}"
AUTO_SECONDS="${2:-0}"
DURATION="${DURATION:-30}"      # seconds of gait to bake for replay
case "$ROBOT" in
  g1|apollo|op3) ;;
  *) echo "ERROR: robot must be one of g1, apollo, op3 (got '$ROBOT')." >&2; exit 2 ;;
esac

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

# A baked trajectory, not the live IK. The biped IK is numpy DLS at 21.95 ms/sample
# -- a 46 Hz ceiling -- while the filter only needs 693 us of a 2000 us budget. The
# demo would otherwise run at ~40 Hz and make the utilisation panel look broken when
# the filter is healthy. See sim/otolith_sim/bake_replay.py.
REPLAY="/tmp/otolith_${ROBOT}_replay.npz"
if [[ ! -f "$REPLAY" ]]; then
  echo "Baking ${DURATION}s of $ROBOT gait -> $REPLAY (one-off, ~1 min)..."
  PYTHONPATH=sim python -m otolith_sim.bake_replay "$ROBOT" "$REPLAY" "$DURATION"
fi

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

echo "Starting sim (puppet, replay gait, 500 Hz)..."
PYTHONPATH=sim nohup python -m otolith_sim.sim_node \
  --robot "$ROBOT" --replay "$REPLAY" >/tmp/otolith_sim.log 2>&1 </dev/null &
SIM_PID=$!

echo "Starting viz stack..."
ros2 launch otolith_viz viz_launch.py >/tmp/otolith_viz.log 2>&1 </dev/null &
LAUNCH_PID=$!

# viz_launch starts fusion_node with default parameters, i.e. go2. Restart it with
# the right robot so the descriptor and sigma_leg match the log. Done as a separate
# process rather than a launch edit so go2_demo.sh keeps working untouched.
sleep 4
pkill -f fusion_node || true
sleep 1
echo "Starting fusion_node (robot=$ROBOT, sigma_leg from descriptor)..."
nohup ros2 run otolith_fusion fusion_node \
  --ros-args -p robot:="$ROBOT" -p use_robot_sigma_leg:=true \
  >/tmp/otolith_fusion.log 2>&1 </dev/null &
FUS_PID=$!

# Silicon-side figures for the bottom-right panel. A STATIC cost model of measured
# PPA plus a derived cycle estimate -- NOT a live Verilator co-simulation (its
# topic field 16 stays 0 until that lands). See sim/otolith_sim/hw_telemetry.py.
PYTHONPATH=sim nohup python -m otolith_sim.hw_telemetry --robot "$ROBOT" \
  >/tmp/otolith_hw.log 2>&1 </dev/null &
HW_PID=$!

trap '
  echo "Tearing down..."
  kill "$SIM_PID" "$LAUNCH_PID" "$FUS_PID" "$HW_PID" 2>/dev/null || true
  wait "$SIM_PID" "$LAUNCH_PID" "$FUS_PID" "$HW_PID" 2>/dev/null || true
  pkill -f "http.server 8000" 2>/dev/null || true
' EXIT INT TERM

echo "Waiting for nodes and bridge to start..."
sleep 4

echo "Up. Topics:"
ros2 topic list | grep otolith || true
echo ""
echo "Open Foxglove Studio (Windows) -> ws://localhost:8765"
echo "Load layout: foxglove/humanoid_demo.json"
echo "Left: 3D panel. Add URDF layer, Source=Topic /robot_description, frame base;"
echo "       Scene mesh-up-axis=Z-up, then restart Studio."
echo "       Paths: /otolith/gt_path (green) vs /otolith/est_path (orange)."
echo "Right top:     /otolith/perf  -- software utilisation (indices below)"
echo "Right bottom:  /otolith/perf_hw -- silicon cost (indices below)"
echo ""
echo "  /otolith/perf indices:"
echo "    [0] predict us    [1] update_legs us  [2] total us   [3] deadline us"
echo "    [4] duty cycle    [5] rows           [6] sigma_leg  [7] rate Hz"
echo "    [8] jitter p99 us"
echo ""
echo "  /otolith/perf_hw indices (STATIC cost model, not a live co-simulation):"
echo "    [0] predict Fmax MHz   [1] update-kernel Fmax MHz  [2] ldl us"
echo "    [3] budget us          [4] update as frac of budget [5] rows"
echo "    [8] update MACs        [9] of which Joseph (width-independent)"
echo "    [10-12] predict LUT / % / DSP   [14-15] fixed vs float pos RMSE"
echo "    [16] live co-simulation? 0 until wired"
echo ""
if [[ "$AUTO_SECONDS" -gt 0 ]]; then
  echo "Auto-demo: exiting after ${AUTO_SECONDS}s."
  sleep "$AUTO_SECONDS"
else
  echo "Press Ctrl+C to stop."
  wait "$SIM_PID"
fi