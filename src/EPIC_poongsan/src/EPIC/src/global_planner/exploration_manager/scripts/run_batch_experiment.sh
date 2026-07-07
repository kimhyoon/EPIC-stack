#!/bin/bash
#
# Batch Experiment Runner for EPIC Drone Exploration
#
# Usage: ./run_batch_experiment.sh [iterations] [rth_delay_sec]
#   iterations:    Number of experiment iterations (default: 10)
#   rth_delay_sec: Seconds to explore before RTH (default: 180, use 10 for testing)
#
# Example:
#   ./run_batch_experiment.sh 10 180   # 10 iterations, 3 min exploration each
#   ./run_batch_experiment.sh 2 10     # 2 iterations, 10 sec (quick test)

set -e

# ============ Configuration ============
ITERATIONS=${1:-10}
RTH_DELAY=${2:-180}
ORIGIN_X=5.0
ORIGIN_Y=0.0
ORIGIN_Z=2.0
ORIGIN_YAW=0.0
FINISH_TIMEOUT=300  # Max seconds to wait for FINISH after RTH
STARTUP_TIMEOUT=120 # Max seconds to wait for WAIT_TRIGGER
COOLDOWN=5          # Seconds between iterations

# Workspace setup - find setup.bash by searching upward from script location
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
SETUP_FILE=""
SEARCH_DIR="$SCRIPT_DIR"
while [ "$SEARCH_DIR" != "/" ]; do
    if [ -f "$SEARCH_DIR/devel/setup.bash" ]; then
        SETUP_FILE="$SEARCH_DIR/devel/setup.bash"
        break
    fi
    SEARCH_DIR="$(dirname "$SEARCH_DIR")"
done

if [ -z "$SETUP_FILE" ]; then
    echo "[ERROR] Cannot find devel/setup.bash in any parent directory"
    exit 1
fi

source "$SETUP_FILE"

# ============ Output directory ============
TIMESTAMP=$(date +%Y%m%d_%H%M%S)
PACKAGE_DIR="$(rospack find epic_planner 2>/dev/null)"
if [ -z "$PACKAGE_DIR" ]; then
    PACKAGE_DIR="$(cd "$(dirname "$0")/.." && pwd)"
fi
EXPERIMENT_DIR="$PACKAGE_DIR/experiments/$TIMESTAMP"
mkdir -p "$EXPERIMENT_DIR"

echo "============================================================"
echo " EPIC Batch Experiment"
echo "============================================================"
echo "  Iterations:     $ITERATIONS"
echo "  RTH delay:      ${RTH_DELAY}s"
echo "  Origin:         ($ORIGIN_X, $ORIGIN_Y, $ORIGIN_Z)"
echo "  Output:         $EXPERIMENT_DIR"
echo "  Finish timeout: ${FINISH_TIMEOUT}s"
echo "============================================================"

# ============ Helper functions ============

wait_for_state() {
    local target="$1"
    local timeout="$2"
    local label="$3"  # e.g. "iter 01"
    local elapsed=0
    local bar_width=30

    while [ $elapsed -lt $timeout ]; do
        local state
        state=$(timeout 3 rostopic echo /planning/state/text -n 1 2>/dev/null | head -1 | tr -d '"' || true)
        if [ "$state" = "$target" ]; then
            printf "\r[$label] Waiting for %s: [%s] done!%s\n" \
                "$target" \
                "$(printf '#%.0s' $(seq 1 $bar_width))" \
                "          "
            return 0
        fi
        elapsed=$((elapsed + 4))
        local pct=$((elapsed * 100 / timeout))
        [ $pct -gt 100 ] && pct=100
        local filled=$((elapsed * bar_width / timeout))
        [ $filled -gt $bar_width ] && filled=$bar_width
        local empty=$((bar_width - filled))
        printf "\r[$label] Waiting for %s: [%s%s] %3d%% (%d/%ds)" \
            "$target" \
            "$(printf '#%.0s' $(seq 1 $filled 2>/dev/null))" \
            "$(printf '.%.0s' $(seq 1 $empty 2>/dev/null))" \
            "$pct" "$elapsed" "$timeout"
        sleep 1
    done
    printf "\r[$label] Waiting for %s: TIMEOUT!%s\n" "$target" "$(printf ' %.0s' $(seq 1 40))"
    return 1
}

wait_for_roscore() {
    local timeout=30
    local elapsed=0
    while [ $elapsed -lt $timeout ]; do
        if rostopic list >/dev/null 2>&1; then
            return 0
        fi
        sleep 1
        elapsed=$((elapsed + 1))
    done
    echo "[ERROR] Timeout waiting for roscore"
    return 1
}

kill_ros() {
    # Kill roslaunch and all child processes
    if [ -n "$LAUNCH_PID" ] && kill -0 "$LAUNCH_PID" 2>/dev/null; then
        kill -INT "$LAUNCH_PID" 2>/dev/null || true
        sleep 2
        kill -9 "$LAUNCH_PID" 2>/dev/null || true
        wait "$LAUNCH_PID" 2>/dev/null || true
    fi
    # Clean up any remaining ROS nodes
    killall -9 rosmaster 2>/dev/null || true
    killall -9 roscore 2>/dev/null || true
    sleep 2
}

save_params() {
    local params_file="$EXPERIMENT_DIR/params.yaml"
    cat > "$params_file" << EOF
# Experiment Parameters
experiment:
  timestamp: "$TIMESTAMP"
  iterations: $ITERATIONS
  rth_delay_sec: $RTH_DELAY
  origin: [$ORIGIN_X, $ORIGIN_Y, $ORIGIN_Z, $ORIGIN_YAW]
  finish_timeout_sec: $FINISH_TIMEOUT

# Config file content
EOF
    # Append garage.yaml content
    local config_file
    config_file="$(rospack find epic_planner 2>/dev/null)/config/garage.yaml"
    if [ -f "$config_file" ]; then
        echo "# --- garage.yaml ---" >> "$params_file"
        cat "$config_file" >> "$params_file"
    fi
    echo "[INFO] Parameters saved to: $params_file"
}

# ============ Save parameters ============
save_params

# ============ Main loop ============
SUCCESS_COUNT=0
FAIL_COUNT=0

for i in $(seq 1 "$ITERATIONS"); do
    ITER_NUM=$(printf "%02d" "$i")
    ITER_DIR="$EXPERIMENT_DIR/iter_$ITER_NUM"
    mkdir -p "$ITER_DIR"

    echo ""
    echo "============================================================"
    echo " Iteration $i / $ITERATIONS"
    echo "============================================================"

    # 1. Launch simulation
    echo "[iter $ITER_NUM] Starting roslaunch..."
    roslaunch epic_planner garage.launch \
        eval_output_dir:="$ITER_DIR" \
        eval_auto_shutdown:=true \
        > "$ITER_DIR/roslaunch.log" 2>&1 &
    LAUNCH_PID=$!
    echo "[iter $ITER_NUM] Launch PID: $LAUNCH_PID"

    # 2. Wait for system ready (WAIT_TRIGGER state)
    if ! wait_for_state "WAIT_TRIGGER" "$STARTUP_TIMEOUT" "iter $ITER_NUM"; then
        echo "[iter $ITER_NUM] FAILED: System did not reach WAIT_TRIGGER"
        FAIL_COUNT=$((FAIL_COUNT + 1))
        kill_ros
        sleep "$COOLDOWN"
        continue
    fi
    # Small delay to ensure all nodes are fully initialized
    sleep 2

    # 3. Trigger exploration
    echo "[iter $ITER_NUM] Triggering exploration..."
    timeout 10 rostopic pub -1 /waypoint_generator/waypoints nav_msgs/Path \
        "header: {frame_id: 'world'}
poses:
  - pose:
      position: {x: 20.0, y: 20.0, z: 2.0}" \
        2>/dev/null || echo "[WARN] Trigger publish timed out"
    echo -n "[iter $ITER_NUM] Exploring: "

    # 4. Wait RTH_DELAY with progress bar
    BAR_WIDTH=30
    for sec in $(seq 1 "$RTH_DELAY"); do
        pct=$((sec * 100 / RTH_DELAY))
        filled=$((sec * BAR_WIDTH / RTH_DELAY))
        empty=$((BAR_WIDTH - filled))
        printf "\r[iter %s] Exploring: [%s%s] %3d%% (%d/%ds)" \
            "$ITER_NUM" \
            "$(printf '#%.0s' $(seq 1 $filled 2>/dev/null))" \
            "$(printf '.%.0s' $(seq 1 $empty 2>/dev/null))" \
            "$pct" "$sec" "$RTH_DELAY"
        sleep 1
    done
    echo ""

    # 5. Call RTH
    echo "[iter $ITER_NUM] Calling RTH to origin ($ORIGIN_X, $ORIGIN_Y, $ORIGIN_Z)..."
    timeout 10 rosservice call /srv_goto \
        "x: $ORIGIN_X
y: $ORIGIN_Y
z: $ORIGIN_Z
yaw: $ORIGIN_YAW" 2>/dev/null || echo "[WARN] RTH service call failed or timed out"

    # 6. Wait for FINISH
    if wait_for_state "FINISH" "$FINISH_TIMEOUT" "iter $ITER_NUM"; then
        echo "[iter $ITER_NUM] FINISH detected!"
        # Wait for evaluate_exploration.py to save CSVs and auto-shutdown
        sleep 5
        SUCCESS_COUNT=$((SUCCESS_COUNT + 1))
    else
        echo "[iter $ITER_NUM] FAILED: FINISH not reached within timeout"
        FAIL_COUNT=$((FAIL_COUNT + 1))
    fi

    # 7. Kill roslaunch
    echo "[iter $ITER_NUM] Shutting down..."
    kill_ros

    # 8. Verify output files
    if [ -f "$ITER_DIR/summary.csv" ]; then
        echo "[iter $ITER_NUM] Results saved to $ITER_DIR/"
        # Print summary
        tail -1 "$ITER_DIR/summary.csv"
    else
        echo "[iter $ITER_NUM] WARNING: summary.csv not found in $ITER_DIR/"
    fi

    # 9. Cooldown before next iteration
    if [ "$i" -lt "$ITERATIONS" ]; then
        echo "[iter $ITER_NUM] Cooling down for ${COOLDOWN}s..."
        sleep "$COOLDOWN"
    fi
done

# ============ Final report ============
echo ""
echo "============================================================"
echo " Experiment Complete"
echo "============================================================"
echo "  Total iterations:  $ITERATIONS"
echo "  Successful:        $SUCCESS_COUNT"
echo "  Failed:            $FAIL_COUNT"
echo "  Results:           $EXPERIMENT_DIR"
echo ""

# List all summary files
echo "  Per-iteration summaries:"
for iter_dir in "$EXPERIMENT_DIR"/iter_*/; do
    if [ -f "$iter_dir/summary.csv" ]; then
        iter_name=$(basename "$iter_dir")
        error=$(head -2 "$iter_dir/summary.csv" | tail -1 | cut -d',' -f3)
        speed=$(head -2 "$iter_dir/summary.csv" | tail -1 | cut -d',' -f4)
        rmse=$(head -2 "$iter_dir/summary.csv" | tail -1 | cut -d',' -f5)
        rate=$(head -2 "$iter_dir/summary.csv" | tail -1 | cut -d',' -f6)
        echo "    $iter_name: error=${error}m, speed=${speed}m/s, rmse=${rmse}m, expl_rate=${rate}"
    fi
done

echo "============================================================"
