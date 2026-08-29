#!/bin/zsh

set -e

SCRIPT_DIR="$(cd "$(dirname "${(%):-%N}")" && pwd)"
SCENE="tunnel"
UPROJECT="/home/eason/Application/AirSim/Unreal/Environments/tunnel/FC-Vision_tunnel.uproject"
UE4_EDITOR="${UE4_EDITOR:-/home/eason/Application/UnrealEngine/Engine/Binaries/Linux/UE4Editor}"
ROS_SETUP="${ROS_SETUP:-${SCRIPT_DIR}/../devel/setup.zsh}"

SESSION="fc_vision"
STATE_DIR="${FC_VISION_STATE_DIR:-/tmp/fc_vision_demo}"
STATE_FILE="${STATE_DIR}/active.env"
UE_PID_FILE="${STATE_DIR}/ue.pid"
LOG_DIR="${SCRIPT_DIR}/../run_log"

AIRSIM_RPC_HOST="${AIRSIM_RPC_HOST:-127.0.0.1}"
AIRSIM_RPC_PORT="${AIRSIM_RPC_PORT:-41451}"
STARTUP_TIMEOUT="${STARTUP_TIMEOUT:-120}"
ACTIVATION_TIMEOUT="${ACTIVATION_TIMEOUT:-300}"
ACTIVATION_POLL_INTERVAL="${ACTIVATION_POLL_INTERVAL:-1}"
LIDAR_TOPIC="${LIDAR_TOPIC:-/airsim_node/drone_1/world_lidar_cloud}"
UE_KILL_TIMEOUT="${UE_KILL_TIMEOUT:-5}"
ATTACH_TMUX="${ATTACH_TMUX:-true}"

process_group_alive() {
    local PID="$1"
    kill -0 -- "-${PID}" 2>/dev/null || kill -0 "$PID" 2>/dev/null
}

signal_group() {
    local PID="$1"
    local SIGNAL="$2"
    kill "-${SIGNAL}" -- "-${PID}" 2>/dev/null || kill "-${SIGNAL}" "$PID" 2>/dev/null || true
}

stop_ue_process() {
    local PID="$1"
    local MARKED_PROJECT="$2"
    local PROCESS_ARGS
    local WAITED=0
    local MAX_POLLS=$((UE_KILL_TIMEOUT * 4))

    [[ "$PID" == <-> ]] || return 1
    process_group_alive "$PID" || return 1

    PROCESS_ARGS="$(ps -p "$PID" -o args= 2>/dev/null || true)"
    if [ -n "$MARKED_PROJECT" ] && [[ "$PROCESS_ARGS" != *"$MARKED_PROJECT"* ]]; then
        echo "[${SCENE}] PID=$PID does not belong to the recorded UE project; skip it."
        return 1
    fi

    echo "[${SCENE}] Stopping UE process group PID=$PID..."
    signal_group "$PID" TERM
    while process_group_alive "$PID"; do
        if [ "$WAITED" -ge "$MAX_POLLS" ]; then
            echo "[${SCENE}] UE did not stop after ${UE_KILL_TIMEOUT}s; force killing."
            signal_group "$PID" KILL
            break
        fi
        sleep 0.25
        WAITED=$((WAITED + 1))
    done
    return 0
}

stop_demo() {
    trap - EXIT INT TERM

    tmux kill-session -t "$SESSION" 2>/dev/null || true
    pkill -TERM -f roslaunch 2>/dev/null || true
    pkill -TERM -f '/devel/lib/airsim_ros_pkgs/airsim_node' 2>/dev/null || true
    pkill -TERM -f '/devel/lib/fc_vision_manager/fc_vision' 2>/dev/null || true
    pkill -TERM -f '/devel/lib/fc_vision_manager/traj_server' 2>/dev/null || true
    pkill -TERM -f '/devel/lib/px4ctrl/px4ctrl_node' 2>/dev/null || true
    pkill -TERM -f '/devel/lib/rgb_point_map/rgb_point_mapping_node' 2>/dev/null || true
    pkill -TERM -f roscore 2>/dev/null || true
    pkill -TERM -f rosmaster 2>/dev/null || true
    pkill -TERM -f rosout 2>/dev/null || true

    local MARKED_PROJECT="$UPROJECT"
    local UE_PID=""
    local HAS_STATE=false
    if [ -f "$STATE_FILE" ]; then
        HAS_STATE=true
        local STATE_PROJECT="$(sed -n 's/^uproject=//p' "$STATE_FILE" | head -n 1)"
        [ -z "$STATE_PROJECT" ] || MARKED_PROJECT="$STATE_PROJECT"
    fi
    [ ! -f "$UE_PID_FILE" ] || UE_PID="$(<"$UE_PID_FILE")"

    if ! stop_ue_process "$UE_PID" "$MARKED_PROJECT" && [ "$HAS_STATE" = "true" ] && [ -n "$MARKED_PROJECT" ]; then
        local MATCHED_PIDS="$(pgrep -f "UE4Editor.*${MARKED_PROJECT}" 2>/dev/null || true)"
        local PID
        for PID in ${(f)MATCHED_PIDS}; do
            stop_ue_process "$PID" "$MARKED_PROJECT" || true
        done
    fi

    rm -f "$UE_PID_FILE" "$STATE_FILE" /tmp/fc_vision_ue4.pid /tmp/fc_vision_ue4_autostarted
    rmdir "$STATE_DIR" 2>/dev/null || true
    echo "[${SCENE}] Demo stopped."
}

if [ "${1:-}" = "--stop" ]; then
    stop_demo
    exit 0
fi
if [ "$#" -ne 0 ]; then
    echo "Usage: $0 [--stop]"
    exit 2
fi

for FILE in "$UPROJECT" "$ROS_SETUP"; do
    if [ ! -f "$FILE" ]; then
        echo "[${SCENE}][ERROR] Required file not found: $FILE"
        exit 1
    fi
done
if [ ! -x "$UE4_EDITOR" ]; then
    echo "[${SCENE}][ERROR] UE4Editor is not executable: $UE4_EDITOR"
    exit 1
fi

source "$ROS_SETUP"
for COMMAND in grep nc pgrep pkill roslaunch rosnode rostopic setsid ss tail timeout tmux; do
    if ! command -v "$COMMAND" >/dev/null; then
        echo "[${SCENE}][ERROR] Missing command: $COMMAND"
        exit 1
    fi
done

if tmux has-session -t "$SESSION" 2>/dev/null || ss -ltn | grep -Eq ":(11311|${AIRSIM_RPC_PORT})\\b"; then
    echo "[${SCENE}][ERROR] A demo is already running. Stop it with ${SCRIPT_DIR}/kill_demo.sh."
    exit 1
fi

mkdir -p "$STATE_DIR" "$LOG_DIR"
TIMESTAMP="$(date +'%Y-%m-%d_%H-%M-%S')"
FC_LOG_FILE="${LOG_DIR}/fc_vision_${SCENE}_${TIMESTAMP}.log"
UE_LOG_FILE="${LOG_DIR}/ue4_${SCENE}_${TIMESTAMP}.log"
LAUNCH_COMPLETE=false

cleanup_failed_start() {
    local EXIT_CODE=$?
    if [ "$LAUNCH_COMPLETE" != "true" ]; then
        echo "[${SCENE}] Startup failed; cleaning partial runtime."
        stop_demo
    fi
    exit "$EXIT_CODE"
}
trap cleanup_failed_start EXIT
trap 'exit 130' INT TERM

UE_ARGS=("$UPROJECT" "-game" "-ResX=${UE_RESX:-1280}" "-ResY=${UE_RESY:-720}" "-log")
[ "${UE_WINDOWED:-true}" = "false" ] || UE_ARGS+=("-windowed")
[ "${UE_UNATTENDED:-true}" = "false" ] || UE_ARGS+=("-unattended")
[ "${UE_NO_SOUND:-false}" = "true" ] && UE_ARGS+=("-NoSound")
[ -z "${UE_EXTRA_ARGS:-}" ] || UE_ARGS+=(${(z)UE_EXTRA_ARGS})

echo "[${SCENE}] Starting UE: $UPROJECT"
setsid "$UE4_EDITOR" "${UE_ARGS[@]}" </dev/null >"$UE_LOG_FILE" 2>&1 &
UE_PID=$!
echo "$UE_PID" > "$UE_PID_FILE"
{
    echo "scene=$SCENE"
    echo "uproject=$UPROJECT"
    echo "ue_pid=$UE_PID"
} > "$STATE_FILE"
sleep 1
if ! process_group_alive "$UE_PID"; then
    echo "[${SCENE}][ERROR] UE exited during startup. Log: $UE_LOG_FILE"
    exit 1
fi

echo "[${SCENE}] Log: $FC_LOG_FILE" | tee -a "$FC_LOG_FILE"
tmux new-session -d -s "$SESSION"
tmux split-window -h -t "$SESSION:0"
tmux split-window -v -t "$SESSION:0.0"
tmux split-window -v -t "$SESSION:0.1"
tmux select-layout -t "$SESSION:0" tiled
tmux split-window -h -t "$SESSION:0.3"
tmux split-window -h -t "$SESSION:0.4"
tmux select-layout -t "$SESSION:0" tiled

WAIT_AIRSIM_CMD="echo '[AirSim] Waiting for RPC ${AIRSIM_RPC_HOST}:${AIRSIM_RPC_PORT} ...'; \
START_TIME=\$(date +%s); \
while ! nc -z ${AIRSIM_RPC_HOST} ${AIRSIM_RPC_PORT} 2>/dev/null; do \
  NOW=\$(date +%s); \
  if [ \$((NOW - START_TIME)) -ge ${STARTUP_TIMEOUT} ]; then \
    echo '[AirSim][ERROR] RPC startup timed out after ${STARTUP_TIMEOUT}s'; exit 1; \
  fi; sleep 1; \
done; echo '[AirSim] RPC ready.'"

WAIT_AIRSIM_ROS_CMD="source '$ROS_SETUP'; \
echo '[ROS] Waiting for /airsim_node ...'; \
START_TIME=\$(date +%s); \
until rosnode list 2>/dev/null | grep -Fxq '/airsim_node'; do \
  NOW=\$(date +%s); \
  if [ \$((NOW - START_TIME)) -ge ${STARTUP_TIMEOUT} ]; then \
    echo '[ROS][ERROR] /airsim_node startup timed out after ${STARTUP_TIMEOUT}s'; exit 1; \
  fi; sleep 1; \
done; echo '[ROS] /airsim_node ready.'"

tmux send-keys -t "$SESSION:0.0" "tail -F '$UE_LOG_FILE'" C-m
tmux send-keys -t "$SESSION:0.1" "$WAIT_AIRSIM_CMD && source '$ROS_SETUP' && roslaunch airsim_ros_pkgs airsim_node.launch; exec zsh" C-m
tmux send-keys -t "$SESSION:0.2" "$WAIT_AIRSIM_ROS_CMD && stdbuf -oL roslaunch fc_vision_manager ${SCENE}.launch 2>&1 | tee -a '$FC_LOG_FILE'; exec zsh" C-m
tmux send-keys -t "$SESSION:0.3" "$WAIT_AIRSIM_ROS_CMD && roslaunch fc_vision_manager rviz.launch; exec zsh" C-m
tmux send-keys -t "$SESSION:0.4" "$WAIT_AIRSIM_ROS_CMD && rosrun topic_tools throttle messages /airsim_node/drone_1/front_center/Scene/compressed 15 /FC_Vision/fpv/compressed; exec zsh" C-m
tmux send-keys -t "$SESSION:0.5" "$WAIT_AIRSIM_ROS_CMD && rosrun topic_tools throttle messages '$LIDAR_TOPIC' 10.0 /FC_Vision/world_cloud; exec zsh" C-m

ACTIVATION_START_TIME="$(date +%s)"
remaining_time() {
    local ELAPSED=$(($(date +%s) - ACTIVATION_START_TIME))
    if [ "$ELAPSED" -ge "$ACTIVATION_TIMEOUT" ]; then
        echo "[${SCENE}][ERROR] Activation timed out after ${ACTIVATION_TIMEOUT}s."
        exit 1
    fi
    echo $((ACTIVATION_TIMEOUT - ELAPSED))
}

wait_for_log() {
    local DESCRIPTION="$1"
    local PATTERN="$2"
    echo "[${SCENE}] Waiting for ${DESCRIPTION}..."
    while ! grep -Fq "$PATTERN" "$FC_LOG_FILE"; do
        remaining_time >/dev/null
        sleep "$ACTIVATION_POLL_INTERVAL"
    done
}

wait_for_log_regex() {
    local DESCRIPTION="$1"
    local PATTERN="$2"
    echo "[${SCENE}] Waiting for ${DESCRIPTION}..."
    while ! grep -Eq "$PATTERN" "$FC_LOG_FILE"; do
        remaining_time >/dev/null
        sleep "$ACTIVATION_POLL_INTERVAL"
    done
}

wait_for_node() {
    local NODE="$1"
    while ! rosnode list 2>/dev/null | grep -Fxq "$NODE"; do
        remaining_time >/dev/null
        sleep "$ACTIVATION_POLL_INTERVAL"
    done
}

wait_for_log "FC-Vision manager" "Initialized!"
echo "[${SCENE}] Waiting for LiDAR: $LIDAR_TOPIC"
timeout "$(remaining_time)" rostopic echo -n 1 "$LIDAR_TOPIC" >/dev/null

rostopic pub --once /fc_vision/demo_command std_msgs/UInt8 "data: 1" >/dev/null
wait_for_log "map update" "[FC-Vision][MapUpdate] Finished!"
rostopic pub --once /fc_vision/demo_command std_msgs/UInt8 "data: 2" >/dev/null
wait_for_log "predefined path" "[FC-Vision][UserDefine] Finished!"
rostopic pub --once /fc_vision/demo_command std_msgs/UInt8 "data: 3" >/dev/null
wait_for_log "replanning FSM" "[FC-Vision][Replan] Activated!"
wait_for_node "/px4ctrl"
wait_for_node "/rgb_point_mapping_node"
rostopic pub --once /fc_vision/demo_command std_msgs/UInt8 "data: 4" >/dev/null
wait_for_log_regex "local flight execution" "LOCAL_(PLAN|EXEC)"

LAUNCH_COMPLETE=true
trap - EXIT INT TERM
echo "[${SCENE}] Demo is flying without a joystick."
echo "[${SCENE}] Stop with: ${SCRIPT_DIR}/kill_demo.sh"

if [ "$ATTACH_TMUX" != "false" ]; then
    tmux attach -t "$SESSION"
fi
