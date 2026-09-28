#!/usr/bin/env bash
# Run MIMOSA against a ROS 2 bag, record the complete run, and create a report.

set -euo pipefail

usage() {
  cat <<'EOF'
Usage:
  run_mimosa_bag.sh [OPTIONS] INPUT_BAG OUTPUT_BAG

Starts MIMOSA, records all topics to OUTPUT_BAG, and plays all topics from
INPUT_BAG. When playback ends, the recording is stopped cleanly and
factor_graph_state_report.py is run on the new bag.

Defaults:
  MIMOSA profile       m113
  RViz visualization  enabled
  simulated time      enabled
  playback clock      enabled

Options:
  --profile NAME          MIMOSA sensor profile (default: m113)
  --viz BOOL              Enable/disable RViz: true or false (default: true)
  --rviz-gpu BOOL         Enable/disable RViz discrete-GPU hints (default: true)
  --config-override FILE  YAML file whose values override the profile config
  --startup-delay SEC     Wait before starting the recorder (default: 2)
  --record-delay SEC      Wait after starting the recorder (default: 1)
  --rate RATE             Bag playback rate (default: 1.0)
  --start-offset SEC      Start this many seconds into the input bag (default: 0)
  --report-output FILE    Report path (default: OUTPUT_BAG/factor_graph_report.html)
  --launch-arg ARG        Extra launch argument, such as imu_topic:=/imu (repeatable)
  --record-arg ARG        Extra ros2 bag record argument (repeatable)
  --play-arg ARG          Extra ros2 bag play argument (repeatable)
  --report-arg ARG        Extra report argument (repeatable)
  -h, --help              Show this help

OUTPUT_BAG must not already exist because ros2 bag record creates it.
Each passthrough value is one argument; repeat the option to pass multiple arguments.
EOF
}

die() {
  echo "Error: $*" >&2
  exit 2
}

is_nonnegative_number() {
  [[ "$1" =~ ^([0-9]+([.][0-9]*)?|[.][0-9]+)$ ]]
}

is_positive_number() {
  awk -v value="$1" 'BEGIN { exit !(value ~ /^([0-9]+([.][0-9]*)?|[.][0-9]+)$/ && value > 0) }'
}

normalize_bool() {
  case "${1,,}" in
    true|1|yes|on) echo true ;;
    false|0|no|off) echo false ;;
    *) return 1 ;;
  esac
}

PROFILE=m113
VIZ=true
RVIZ_GPU=true
CONFIG_OVERRIDE=
STARTUP_DELAY=2
RECORD_DELAY=1
PLAYBACK_RATE=1.0
START_OFFSET=0
REPORT_OUTPUT=
EXTRA_LAUNCH_ARGS=()
EXTRA_RECORD_ARGS=()
EXTRA_PLAY_ARGS=()
EXTRA_REPORT_ARGS=()
POSITIONAL=()

while (($#)); do
  case "$1" in
    --profile|--viz|--rviz-gpu|--config-override|--startup-delay|--record-delay|--rate|--start-offset|--report-output|--launch-arg|--record-arg|--play-arg|--report-arg)
      (($# >= 2)) || die "$1 requires a value"
      option=$1
      value=$2
      shift 2
      case "$option" in
        --profile) PROFILE=$value ;;
        --viz) VIZ=$(normalize_bool "$value") || die "--viz must be true or false" ;;
        --rviz-gpu) RVIZ_GPU=$(normalize_bool "$value") || die "--rviz-gpu must be true or false" ;;
        --config-override) CONFIG_OVERRIDE=$value ;;
        --startup-delay) STARTUP_DELAY=$value ;;
        --record-delay) RECORD_DELAY=$value ;;
        --rate) PLAYBACK_RATE=$value ;;
        --start-offset) START_OFFSET=$value ;;
        --report-output) REPORT_OUTPUT=$value ;;
        --launch-arg) EXTRA_LAUNCH_ARGS+=("$value") ;;
        --record-arg) EXTRA_RECORD_ARGS+=("$value") ;;
        --play-arg) EXTRA_PLAY_ARGS+=("$value") ;;
        --report-arg) EXTRA_REPORT_ARGS+=("$value") ;;
      esac
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    --)
      shift
      POSITIONAL+=("$@")
      break
      ;;
    -*) die "unknown option: $1" ;;
    *) POSITIONAL+=("$1"); shift ;;
  esac
done

((${#POSITIONAL[@]} == 2)) || { usage >&2; exit 2; }

INPUT_BAG=${POSITIONAL[0]}
OUTPUT_BAG=${POSITIONAL[1]}

command -v ros2 >/dev/null 2>&1 || die "ros2 is not available; source ROS 2 and this workspace first"
[[ -e "$INPUT_BAG" ]] || die "input bag does not exist: $INPUT_BAG"
[[ ! -e "$OUTPUT_BAG" ]] || die "output bag already exists: $OUTPUT_BAG"
[[ -z "$CONFIG_OVERRIDE" || -f "$CONFIG_OVERRIDE" ]] || die "config override does not exist: $CONFIG_OVERRIDE"
is_nonnegative_number "$STARTUP_DELAY" || die "--startup-delay must be non-negative"
is_nonnegative_number "$RECORD_DELAY" || die "--record-delay must be non-negative"
is_nonnegative_number "$START_OFFSET" || die "--start-offset must be non-negative"
is_positive_number "$PLAYBACK_RATE" || die "--rate must be greater than zero"

output_parent=$(dirname "$OUTPUT_BAG")
mkdir -p "$output_parent" || die "could not create output parent directory: $output_parent"

LAUNCH_PID=
RECORD_PID=
PLAY_PID=
CLEANING_UP=false

stop_process() {
  local pid=$1
  local label=$2
  local attempts=0

  [[ -n "$pid" ]] || return 0
  kill -0 "$pid" 2>/dev/null || { wait "$pid" 2>/dev/null || true; return 0; }

  echo "Stopping $label..."
  kill -INT "$pid" 2>/dev/null || true
  while kill -0 "$pid" 2>/dev/null && ((attempts < 100)); do
    sleep 0.1
    ((attempts += 1))
  done
  if kill -0 "$pid" 2>/dev/null; then
    echo "$label did not stop after SIGINT; sending SIGTERM." >&2
    kill -TERM "$pid" 2>/dev/null || true
  fi
  wait "$pid" 2>/dev/null || true
}

cleanup() {
  local status=$?
  [[ "$CLEANING_UP" == false ]] || return
  CLEANING_UP=true
  trap - EXIT INT TERM
  stop_process "$PLAY_PID" "bag playback"
  stop_process "$RECORD_PID" "bag recorder"
  stop_process "$LAUNCH_PID" "MIMOSA launch"
  exit "$status"
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

launch_cmd=(
  ros2 launch mimosa mimosa.launch.py
  "profile:=$PROFILE"
  "viz:=$VIZ"
  "rviz_gpu:=$RVIZ_GPU"
  use_sim_time:=true
)
if [[ -n "$CONFIG_OVERRIDE" ]]; then
  launch_cmd+=("config_override:=$CONFIG_OVERRIDE")
fi
launch_cmd+=("${EXTRA_LAUNCH_ARGS[@]}")

echo "Starting MIMOSA (profile=$PROFILE, viz=$VIZ)..."
"${launch_cmd[@]}" &
LAUNCH_PID=$!
sleep "$STARTUP_DELAY"
kill -0 "$LAUNCH_PID" 2>/dev/null || { wait "$LAUNCH_PID" || true; die "MIMOSA launch exited during startup"; }

record_cmd=(ros2 bag record --all-topics --output "$OUTPUT_BAG" --disable-keyboard-controls)
record_cmd+=("${EXTRA_RECORD_ARGS[@]}")
echo "Recording all topics to: $OUTPUT_BAG"
"${record_cmd[@]}" &
RECORD_PID=$!
sleep "$RECORD_DELAY"
kill -0 "$RECORD_PID" 2>/dev/null || { wait "$RECORD_PID" || true; die "bag recorder exited during startup"; }

play_cmd=(
  ros2 bag play "$INPUT_BAG"
  --clock
  --rate "$PLAYBACK_RATE"
  --start-offset "$START_OFFSET"
  --disable-keyboard-controls
)
play_cmd+=("${EXTRA_PLAY_ARGS[@]}")
echo "Playing all topics from: $INPUT_BAG"
"${play_cmd[@]}" &
PLAY_PID=$!
set +e
wait "$PLAY_PID"
play_status=$?
set -e
PLAY_PID=

stop_process "$RECORD_PID" "bag recorder"
RECORD_PID=
stop_process "$LAUNCH_PID" "MIMOSA launch"
LAUNCH_PID=

((play_status == 0)) || die "bag playback failed with exit status $play_status"
[[ -e "$OUTPUT_BAG/metadata.yaml" ]] || die "recording did not produce $OUTPUT_BAG/metadata.yaml"

report_cmd=(ros2 run mimosa factor_graph_state_report.py "$OUTPUT_BAG")
if [[ -n "$REPORT_OUTPUT" ]]; then
  report_cmd+=(--output "$REPORT_OUTPUT")
fi
report_cmd+=("${EXTRA_REPORT_ARGS[@]}")

echo "Generating factor-graph state report..."
"${report_cmd[@]}"
echo "Completed recording and report for: $OUTPUT_BAG"

trap - EXIT INT TERM
