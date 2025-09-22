#!/usr/bin/env bash
# ros2_play_merge.sh
#
# Usage examples:
#   # Play multiple bags aligned on one /clock (no merge written)
#   ./ros2_play_merge.sh runA/imu runA/cams runA/rtk -- --rate 1.0
#
#   # Create a merged bag (single timeline) and then stop; you can play it later with one process
#   ./ros2_play_merge.sh --merge-out merged_run runA/imu runA/cams runA/rtk -- --rate 1.0
#
# Notes:
# - FIRST bag is the /clock source. Other players follow that simulated time.
# - If --merge-out is given, we start `ros2 bag record -a` to write a single merged bag directory.
# - Any args after `--` are forwarded to each `ros2 bag play` (e.g., --rate, --loop, --start-offset N).
# - Consumers must have use_sim_time:=true (set in their launch or params).

set -euo pipefail

merge_out=""
play_args=()
bags=()

# --- parse CLI ---
seen_ddash=0
while [[ $# -gt 0 ]]; do
  if [[ $seen_ddash -eq 1 ]]; then
    play_args+=("$1"); shift; continue
  fi
  case "$1" in
    -h|--help)
      cat <<EOF
Usage:
  $0 [--merge-out DIR] BAG_DIR1 [BAG_DIR2 ...] [-- <ros2 bag play args>]

Examples:
  $0 runA/imu runA/cams runA/rtk -- --rate 0.5 --loop
  $0 --merge-out merged_run runA/imu runA/cams -- --start-offset 0.2

Details:
- FIRST bag publishes /clock. Others follow that single simulated timeline.
- With --merge-out DIR: start a recorder (ros2 bag record -a -o DIR) while playing all bags.
  When you Ctrl-C, the recorder stops and you'll have a single merged bag in DIR/.
EOF
      exit 0
      ;;
    --merge-out)
      [[ $# -ge 2 ]] || { echo "[ERR] --merge-out needs a directory name"; exit 2; }
      merge_out="$2"; shift 2;;
    --)
      seen_ddash=1; shift;;
    -*)
      echo "[WARN] Unknown option $1 before '--' (will be ignored; pass ros2 play args after '--')"
      shift;;
    *)
      bags+=("$1"); shift;;
  esac
done

if [[ ${#bags[@]} -lt 1 ]]; then
  echo "[ERR] Provide at least one BAG_DIR"; exit 2
fi

# --- sanity: each bag must be a ros2 bag directory (metadata.yaml present) ---
for b in "${bags[@]}"; do
  if [[ ! -f "$b/metadata.yaml" ]]; then
    echo "[ERR] Not a ros2 bag directory (missing metadata.yaml): $b"
    exit 3
  fi
done

echo "[INFO] Bags:"
for i in "${!bags[@]}"; do echo "  [$i] ${bags[$i]}"; done
echo "[INFO] First bag is /clock source."

# --- cleanup trap ---
pids=()
cleanup() {
  echo -e "\n[INFO] Stopping players/recorder..."
  for pid in "${pids[@]:-}"; do
    kill "$pid" 2>/dev/null || true
  done
  wait || true
}
trap cleanup INT TERM

# --- recorder (optional merge) ---
if [[ -n "$merge_out" ]]; then
  if [[ -e "$merge_out" ]]; then
    echo "[ERR] Merge output path already exists: $merge_out"
    exit 4
  fi
  echo "[INFO] Starting recorder (merged output): $merge_out"
  # Record all topics (includes /clock, which is fine). If you want to exclude /clock:
  # ros2 bag record -a -o "$merge_out" --exclude "/clock"
  ros2 bag record -a -o "$merge_out" &
  pids+=("$!")
  # Give recorder a moment to subscribe before players start
  sleep 0.5
fi

# --- players ---
# First bag publishes /clock (single simulated timeline)
first="${bags[0]}"
echo "[INFO] Start clock source: $first"
ros2 bag play "$first" --clock "${play_args[@]}" &
pids+=("$!")

# Other bags: no --clock, but they *follow* the clock because rosbag2 uses ROS time to schedule
if [[ ${#bags[@]} -gt 1 ]]; then
  for b in "${bags[@]:1}"; do
    echo "[INFO] Start: $b"
    ros2 bag play "$b" "${play_args[@]}" &
    pids+=("$!")
  done
fi

# --- wait for everything ---
wait "${pids[@]}"

# If we reached here cleanly and a merge was requested, print what to do next
if [[ -n "$merge_out" ]]; then
  echo "[OK] Merged bag written to: $merge_out"
  echo "Play it later with a single process:"
  echo "  ros2 bag play \"$merge_out\" --clock"
fi
