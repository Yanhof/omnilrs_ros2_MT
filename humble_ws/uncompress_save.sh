#!/usr/bin/env bash
set -euo pipefail

# Usage:
#   ./uncompress_save.sh [--mode base|suffix] [--suffix NAME] [--keep-compressed] [--force] INPUT_BAG_DIR [OUTPUT_BAG_DIR]
#
# Defaults:
#   --mode suffix           -> publish to "<base>/uncompressed"
#   --suffix uncompressed   -> the suffix appended when --mode suffix
#   --keep-compressed       -> also record the original /compressed topics
#   --force                 -> skip preflight "ghost node" checks
#
# Requirements:
#   - ROS 2 Humble sourced
#   - image_transport + compressed_image_transport installed
#   - python3 + PyYAML (python3-yaml)

MODE="suffix"            # "suffix" or "base"
SUFFIX="uncompressed"
KEEP_COMPRESSED=0
FORCE=0

# ---- parse options ----
ARGS=()
while [[ $# -gt 0 ]]; do
  case "$1" in
    --mode) MODE="${2:-suffix}"; shift 2;;
    --suffix) SUFFIX="${2:-uncompressed}"; shift 2;;
    --keep-compressed) KEEP_COMPRESSED=1; shift;;
    --force) FORCE=1; shift;;
    -h|--help)
      echo "Usage: $0 [--mode base|suffix] [--suffix NAME] [--keep-compressed] [--force] INPUT_BAG_DIR [OUTPUT_BAG_DIR]"
      exit 0;;
    --) shift; break;;
    -*)
      echo "[!] Unknown option: $1" >&2; exit 2;;
    *)
      ARGS+=("$1"); shift;;
  esac
done
set -- "${ARGS[@]}" "$@"

if [[ $# -lt 1 || $# -gt 2 ]]; then
  echo "Usage: $0 [--mode base|suffix] [--suffix NAME] [--keep-compressed] [--force] INPUT_BAG_DIR [OUTPUT_BAG_DIR]" >&2
  exit 2
fi

INPUT_BAG="$(readlink -f "$1")"
if [[ ! -d "$INPUT_BAG" ]]; then
  echo "[!] Input bag directory not found: $INPUT_BAG" >&2
  exit 1
fi

OUTPUT_BAG="${2:-${INPUT_BAG}_uncompressed}"
OUTPUT_BAG="$(readlink -f "$OUTPUT_BAG")"
if [[ -e "$OUTPUT_BAG" ]]; then
  echo "[!] Output path already exists: $OUTPUT_BAG" >&2
  exit 1
fi

command -v ros2 >/dev/null || { echo "[!] ros2 not found in PATH. Source your ROS 2 setup.bash"; exit 1; }
command -v python3 >/dev/null || { echo "[!] python3 not found"; exit 1; }

METADATA="$INPUT_BAG/metadata.yaml"
if [[ ! -f "$METADATA" ]]; then
  echo "[!] metadata.yaml not found in: $INPUT_BAG" >&2
  exit 1
fi

# Check the compressed transport plugin
if ! ros2 run image_transport list_transports 2>/dev/null | grep -q "image_transport/compressed"; then
  echo "[!] 'compressed_image_transport' missing. Install: sudo apt-get install -y ros-humble-compressed-image-transport" >&2
  exit 1
fi

# Preflight: refuse to run if ghosts are visible in *this* domain (unless --force)
if [[ $FORCE -eq 0 ]]; then
  if ros2 topic list | grep -qx "/in/compressed"; then
    echo "[!] Found stray /in/compressed subscriber (old republisher). Kill it or re-run with --force (or use a private ROS_DOMAIN_ID)." >&2
    exit 1
  fi
  if ros2 node list | awk '{print $1}' | grep -qx "/image_republisher"; then
    echo "[!] Found generic /image_republisher nodes from a previous run. Kill them or re-run with --force (or isolate domain)." >&2
    exit 1
  fi
fi

TMPDIR="$(mktemp -d)"
COMPRESSED_TXT="$TMPDIR/compressed.txt"
BASES_TXT="$TMPDIR/bases.txt"
UNCHANGED_TXT="$TMPDIR/unchanged.txt"

# --- Discover topics from metadata.yaml (no depth handling) ---
python3 - "$INPUT_BAG" "$COMPRESSED_TXT" "$BASES_TXT" "$UNCHANGED_TXT" << 'PY'
import sys, os, yaml

bag_dir, out_comp, out_bases, out_unch = sys.argv[1:5]
meta = os.path.join(bag_dir, "metadata.yaml")
with open(meta, "r") as f:
    data = yaml.safe_load(f)

info = data.get("rosbag2_bagfile_information", data)
twmc = info.get("topics_with_message_count", [])

topics = []
for t in twmc:
    md = t.get("topic_metadata", {})
    name = md.get("name", "")
    typ  = md.get("type", "")
    topics.append((name, typ))

compressed = [name for (name, _) in topics if name.endswith("/compressed")]

def base_from(n):
    return n[:-len("/compressed")] if n.endswith("/compressed") else n

bases = [base_from(n) for n in compressed]
unchanged = [name for (name, _) in topics if name not in set(compressed)]

with open(out_comp, "w") as f:
    for n in compressed: f.write(n + "\n")
with open(out_bases, "w") as f:
    for n in bases: f.write(n + "\n")
with open(out_unch, "w") as f:
    for n in unchanged: f.write(n + "\n")
PY

# Read lists (safe if empty)
mapfile -t COMPRESSED_TOPICS < "$COMPRESSED_TXT" || COMPRESSED_TOPICS=()
mapfile -t BASE_TOPICS       < "$BASES_TXT"      || BASE_TOPICS=()
mapfile -t UNCHANGED_TOPICS  < "$UNCHANGED_TXT"  || UNCHANGED_TOPICS=()

echo "[i] Input bag:  $INPUT_BAG"
echo "[i] Output bag: $OUTPUT_BAG"

if [[ ${#COMPRESSED_TOPICS[@]} -eq 0 ]]; then
  echo "[!] No /compressed topics found. Nothing to do."
  rm -rf "$TMPDIR"; exit 1
fi

echo "[i] Found compressed topics:"
for t in "${COMPRESSED_TOPICS[@]}"; do echo "    $t"; done

echo "[i] Output mode: $MODE"
if [[ "$MODE" == "suffix" ]]; then
  echo "[i] Output suffix: /$SUFFIX (e.g., <base>/$SUFFIX)"
fi
[[ $KEEP_COMPRESSED -eq 1 ]] && echo "[i] Will also keep /compressed topics in the new bag."

# Decide output topics for each base
declare -a OUT_TOPICS=()
if [[ "$MODE" == "base" ]]; then
  OUT_TOPICS=("${BASE_TOPICS[@]}")
else
  for b in "${BASE_TOPICS[@]}"; do OUT_TOPICS+=("${b}/${SUFFIX}"); done
fi

echo "[i] Will republish → raw on topics:"
for t in "${OUT_TOPICS[@]}"; do echo "    $t"; done

# --- helper: make a safe node name ---
safe_name () {
  echo "$1" | sed 's#[^a-zA-Z0-9_]#_#g' | tr -s '_' | sed 's#^_*##; s#_*$##'
}

# --- Start republishers (one per input stream) ---
PIDS=()
for idx in "${!BASE_TOPICS[@]}"; do
  b="${BASE_TOPICS[$idx]}"
  out="${OUT_TOPICS[$idx]}"
  SAFE="$(safe_name "$out")"
  NODE="image_republisher_${SAFE}"
  (
    ros2 run image_transport republish compressed raw \
      --ros-args -r __node:="$NODE" \
                 -r in/compressed:="${b}/compressed" \
                 -r out:="$out"
  ) & PIDS+=($!)
done

# --- Build record topic list: all unchanged + new uncompressed topics (+ optional keep compressed) ---
REC_TOPICS=()
for t in "${UNCHANGED_TOPICS[@]}"; do REC_TOPICS+=("$t"); done
for t in "${OUT_TOPICS[@]}";     do REC_TOPICS+=("$t"); done
if [[ $KEEP_COMPRESSED -eq 1 ]]; then
  for t in "${COMPRESSED_TOPICS[@]}"; do REC_TOPICS+=("$t"); done
fi

echo "[i] Recording ${#REC_TOPICS[@]} topics to: $OUTPUT_BAG"
ros2 bag record -o "$OUTPUT_BAG" "${REC_TOPICS[@]}" &
REC_PID=$!
echo "[i] Recorder PID: $REC_PID"

# ---- Robust finalization helpers (ensure metadata.yaml) ----
__CLEANED=0
finish() {
  [[ $__CLEANED -eq 1 ]] && return
  __CLEANED=1

  # 1) Stop recorder first (so it writes metadata.yaml)
  if kill -0 "$REC_PID" 2>/dev/null; then
    echo "[i] Stopping recorder..."
    kill -INT "$REC_PID" 2>/dev/null || true

    # wait up to 10s for graceful shutdown
    for i in {1..100}; do
      kill -0 "$REC_PID" 2>/dev/null || break
      sleep 0.1
    done

    if kill -0 "$REC_PID" 2>/dev/null; then
      echo "[!] Recorder still running after SIGINT; sending SIGTERM..."
      kill -TERM "$REC_PID" 2>/dev/null || true
      wait "$REC_PID" 2>/dev/null || true
    fi
  fi

  # 2) Stop republishers
  echo "[i] Stopping republishers..."
  for pid in "${PIDS[@]}"; do kill "$pid" 2>/dev/null || true; done
  wait || true

  # 3) Make sure metadata.yaml exists (reindex if needed)
  if [[ ! -f "$OUTPUT_BAG/metadata.yaml" ]]; then
    echo "[i] metadata.yaml missing; reindexing output bag..."
    ros2 bag reindex "$OUTPUT_BAG" || true
  fi

  # 4) Cleanup temp dir
  rm -rf "$TMPDIR"
}

# Ensure we finalize on Ctrl-C or errors
trap finish EXIT INT TERM

echo "[i] Giving nodes 2 seconds to initialize..."
sleep 2

# --- Play once (no loop) ---
echo "[i] Replaying input bag..."
set +e
ros2 bag play "$INPUT_BAG"
PLAY_RC=$?
set -e

echo "[i] Playback finished (rc=$PLAY_RC). Finalizing..."
finish
trap - EXIT INT TERM

echo "[✓] Done. New bag at: $OUTPUT_BAG"
echo "[i] Verify with: ros2 bag info \"$OUTPUT_BAG\""

