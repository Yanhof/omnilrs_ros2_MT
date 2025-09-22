#!/usr/bin/env bash
set -euo pipefail

# Usage:
#   ./uncompress_bag.sh INPUT_BAG_DIR [OUTPUT_BAG_DIR]
#
# Requirements:
#   - ROS 2 Humble sourced
#   - image_transport available (for republish)
#   - python3 + PyYAML (python3-yaml)

if [[ $# -lt 1 || $# -gt 2 ]]; then
  echo "Usage: $0 INPUT_BAG_DIR [OUTPUT_BAG_DIR]" >&2
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

TMPDIR="$(mktemp -d)"
COMPRESSED_TXT="$TMPDIR/compressed.txt"
BASES_TXT="$TMPDIR/bases.txt"
UNCHANGED_TXT="$TMPDIR/unchanged.txt"

# --- Discover topics from metadata.yaml (no jq needed) ---
python3 - "$INPUT_BAG" "$COMPRESSED_TXT" "$BASES_TXT" "$UNCHANGED_TXT" << 'PY'
import sys, os, yaml

bag_dir, out_comp, out_bases, out_unch = sys.argv[1:5]
meta = os.path.join(bag_dir, "metadata.yaml")
with open(meta, "r") as f:
    data = yaml.safe_load(f)

# Support both schemas:
# - Older: topics_with_message_count at root
# - Newer (Humble+): under rosbag2_bagfile_information
twmc = data.get("topics_with_message_count")
if twmc is None:
    info = data.get("rosbag2_bagfile_information", {})
    twmc = info.get("topics_with_message_count", [])

topics = []
for t in twmc:
    md = t.get("topic_metadata", {})
    name = md.get("name", "")
    typ  = md.get("type", "")
    topics.append((name, typ))

# Identify compressed image topics (suffix match is robust)
compressed = [name for (name, typ) in topics if name.endswith("/compressed")]
bases = [name[:-len("/compressed")] for name in compressed]
unchanged = [name for (name, _) in topics if name not in compressed]

with open(out_comp, "w") as f:
    for n in compressed:
        f.write(n + "\n")
with open(out_bases, "w") as f:
    for n in bases:
        f.write(n + "\n")
with open(out_unch, "w") as f:
    for n in unchanged:
        f.write(n + "\n")
PY


# Read lists (safe if empty)
mapfile -t COMPRESSED_TOPICS < "$COMPRESSED_TXT" || COMPRESSED_TOPICS=()
mapfile -t BASE_TOPICS       < "$BASES_TXT"      || BASE_TOPICS=()
mapfile -t UNCHANGED_TOPICS  < "$UNCHANGED_TXT"  || UNCHANGED_TOPICS=()

echo "[i] Input bag:  $INPUT_BAG"
echo "[i] Output bag: $OUTPUT_BAG"
echo "[i] Found compressed topics:"
if [[ ${#COMPRESSED_TOPICS[@]} -eq 0 ]]; then
  echo "    (none)"
  echo "[!] No /compressed image topics found. Nothing to do."
  rm -rf "$TMPDIR"
  exit 1
else
  printf '    %s\n' "${COMPRESSED_TOPICS[@]}"
fi

echo "[i] Will republish → raw on base topics:"
printf '    %s\n' "${BASE_TOPICS[@]}"

# --- Start republishers (one per base) ---
PIDS=()
for b in "${BASE_TOPICS[@]}"; do
  # Make a safe node name from the topic: letters/digits/_ only, no duplicate underscores
  SAFE=$(echo "$b" | sed 's#[^a-zA-Z0-9_]#_#g' | tr -s '_' | sed 's#^_*##; s#_*$##')
  NODE="image_republisher_${SAFE}"

  # The key change is from '-r in:="$b"' to '-r in/compressed:="$b/compressed"'
  ( ros2 run image_transport republish compressed raw \
  --ros-args -r __node:="$NODE" -r in/compressed:="$b/compressed" -r out:="$b" \
  -p reliability:=reliable -p history:=keep_last -p depth:=10 ) &
  PIDS+=($!)
done

# --- Build record topic list: all unchanged + new raw bases ---
REC_TOPICS=()
for t in "${UNCHANGED_TOPICS[@]}"; do REC_TOPICS+=("$t"); done
for t in "${BASE_TOPICS[@]}";     do REC_TOPICS+=("$t"); done

echo "[i] Recording ${#REC_TOPICS[@]} topics to: $OUTPUT_BAG"
ros2 bag record -o "$OUTPUT_BAG" "${REC_TOPICS[@]}" &
REC_PID=$!

echo "[i] Giving nodes 3 seconds to initialize..."
sleep 3 #

# --- Play once (no loop) ---
echo "[i] Replaying input bag..."
set +e
ros2 bag play "$INPUT_BAG"
PLAY_RC=$?
set -e

echo "[i] Playback finished (rc=$PLAY_RC). Stopping recorder and republishers..."
# Cleanly stop recorder to finalize indices
kill -INT "$REC_PID" 2>/dev/null || true
wait "$REC_PID" 2>/dev/null || true

# Then kill ONLY the republishers
for pid in "${PIDS[@]}"; do kill "$pid" 2>/dev/null || true; done
wait || true

echo "[✓] Done. New bag at: $OUTPUT_BAG"
echo "[i] Verify with: ros2 bag info \"$OUTPUT_BAG\""
rm -rf "$TMPDIR"
