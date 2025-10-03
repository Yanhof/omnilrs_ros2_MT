#!/usr/bin/env bash
set -euo pipefail

# --- adjust these two paths ---
IN_ROOT="/home/yhofmann/git/grand_tour_dataset/examples_ros1/ros2_bags_converted_sqlite"
OUT_ROOT="/home/yhofmann/git/grand_tour_dataset/examples_ros1/ros2_bags_converted_mcap"

# ----- safe ROS 2 sourcing with 'set -u' -----
set +u
export AMENT_TRACE_SETUP_FILES=0
export AMENT_PYTHON_EXECUTABLE="$(command -v python3 || echo /usr/bin/python3)"
source /opt/ros/humble/setup.bash
set -u
# ---------------------------------------------

# Require the MCAP writer plugin
if ! ros2 bag info -s mcap --help >/dev/null 2>&1; then
  echo "[ERR] mcap storage not available. Install: sudo apt install ros-humble-rosbag2-storage-mcap"
  exit 1
fi

mkdir -p "$OUT_ROOT"

# MCAP writer options (compress with Zstd)
WRITER_OPTS="/tmp/mcap_writer_options.yaml"
cat > "$WRITER_OPTS" <<'EOF'
compression: Zstd
EOF

echo "[INFO] Scanning subfolders under: $IN_ROOT"

# Iterate all first-level subfolders; handle spaces safely
find "$IN_ROOT" -mindepth 1 -maxdepth 1 -type d -print0 |
while IFS= read -r -d '' IN_DIR; do
  # must contain metadata.yaml to be a rosbag2 folder
  if [[ ! -f "$IN_DIR/metadata.yaml" ]]; then
    echo "[SKIP] No metadata.yaml in: $IN_DIR"
    continue
  fi

  base="$(basename "$IN_DIR")"
  OUT_DIR="${OUT_ROOT}/${base}_mcap"

  # if any output dir/file exists already, skip (or rm -rf to force)
  if [[ -e "$OUT_DIR" ]]; then
    echo "[SKIP] Output exists: $OUT_DIR"
    continue
  fi



  # Per-bag output spec (needs variable expansion → no single quotes)
  TO_MCAP="$(mktemp /tmp/to_mcap.XXXXXX.yaml)"
  cat > "$TO_MCAP" <<EOF
output_bags:
  - uri: $OUT_DIR
    storage_id: mcap
    storage_config_uri: $WRITER_OPTS
    all: true
EOF

  echo "[INFO] sqlite -> MCAP:"
  echo "       IN : $IN_DIR"
  echo "       OUT: $OUT_DIR"
  ros2 bag convert -i "$IN_DIR" -o "$TO_MCAP"

  rm -f "$TO_MCAP"
done

echo "[INFO] Done. MCAP outputs found:"
find "$OUT_ROOT" -type f -name '*.mcap' -print | sort
