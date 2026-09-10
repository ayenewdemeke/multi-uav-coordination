#!/bin/sh
set -eu
project_dir=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
if [ -n "${WEBOTS_BIN:-}" ]; then
  webots_bin=$WEBOTS_BIN
elif command -v webots >/dev/null 2>&1; then
  webots_bin=$(command -v webots)
elif [ -x /Applications/Webots.app/Contents/MacOS/webots ]; then
  webots_bin=/Applications/Webots.app/Contents/MacOS/webots
else
  echo "Webots was not found. Set WEBOTS_BIN to its executable." >&2; exit 127
fi
for method in proposed battery_only; do
  for uav_count in 3 4 5 6; do
    for charger_count in 1 2 3 4; do
      result_dir="$project_dir/results/${method}/uav${uav_count}_chargers_${charger_count}"
      mkdir -p "$result_dir"
      rm -f "$result_dir"/UAV*.jsonl
      CR_CBBA_METHOD="$method" \
        CR_CBBA_UAV_COUNT="$uav_count" \
        CR_CBBA_CHARGER_COUNT="$charger_count" \
        "$webots_bin" --batch --mode=fast "$project_dir/worlds/cr-cbba.wbt"
    done
  done
done
mkdir -p "$project_dir/.mplconfig"
if [ -x "$project_dir/.venv/bin/python" ]; then
  python_bin="$project_dir/.venv/bin/python"
else
  python_bin=python3
fi
MPLBACKEND=Agg MPLCONFIGDIR="$project_dir/.mplconfig" "$python_bin" "$project_dir/analysis/analyze_results.py"
