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
for charger_count in 1 2 3; do
  for experiment_mode in baseline proposed; do
    result_dir="$project_dir/results/chargers_$charger_count/$experiment_mode"
    mkdir -p "$result_dir"
    rm -f "$result_dir"/UAV*.jsonl
    printf '%s\n' "$experiment_mode" > "$project_dir/experiment_mode.txt"
    CR_CBBA_MODE="$experiment_mode" CR_CBBA_CHARGERS="$charger_count" \
      "$webots_bin" --batch --mode=fast "$project_dir/worlds/cr-cbba.wbt"
  done
done
printf '%s\n' proposed > "$project_dir/experiment_mode.txt"
mkdir -p "$project_dir/.mplconfig"
if [ -x "$project_dir/.venv/bin/python" ]; then
  python_bin="$project_dir/.venv/bin/python"
else
  python_bin=python3
fi
MPLBACKEND=Agg MPLCONFIGDIR="$project_dir/.mplconfig" "$python_bin" "$project_dir/analysis/analyze_results.py"
