#!/usr/bin/env bash
source /opt/ros/lyrical/setup.bash
set -euo pipefail

ROOT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)
EXAMPLE_DIR="$ROOT_DIR/examples/hospital_delivery"
LIFECYCLE="$EXAMPLE_DIR/scripts/codex_project.py"
VERIFY_SCRIPT="$EXAMPLE_DIR/scripts/verify_acceptance.py"

headless=false
verify=false
started=false

usage() {
  printf '%s\n' \
    'Usage: scripts/demo_hospital.sh [--headless] [--verify]' \
    '' \
    '  --headless  Run Gazebo without the GUI.' \
    '  --verify    Run the independent schema-2 acceptance monitor.'
}

cleanup() {
  if [[ "$started" == true ]]; then
    python3 "$LIFECYCLE" stop >/dev/null || {
      printf '%s\n' 'Hospital runtime cleanup did not complete.' >&2
      return 1
    }
    started=false
  fi
}

on_signal() {
  exit 130
}

trap cleanup EXIT
trap on_signal HUP INT TERM

while (($#)); do
  case "$1" in
    --headless)
      headless=true
      ;;
    --verify)
      verify=true
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      printf 'Unknown option: %s\n' "$1" >&2
      usage >&2
      exit 2
      ;;
  esac
  shift
done

start_args=(start --timeout 60)
if [[ "$headless" == false ]]; then
  start_args+=(--gui)
fi

python3 "$LIFECYCLE" "${start_args[@]}"
started=true

if [[ "$verify" == true ]]; then
  python3 "$VERIFY_SCRIPT" \
    --timeout 180 \
    --output "$EXAMPLE_DIR/logs/acceptance_report.json"
  exit 0
fi

python3 "$LIFECYCLE" mission-start
printf '%s\n' 'Hospital delivery is running; press Ctrl-C to stop safely.'
while true; do
  status=$(python3 "$LIFECYCLE" mission-status)
  printf '%s\n' "$status"
  case "$status" in
    *'"state": "SUCCEEDED"'*|*'"state":"SUCCEEDED"'*) exit 0 ;;
    *'"state": "FAILED"'*|*'"state":"FAILED"'*) exit 1 ;;
    *'"state": "CANCELLED"'*|*'"state":"CANCELLED"'*) exit 1 ;;
    *'"state": "ESTOPPED"'*|*'"state":"ESTOPPED"'*) exit 1 ;;
  esac
  sleep 1
done
