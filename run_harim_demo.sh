#!/usr/bin/env bash
set -euo pipefail

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PYTHON_EXE="${PYTHON_EXE:-$PROJECT_ROOT/.conda/env_isaacsim_5_1_0/bin/python}"
DEMO_SCRIPT="$PROJECT_ROOT/isaac_sim/scripts/run_harim_pallet_demo.py"

HEADLESS=0
ACCEPT_EULA=0
CYCLES=0
STACK_COLS=2
STACK_ROWS=2
STACK_LAYERS=2
SELF_TEST_FRAMES=0
SELF_TEST_FORCE_STACK_COMPLETE=0
MOVE_SPEED=0.65
PICKUP_X=1.25
PICKUP_Y=-0.31
DROP_X=11.85
DROP_Y=-0.31
CAPTURE_PATH=""

usage() {
  cat <<'EOF'
Usage: ./run_harim_demo.sh [options]

Options:
  --headless
  --accept-eula
  --cycles N
  --stack-cols N
  --stack-rows N
  --stack-layers N
  --self-test-frames N
  --self-test-force-stack-complete
  --move-speed FLOAT
  --pickup-x FLOAT
  --pickup-y FLOAT
  --drop-x FLOAT
  --drop-y FLOAT
  --capture-path PATH
  --python-exe PATH
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --headless)
      HEADLESS=1
      shift
      ;;
    --accept-eula)
      ACCEPT_EULA=1
      shift
      ;;
    --cycles)
      CYCLES="$2"
      shift 2
      ;;
    --stack-cols)
      STACK_COLS="$2"
      shift 2
      ;;
    --stack-rows)
      STACK_ROWS="$2"
      shift 2
      ;;
    --stack-layers)
      STACK_LAYERS="$2"
      shift 2
      ;;
    --self-test-frames)
      SELF_TEST_FRAMES="$2"
      shift 2
      ;;
    --self-test-force-stack-complete)
      SELF_TEST_FORCE_STACK_COMPLETE=1
      shift
      ;;
    --move-speed)
      MOVE_SPEED="$2"
      shift 2
      ;;
    --pickup-x)
      PICKUP_X="$2"
      shift 2
      ;;
    --pickup-y)
      PICKUP_Y="$2"
      shift 2
      ;;
    --drop-x)
      DROP_X="$2"
      shift 2
      ;;
    --drop-y)
      DROP_Y="$2"
      shift 2
      ;;
    --capture-path)
      CAPTURE_PATH="$2"
      shift 2
      ;;
    --python-exe)
      PYTHON_EXE="$2"
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "Unknown argument: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

if [[ ! -x "$PYTHON_EXE" ]]; then
  echo "Isaac Sim Python environment was not found: $PYTHON_EXE" >&2
  echo "Run setup first: ./setup_isaacsim_env.sh" >&2
  exit 1
fi

if [[ ! -f "$DEMO_SCRIPT" ]]; then
  echo "Demo script was not found: $DEMO_SCRIPT" >&2
  exit 1
fi

if [[ "$ACCEPT_EULA" -eq 1 ]]; then
  export OMNI_KIT_ACCEPT_EULA=YES
fi

ARGS=(
  "$DEMO_SCRIPT"
  "--cycles" "$CYCLES"
  "--stack-cols" "$STACK_COLS"
  "--stack-rows" "$STACK_ROWS"
  "--stack-layers" "$STACK_LAYERS"
  "--self-test-frames" "$SELF_TEST_FRAMES"
  "--move-speed" "$MOVE_SPEED"
  "--pickup-x" "$PICKUP_X"
  "--pickup-y" "$PICKUP_Y"
  "--drop-x" "$DROP_X"
  "--drop-y" "$DROP_Y"
)

if [[ "$HEADLESS" -eq 1 ]]; then
  ARGS+=("--headless")
fi

if [[ "$SELF_TEST_FORCE_STACK_COMPLETE" -eq 1 ]]; then
  ARGS+=("--self-test-force-stack-complete")
fi

if [[ -n "$CAPTURE_PATH" ]]; then
  ARGS+=("--capture-path" "$CAPTURE_PATH")
fi

exec "$PYTHON_EXE" "${ARGS[@]}"
