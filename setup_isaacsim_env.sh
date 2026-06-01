#!/usr/bin/env bash
set -euo pipefail

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ENV_PATH="${ENV_PATH:-$PROJECT_ROOT/.conda/env_isaacsim_5_1_0}"
PYTHON_EXE="$ENV_PATH/bin/python"
REQUIREMENTS_FILE="$PROJECT_ROOT/requirements-isaacsim.txt"
FORCE=0

usage() {
  cat <<'EOF'
Usage: ./setup_isaacsim_env.sh [--env-path PATH] [--force]

Creates a local Python 3.11 conda environment and installs Isaac Sim 5.1 pip packages.
The default environment path is ./.conda/env_isaacsim_5_1_0 under the repository root.
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --env-path)
      ENV_PATH="$2"
      PYTHON_EXE="$ENV_PATH/bin/python"
      shift 2
      ;;
    --force)
      FORCE=1
      shift
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

if [[ ! -f "$REQUIREMENTS_FILE" ]]; then
  echo "Isaac Sim requirements file was not found: $REQUIREMENTS_FILE" >&2
  exit 1
fi

if [[ -d "$ENV_PATH" && "$FORCE" -ne 1 ]]; then
  echo "Environment already exists: $ENV_PATH"
  echo "Move or delete it manually if you need to recreate it."
  exit 0
fi

if [[ -d "$ENV_PATH" && "$FORCE" -eq 1 ]]; then
  echo "Refusing to delete an existing environment automatically: $ENV_PATH" >&2
  echo "Move or delete it manually, then rerun." >&2
  exit 1
fi

if ! command -v conda >/dev/null 2>&1; then
  echo "conda was not found on PATH. Install Miniconda/Anaconda or open a conda-enabled shell, then rerun." >&2
  exit 1
fi

echo "Creating conda environment: $ENV_PATH"
conda create -p "$ENV_PATH" python=3.11 -y

if [[ ! -x "$PYTHON_EXE" ]]; then
  echo "Python executable was not created: $PYTHON_EXE" >&2
  exit 1
fi

echo "Installing Isaac Sim 5.1 pip packages. This can take a long time."
"$PYTHON_EXE" -m pip install --upgrade pip
"$PYTHON_EXE" -m pip install -r "$REQUIREMENTS_FILE"

echo "Environment ready: $PYTHON_EXE"
echo "Run the GUI demo with:"
echo "./run_harim_demo.sh --accept-eula --cycles 1"
