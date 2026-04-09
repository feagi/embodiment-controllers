#!/usr/bin/env bash
# Setup MuJoCo controller for feagi-desktop using FEAGI Python SDK only
# Run from FEAGI-2.0 project root. Creates ~/.feagi/controllers/mujoco/2.0.0/

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
FEAGI_ROOT="$(cd "$SCRIPT_DIR/../../../.." && pwd)"
MUJOCO_SRC="$SCRIPT_DIR"
CONTROLLER_ID="mujoco"
VERSION="2.0.0"
CONTROLLER_DIR="$HOME/.feagi/controllers/$CONTROLLER_ID/$VERSION"

echo "[SETUP] MuJoCo controller (FEAGI Python SDK)"
echo "  Source: $MUJOCO_SRC"
echo "  Target: $CONTROLLER_DIR"

# Resolve Python (prefer bundled from feagi-desktop in dev)
case "$(uname -s)-$(uname -m)" in
  Darwin-arm64)  BUNDLED_PY="$FEAGI_ROOT/feagi-desktop/src-tauri/resources/python-macos-arm64/bin/python3" ;;
  Darwin-x86_64)  BUNDLED_PY="$FEAGI_ROOT/feagi-desktop/src-tauri/resources/python-macos-x64/bin/python3" ;;
  Linux-x86_64)  BUNDLED_PY="$FEAGI_ROOT/feagi-desktop/src-tauri/resources/python-linux-x64/bin/python3" ;;
  Linux-aarch64) BUNDLED_PY="$FEAGI_ROOT/feagi-desktop/src-tauri/resources/python-linux-arm64/bin/python3" ;;
  *)             BUNDLED_PY="" ;;
esac
if [[ -f "$BUNDLED_PY" ]]; then
  PYTHON="$BUNDLED_PY"
  echo "  Python: bundled ($PYTHON)"
else
  PYTHON="$(which python3)"
  echo "  Python: system ($PYTHON)"
fi

mkdir -p "$CONTROLLER_DIR"
rm -rf "$CONTROLLER_DIR/venv"
cp "$MUJOCO_SRC/controller.py" "$CONTROLLER_DIR/"
cp "$MUJOCO_SRC/requirements.txt" "$CONTROLLER_DIR/"
cp "$MUJOCO_SRC/manifest.json" "$CONTROLLER_DIR/"

echo "[VENV] Creating virtual environment..."
"$PYTHON" -m venv "$CONTROLLER_DIR/venv"
if [[ -f "$CONTROLLER_DIR/venv/Scripts/python.exe" ]]; then
  VENV_PY="$CONTROLLER_DIR/venv/Scripts/python.exe"
  VENV_PIP="$CONTROLLER_DIR/venv/Scripts/pip.exe"
else
  VENV_PY="$CONTROLLER_DIR/venv/bin/python"
  VENV_PIP="$CONTROLLER_DIR/venv/bin/pip"
fi

echo "[PIP] Installing dependencies..."
"$VENV_PIP" install "feagi-core>=2.1.36" "mujoco==3.6.0" "numpy==1.26.4" -q

echo "[VERIFY] Checking imports..."
"$VENV_PY" -c "import feagi; import mujoco; from feagi.pns import brain_output; print('OK')"

echo "[DONE] Writing .installed and .current..."
echo "2.0.0" > "$HOME/.feagi/controllers/$CONTROLLER_ID/.current"
touch "$CONTROLLER_DIR/.installed"

echo ""
echo "MuJoCo controller ready. Launch from feagi-desktop with a MuJoCo embodiment."
echo "Ensure FEAGI is running and FEAGI_AUTH_TOKEN_B64 is set (or use default)."
