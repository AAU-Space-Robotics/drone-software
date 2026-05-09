#!/bin/bash

SCRIPT_DIR=$(dirname "$(realpath "$0")")
WORKSPACE_DIR=$(dirname "$SCRIPT_DIR")
VENV_DIR="$WORKSPACE_DIR/.venv"
REQUIREMENTS="$SCRIPT_DIR/requirements.txt"

echo "Setting up Python virtual environment at $VENV_DIR ..."

if [ ! -f "$REQUIREMENTS" ]; then
    echo "Error: requirements.txt not found at $REQUIREMENTS"
    exit 1
fi

PYTHON_VERSION=$(python3 -c 'import sys; print(f"{sys.version_info.major}.{sys.version_info.minor}")')
VENV_PKG="python${PYTHON_VERSION}-venv"

if ! dpkg -l | grep -q "^ii  $VENV_PKG "; then
    echo "Installing $VENV_PKG ..."
    sudo apt-get install -y "$VENV_PKG"
fi

if [ ! -d "$VENV_DIR" ]; then
    python3 -m venv "$VENV_DIR"
    if [ ! -f "$VENV_DIR/bin/activate" ]; then
        echo "Error: virtual environment creation failed."
        exit 1
    fi
    echo "Created virtual environment."
else
    echo "Virtual environment already exists, skipping creation."
fi

source "$VENV_DIR/bin/activate"

pip install --upgrade pip --quiet
pip install -r "$REQUIREMENTS"

echo ""
echo "Done. Activate with:"
echo "  source $VENV_DIR/bin/activate"
