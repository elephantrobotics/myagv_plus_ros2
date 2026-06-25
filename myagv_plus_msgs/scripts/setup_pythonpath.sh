#!/usr/bin/env bash
# Add this folder (ros_client.py) to PYTHONPATH in ~/.bashrc so
# `from ros_client import AGVIOClient` works from anywhere. Idempotent.

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
LINE="export PYTHONPATH=\$PYTHONPATH:$SCRIPT_DIR"

if grep -Fqx "$LINE" ~/.bashrc 2>/dev/null; then
    echo "[skip] already in ~/.bashrc"
else
    echo "$LINE" >> ~/.bashrc
    echo "[added] appended to ~/.bashrc"
fi

echo "[next] run: source ~/.bashrc"
echo "[check] then: python3 -c \"from ros_client import AGVIOClient; print('ok')\""
