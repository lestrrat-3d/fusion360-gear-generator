#!/usr/bin/env bash
# Deploy the add-in to the Windows Fusion AddIns folder.
# Fusion's Aug 2026 update stopped loading add-ins from \\wsl.localhost paths,
# so the repo (WSL) is the source of truth and this copies the runtime files out.
set -euo pipefail

SRC="$(cd "$(dirname "$0")" && pwd)"
DEST="/mnt/c/Users/lestr/AppData/Roaming/Autodesk/Autodesk Fusion 360/API/AddIns/Gear Generator"

mkdir -p "$DEST"
rsync -r --delete \
    --exclude '__pycache__' \
    --include '/Gear Generator.py' \
    --include '/Gear Generator.manifest' \
    --include '/config.py' \
    --include '/commands/***' \
    --include '/lib/***' \
    --include '/diag_*.py' \
    --exclude '*' \
    "$SRC/" "$DEST/"

echo "Deployed to $DEST"
