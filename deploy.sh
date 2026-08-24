#!/usr/bin/env bash
# Deploy the add-in to Fusion's AddIns folder.
# Fusion's Aug 2026 update stopped loading add-ins from \\wsl.localhost paths,
# so the repo is the source of truth and this copies the runtime files out.
#
# Destination resolution, first hit wins:
#   1. $FUSION_ADDIN_DIR — the "Gear Generator" folder itself, not its parent.
#   2. .deploy.conf next to this script (untracked), a shell fragment that
#      sets FUSION_ADDIN_DIR.
#   3. Auto-discovery of the one "Gear Generator" folder under the standard
#      per-OS AddIns locations listed in standard_addin_parents below.
set -euo pipefail

SRC="$(cd "$(dirname "$0")" && pwd)"

if [[ -z "${FUSION_ADDIN_DIR:-}" && -f "$SRC/.deploy.conf" ]]; then
    # shellcheck source=/dev/null
    source "$SRC/.deploy.conf"
fi

standard_addin_parents() {
    # WSL view of a Windows install, any Windows user profile.
    local d
    for d in /mnt/c/Users/*/AppData/Roaming/Autodesk/"Autodesk Fusion 360"/API/AddIns; do
        [[ -d "$d" ]] && printf '%s\n' "$d"
    done
    # Native Windows shells (Git Bash / MSYS) expose APPDATA.
    if [[ -n "${APPDATA:-}" && -d "$APPDATA/Autodesk/Autodesk Fusion 360/API/AddIns" ]]; then
        printf '%s\n' "$APPDATA/Autodesk/Autodesk Fusion 360/API/AddIns"
    fi
    # macOS.
    if [[ -d "$HOME/Library/Application Support/Autodesk/Autodesk Fusion 360/API/AddIns" ]]; then
        printf '%s\n' "$HOME/Library/Application Support/Autodesk/Autodesk Fusion 360/API/AddIns"
    fi
}

if [[ -z "${FUSION_ADDIN_DIR:-}" ]]; then
    mapfile -t parents < <(standard_addin_parents)
    if [[ ${#parents[@]} -eq 1 ]]; then
        FUSION_ADDIN_DIR="${parents[0]}/Gear Generator"
        echo "Auto-detected AddIns folder: ${parents[0]}"
    else
        {
            echo "deploy.sh: cannot pick a destination (${#parents[@]} standard AddIns folders found)."
            echo "Set FUSION_ADDIN_DIR to the 'Gear Generator' folder, either in the environment"
            echo "or in $SRC/.deploy.conf (a line: FUSION_ADDIN_DIR=/path/to/AddIns/Gear Generator)."
            echo "Fusion's standard AddIns locations:"
            echo "  Windows (from WSL): /mnt/c/Users/<you>/AppData/Roaming/Autodesk/Autodesk Fusion 360/API/AddIns"
            echo "  Windows (native):   %APPDATA%\\Autodesk\\Autodesk Fusion 360\\API\\AddIns"
            echo "  macOS:              ~/Library/Application Support/Autodesk/Autodesk Fusion 360/API/AddIns"
        } >&2
        exit 2
    fi
fi

mkdir -p "$FUSION_ADDIN_DIR"
rsync -r --delete \
    --exclude '__pycache__' \
    --include '/Gear Generator.py' \
    --include '/Gear Generator.manifest' \
    --include '/config.py' \
    --include '/commands/***' \
    --include '/lib/***' \
    --include '/diag_*.py' \
    --exclude '*' \
    "$SRC/" "$FUSION_ADDIN_DIR/"

echo "Deployed to $FUSION_ADDIN_DIR"
