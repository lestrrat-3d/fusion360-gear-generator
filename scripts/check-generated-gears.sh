#!/usr/bin/env bash
# Runs `gearport check` for every gear that has a fragment under port/<dir>/fragments/, and names
# every gear whose lib/geargen/<gear>.py differs from a fresh generation before failing.
#
# Usage, from anywhere:
#   scripts/check-generated-gears.sh [path/to/gearport]
# Without an argument it builds gearport into .tmp/gearport first.
set -euo pipefail

root="$(cd "$(dirname "$0")/.." && pwd)"
cd "$root"

gearport="${1:-}"
if [[ -z "$gearport" ]]; then
    mkdir -p .tmp
    go build -C port -o ../.tmp/gearport ./cmd/gearport
    gearport=.tmp/gearport
fi

shopt -s nullglob
fragments=(port/*/fragments/*.py.frag)
if (( ${#fragments[@]} == 0 )); then
    echo "check-generated-gears: no fragments found under port/*/fragments/" >&2
    exit 1
fi

failed=()
for frag in "${fragments[@]}"; do
    gear=$(basename "$frag" .py.frag)
    echo "gearport check $gear"
    if ! "$gearport" check "$gear"; then
        failed+=("$gear")
    fi
done

if (( ${#failed[@]} > 0 )); then
    echo "check-generated-gears: not what the port generates: ${failed[*]}" >&2
    exit 1
fi
echo "check-generated-gears: ${#fragments[@]} gears match the port"
