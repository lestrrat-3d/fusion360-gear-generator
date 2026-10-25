#!/usr/bin/env bash
# Puts Autodesk's Fusion API Python stubs (FusionAPIReference, Fusion_API_Python_Reference/defs only) at the commit
# pinned by FUSION_API_COMMIT in .github/workflows/checks.yml into .tmp/FusionAPIReference, for
# scripts/check_fusion_api.py. CI runs the same script, so a local check and CI read the same stubs.
#
# Usage, from anywhere:
#   scripts/fetch-fusion-api-stubs.sh
# A checkout already at the pinned commit is left as it is.
set -euo pipefail

root="$(cd "$(dirname "$0")/.." && pwd)"
workflow="$root/.github/workflows/checks.yml"
dest="$root/.tmp/FusionAPIReference"
url=https://github.com/AutodeskFusion360/FusionAPIReference.git

commit="$(sed -nE 's/^[[:space:]]*FUSION_API_COMMIT:[[:space:]]*([0-9a-f]{40}).*/\1/p' "$workflow")"
if [[ -z "$commit" ]]; then
    echo "fetch-fusion-api-stubs: no FUSION_API_COMMIT in $workflow" >&2
    exit 1
fi

if [[ -f "$dest/Fusion_API_Python_Reference/defs/adsk/core.py" ]] &&
    [[ "$(git -C "$dest" rev-parse HEAD 2>/dev/null)" == "$commit" ]]; then
    echo "fetch-fusion-api-stubs: $dest is at $commit"
    exit 0
fi

rm -rf "$dest"
mkdir -p "$dest"
git -C "$dest" init -q
git -C "$dest" remote add origin "$url"
git -C "$dest" sparse-checkout set --no-cone Fusion_API_Python_Reference/defs
git -C "$dest" fetch -q --depth 1 --filter=blob:none origin "$commit"
git -C "$dest" checkout -q FETCH_HEAD
test -f "$dest/Fusion_API_Python_Reference/defs/adsk/core.py"
echo "fetch-fusion-api-stubs: $dest is at $commit"
