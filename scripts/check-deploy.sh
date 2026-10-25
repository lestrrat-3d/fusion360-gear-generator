#!/usr/bin/env bash
# Runs deploy.sh into a scratch add-in folder and checks what it shipped: every shipped Python file
# must compile, and nothing outside the add-in's runtime (tests, Go sources, port/, docs/, .tmp/,
# bytecode, diagnostic output) may be copied. The shipped set is whatever deploy.sh copies, so this
# check cannot drift from deploy.sh's include list.
#
# Usage, from anywhere:
#   scripts/check-deploy.sh
# The scratch folder is .tmp/deploy-check/ in the repository and is replaced on every run.
set -euo pipefail

root="$(cd "$(dirname "$0")/.." && pwd)"
scratch="$root/.tmp/deploy-check"
dest="$scratch/Gear Generator"

rm -rf "$scratch"
mkdir -p "$scratch"
FUSION_ADDIN_DIR="$dest" "$root/deploy.sh"

cd "$dest"
for required in "Gear Generator.py" "Gear Generator.manifest" LICENSE NOTICE; do
    if [[ ! -f "$required" ]]; then
        echo "check-deploy: deploy.sh did not ship $required" >&2
        exit 1
    fi
done

forbidden='(^|/)(tests|port|docs|spec|\.tmp|\.git|\.github|__pycache__)(/|$)|\.go$|\.pyc$|_out\.txt$'
bad="$(find . -mindepth 1 -printf '%P\n' | grep -E "$forbidden" || true)"
if [[ -n "$bad" ]]; then
    echo "check-deploy: deploy.sh shipped paths the add-in does not need:" >&2
    sed 's/^/  /' >&2 <<<"$bad"
    exit 1
fi

# Bytecode goes to the scratch folder's own cache prefix so the check leaves the shipped tree as it was.
PYTHONPYCACHEPREFIX="$scratch/pycache" python3 -m compileall -q -f .
count="$(find . -name '*.py' | wc -l)"
echo "check-deploy: deploy.sh shipped $count Python files and all compile"
