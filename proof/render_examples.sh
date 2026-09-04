#!/usr/bin/env bash
# Regenerate the README's gear example images from the proof geometry.
#
# It wires the same local sketch engine run.sh does, so the outline check
# runs against the engine the proofs were run against rather than whatever
# the module cache happens to hold.
set -euo pipefail

here=$(cd "$(dirname "$0")" && pwd)
repo=$(cd "$here/.." && pwd)

common=$(git -C "$repo" rev-parse --path-format=absolute --git-common-dir)
main_repo=$(cd "$(dirname "$common")" && pwd)
sketch_dir=${SKETCH_DIR:-"$main_repo/../sketch"}

if [[ ! -f "$sketch_dir/go.mod" ]]; then
	echo "Go module not found at: $sketch_dir" >&2
	exit 2
fi

mkdir -p "$repo/.tmp"
work=$(mktemp -d "$repo/.tmp/render-go-work.XXXXXX")
gowork="$work/go.work"
trap 'rm -rf -- "$work"' EXIT
(
	cd "$work"
	go work init "$here"
	go work edit \
		-go=1.26.8 \
		"-replace=github.com/lestrrat-3d/sketch=$sketch_dir" \
		go.work
)

echo "using sketch engine at: $sketch_dir"
cd "$here"
GOWORK="$gowork" go run ./cmd/genexamples -out "$repo/docs/images/gears" "$@"
