#!/usr/bin/env bash
set -euo pipefail

here=$(cd "$(dirname "$0")" && pwd)
repo=$(cd "$here/.." && pwd)

common=$(git -C "$repo" rev-parse --path-format=absolute --git-common-dir)
main_repo=$(cd "$(dirname "$common")" && pwd)
sketch_dir=${SKETCH_DIR:-"$main_repo/../sketch"}
decad_dir=${DECAD_DIR:-"$main_repo/../decad"}

for module_dir in "$sketch_dir" "$decad_dir"; do
	if [[ ! -f "$module_dir/go.mod" ]]; then
		echo "Go module not found at: $module_dir" >&2
		exit 2
	fi
done

gowork="$repo/.tmp/proof-go.work"
mkdir -p "${gowork%/*}"
trap 'rm -f "$gowork" "${gowork%/*}/go.work"' EXIT
(
	cd "${gowork%/*}"
	go work init "$here"
	go work edit \
		-go=1.26.1 \
		"-replace=github.com/lestrrat-3d/sketch=$sketch_dir" \
		"-replace=github.com/lestrrat-3d/decad=$decad_dir" \
		go.work
	mv go.work "$gowork"
)

echo "using sketch engine at: $sketch_dir"
echo "using decad engine at: $decad_dir"
cd "$here"
GOWORK="$gowork" go test ./...
