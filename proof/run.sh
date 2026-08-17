#!/usr/bin/env bash
set -euo pipefail

here=$(cd "$(dirname "$0")" && pwd)
repo=$(cd "$here/.." && pwd)

common=$(git -C "$repo" rev-parse --path-format=absolute --git-common-dir)
main_repo=$(cd "$(dirname "$common")" && pwd)
sketch_dir=${SKETCH_DIR:-"$main_repo/../sketch"}
decad_dir=${DECAD_DIR:-"$main_repo/../decad"}

verify_revision() {
	local name=$1
	local module_dir=$2
	local expected=$3
	local actual

	actual=$(git -C "$module_dir" rev-parse --verify HEAD^{commit} 2>/dev/null || true)
	if [[ "$actual" != "$expected" ]]; then
		echo "Unexpected $name revision at $module_dir: got ${actual:-none}, want $expected" >&2
		exit 2
	fi
	echo "verified $name revision: $actual"
}

for module_dir in "$sketch_dir" "$decad_dir"; do
	if [[ ! -f "$module_dir/go.mod" ]]; then
		echo "Go module not found at: $module_dir" >&2
		exit 2
	fi
done

if [[ "${PROOF_VERIFY_REVISIONS:-}" == 1 ]]; then
	: "${SKETCH_COMMIT:?SKETCH_COMMIT is required when PROOF_VERIFY_REVISIONS=1}"
	: "${DECAD_COMMIT:?DECAD_COMMIT is required when PROOF_VERIFY_REVISIONS=1}"
	verify_revision sketch "$sketch_dir" "$SKETCH_COMMIT"
	verify_revision decad "$decad_dir" "$DECAD_COMMIT"
fi

mkdir -p "$repo/.tmp"
work=$(mktemp -d "$repo/.tmp/proof-go-work.XXXXXX")
gowork="$work/go.work"
trap 'rm -rf -- "$work"' EXIT
(
	cd "$work"
	go work init "$here"
	go work edit \
		-go=1.26.1 \
		"-replace=github.com/lestrrat-3d/sketch=$sketch_dir" \
		"-replace=github.com/lestrrat-3d/decad=$decad_dir" \
		go.work
)

echo "using sketch engine at: $sketch_dir"
echo "using decad engine at: $decad_dir"
cd "$here"
GOWORK="$gowork" go test ./...
