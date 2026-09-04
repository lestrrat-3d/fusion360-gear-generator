#!/usr/bin/env bash
set -euo pipefail

here=$(cd "$(dirname "$0")" && pwd)
repo=$(cd "$here/.." && pwd)

common=$(git -C "$repo" rev-parse --path-format=absolute --git-common-dir)
main_repo=$(cd "$(dirname "$common")" && pwd)
sketch_dir=${SKETCH_DIR:-"$main_repo/../sketch"}
decad_dir=${DECAD_DIR:-"$main_repo/../decad"}

# The engine revisions are already pinned, in proof/go.mod, by the pseudo-version
# Go records for a module with no tags: the 12 hex digits after the last dash are
# the commit. That is the one place they are written. Reading them here rather
# than keeping a second copy is what stops a local run and CI from drifting apart,
# and it means `go get` is the only thing that ever moves a pin.
pinned_revision() {
	local module=$1
	sed -n "s|^[[:space:]]*github.com/lestrrat-3d/$module v[^ ]*-\([0-9a-f]\{12\}\)$|\1|p" \
		"$here/go.mod" | head -1
}

verify_revision() {
	local name=$1
	local module_dir=$2
	local expected=$3
	local actual

	if [[ -z "$expected" ]]; then
		echo "proof/go.mod names no pseudo-version for $name, so its revision cannot be pinned" >&2
		exit 2
	fi
	# go.mod carries the 12-digit short commit, so compare at that width.
	actual=$(git -C "$module_dir" rev-parse --short=12 --verify HEAD^{commit} 2>/dev/null || true)
	if [[ "$actual" != "$expected" ]]; then
		echo "Unexpected $name revision at $module_dir: got ${actual:-none}, want $expected" >&2
		echo "proof/go.mod pins $name at $expected; check that revision out, or run with" >&2
		echo "PROOF_VERIFY_REVISIONS=0 to run against whatever is there." >&2
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

# Verifying is the default, and that is deliberate. When it was opt-in, a local
# run silently used whatever engine checkout sat beside the repo; a proof that
# passed locally against a newer engine then failed CI against the pin, twice,
# and the second time the green local run was taken as evidence the proof was
# sound. Failing closed is what makes a local run mean something. There is no
# environment override: the point is that every run uses the same revision, so
# the only way to move one is to move it in go.mod.
if [[ "${PROOF_VERIFY_REVISIONS:-1}" != 0 ]]; then
	verify_revision sketch "$sketch_dir" "$(pinned_revision sketch)"
	verify_revision decad "$decad_dir" "$(pinned_revision decad)"
else
	echo "PROOF_VERIFY_REVISIONS=0: running against whatever is checked out, go.mod not enforced" >&2
fi

mkdir -p "$repo/.tmp"
work=$(mktemp -d "$repo/.tmp/proof-go-work.XXXXXX")
gowork="$work/go.work"
trap 'rm -rf -- "$work"' EXIT
(
	cd "$work"
	go work init "$here"
	go work edit \
		-go=1.26.8 \
		"-replace=github.com/lestrrat-3d/sketch=$sketch_dir" \
		"-replace=github.com/lestrrat-3d/decad=$decad_dir" \
		go.work
)

echo "using sketch engine at: $sketch_dir"
echo "using decad engine at: $decad_dir"
cd "$here"
GOWORK="$gowork" go test ./...
