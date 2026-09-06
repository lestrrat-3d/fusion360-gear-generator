#!/usr/bin/env bash
set -euo pipefail

here=$(cd "$(dirname "$0")" && pwd)
repo=$(cd "$here/.." && pwd)

# run.sh [--package ./PATTERN]... [-- GO_TEST_ARG...]
#
#   run.sh
#   run.sh --package ./bevelgear -- -run '^TestGearProfiles$' -count=1
#   run.sh --package ./proofkit --package ./proofkit3d -- -race -count=1
#
# Selection exists so a focused local run does not pay for every gear. It is not a
# way to run less than the suite promises: a bad option fails here, before any
# reassuring setup output, and the engine verification below runs whatever was
# selected. Parsing comes first so a typo costs nothing.
usage="usage: run.sh [--package ./PATTERN]... [-- GO_TEST_ARG...]"
packages=()
go_args=()

while [[ $# -gt 0 ]]; do
	case $1 in
	--package)
		if [[ $# -lt 2 ]]; then
			echo "--package needs a package pattern" >&2
			echo "$usage" >&2
			exit 2
		fi
		# Module-relative only. An import path or an absolute directory would name
		# something outside this module, and `..` would climb out of proof/.
		case $2 in
		./*) ;;
		*)
			echo "package pattern must begin with ./: '$2'" >&2
			echo "$usage" >&2
			exit 2
			;;
		esac
		if [[ "/$2/" == */../* ]]; then
			echo "package pattern must stay inside proof/: '$2'" >&2
			echo "$usage" >&2
			exit 2
		fi
		packages+=("$2")
		shift 2
		;;
	--)
		# Everything after this belongs to go test, one array element per token, so
		# a -run expression holding a space stays one argument. Go validates them.
		shift
		go_args=("$@")
		break
		;;
	*)
		echo "unknown option: '$1'" >&2
		echo "$usage" >&2
		exit 2
		;;
	esac
done

# No --package means the whole suite, including when only go-test flags were given.
# ./... rather than .: a bare . would build the one package holding no gear proof
# at all and still report success.
if [[ ${#packages[@]} -eq 0 ]]; then
	packages=(./...)
fi

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
# A focused selector must match at least one registered test. Go otherwise exits
# successfully when -run matches nothing, which can hide a misspelled proof name.
selected=0
for arg in "${go_args[@]}"; do
	case $arg in
	-run|-run=*) selected=1 ;;
	esac
done
if [[ $selected -eq 1 ]]; then
	# Observe the actual run instead of executing the selected tests twice.
	# Verbose output records test starts; -json callers receive run events.
	go_args+=(-v)
fi
# From proof/, so a package pattern means the same thing wherever run.sh was called.
cd "$here"
printf 'running: go test'
printf ' %q' "${go_args[@]}" "${packages[@]}"
printf '\n'
set +e
GOWORK="$gowork" go test "${go_args[@]}" "${packages[@]}" 2>&1 | tee "$work/test-output"
status=$?
set -e
if [[ $status -ne 0 ]]; then
	exit 1
fi
if [[ $selected -eq 1 ]] && ! grep -Eq '^=== RUN   |"Action":"run"' "$work/test-output"; then
	echo "focused selector matched zero tests" >&2
	exit 1
fi
