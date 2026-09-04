#!/usr/bin/env bash
set -euo pipefail

here=$(cd "$(dirname "$0")" && pwd)
repo=$(cd "$here/.." && pwd)
mkdir -p "$repo/.tmp"
fixture=$(mktemp -d "$repo/.tmp/proof-run-fixture.XXXXXX")
trap 'rm -rf -- "$fixture"' EXIT

mkdir -p "$fixture/repo/proof" "$fixture/sketch" "$fixture/decad"
git -C "$fixture/repo" init --quiet
cp "$here/run.sh" "$fixture/repo/proof/run.sh"
chmod +x "$fixture/repo/proof/run.sh"
proof_gomod="$fixture/repo/proof/go.mod"
printf 'package proof\n' > "$fixture/repo/proof/proof.go"
printf 'package proof\n' > "$fixture/repo/proof/proof_test.go"
printf 'module github.com/lestrrat-3d/sketch\ngo 1.26.1\n' > "$fixture/sketch/go.mod"
printf 'module github.com/lestrrat-3d/decad\ngo 1.26.1\n' > "$fixture/decad/go.mod"
git -C "$fixture/sketch" init --quiet
git -C "$fixture/sketch" config user.email fixture@example.invalid
git -C "$fixture/sketch" config user.name fixture
git -C "$fixture/sketch" add go.mod
git -C "$fixture/sketch" commit --quiet -m fixture
git -C "$fixture/decad" init --quiet
git -C "$fixture/decad" config user.email fixture@example.invalid
git -C "$fixture/decad" config user.name fixture
git -C "$fixture/decad" add go.mod
git -C "$fixture/decad" commit --quiet -m fixture
sketch_commit=$(git -C "$fixture/sketch" rev-parse HEAD)
decad_commit=$(git -C "$fixture/decad" rev-parse HEAD)
sketch_short=$(git -C "$fixture/sketch" rev-parse --short=12 HEAD)
decad_short=$(git -C "$fixture/decad" rev-parse --short=12 HEAD)

# run.sh reads the pins out of go.mod, so the fixture's go.mod has to name its own
# throwaway commits in the pseudo-version form Go uses for an untagged module.
cat > "$proof_gomod" <<GOMOD
module github.com/lestrrat-3d/fusion360-gear-generator/proof

go 1.26.1

require (
	github.com/lestrrat-3d/decad v0.0.0-20260829122252-$decad_short
	github.com/lestrrat-3d/sketch v0.0.0-20260811135123-$sketch_short
)
GOMOD

output=$(cd "$fixture/repo/proof" && ./run.sh)
printf '%s\n' "$output" | grep -Fq "using sketch engine at: $fixture/repo/../sketch"
printf '%s\n' "$output" | grep -Fq "using decad engine at: $fixture/repo/../decad"
printf '%s\n' "$output" | grep -Fq "verified sketch revision: $sketch_short"
printf '%s\n' "$output" | grep -Fq "verified decad revision: $decad_short"

first_output="$fixture/first.out"
second_output="$fixture/second.out"
(
	cd "$fixture/repo/proof"
	./run.sh > "$first_output" 2>&1
) &
first_pid=$!
(
	cd "$fixture/repo/proof"
	./run.sh > "$second_output" 2>&1
) &
second_pid=$!

set +e
wait "$first_pid"
first_status=$?
wait "$second_pid"
second_status=$?
set -e
test "$first_status" -eq 0
test "$second_status" -eq 0
grep -Fq "using sketch engine at: $fixture/repo/../sketch" "$first_output"
grep -Fq "using sketch engine at: $fixture/repo/../sketch" "$second_output"
grep -Fq "using decad engine at: $fixture/repo/../decad" "$first_output"
grep -Fq "using decad engine at: $fixture/repo/../decad" "$second_output"
grep -Fq "verified sketch revision: $sketch_short" "$first_output"
grep -Fq "verified sketch revision: $sketch_short" "$second_output"
grep -Fq "verified decad revision: $decad_short" "$first_output"
grep -Fq "verified decad revision: $decad_short" "$second_output"

# A checkout that does not match go.mod fails closed rather than running. This is
# the case that matters: a run that quietly skipped verification is what let a
# proof pass locally against one engine and fail CI against another.
git -C "$fixture/decad" commit --quiet --allow-empty -m moved
set +e
output=$(cd "$fixture/repo/proof" && ./run.sh 2>&1)
status=$?
set -e
test "$status" -eq 2
printf '%s\n' "$output" | grep -Fq "Unexpected decad revision"
printf '%s\n' "$output" | grep -Fq "proof/go.mod pins decad at $decad_short"

# An ambient SKETCH_COMMIT must not change the answer; go.mod is the only source.
set +e
output=$(cd "$fixture/repo/proof" && SKETCH_COMMIT=0000000000000000000000000000000000000000 DECAD_COMMIT=1111 ./run.sh 2>&1)
status=$?
set -e
test "$status" -eq 2
printf '%s\n' "$output" | grep -Fq "proof/go.mod pins decad at $decad_short"

# PROOF_VERIFY_REVISIONS=0 is the deliberate opt-out, and it says so out loud.
git -C "$fixture/decad" reset --quiet --hard HEAD~1
output=$(cd "$fixture/repo/proof" && PROOF_VERIFY_REVISIONS=0 ./run.sh 2>&1)
printf '%s\n' "$output" | grep -Fq "go.mod not enforced"
printf '%s\n' "$output" | grep -vq "verified sketch revision"

printf '%s\n' 'proof/run.sh path fixture: OK'
