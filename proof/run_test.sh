#!/usr/bin/env bash
set -euo pipefail

here=$(cd "$(dirname "$0")" && pwd)
repo=$(cd "$here/.." && pwd)
mkdir -p "$repo/.tmp"
fixture=$(mktemp -d "$repo/.tmp/proof-run-fixture.XXXXXX")
trap 'rm -rf -- "$fixture"' EXIT

# Absence is checked through this rather than through `! ... | grep -Fq`, because
# `set -e` ignores a command whose status is inverted with `!`: such a line passes
# whatever the output says. `grep -vq` is the same trap by another route — it asks
# whether any line differs, which is nearly always true.
refute() {
	local text=$1
	local needle=$2
	if printf '%s\n' "$text" | grep -Fq -- "$needle"; then
		printf 'unexpected in output: %s\n' "$needle" >&2
		exit 1
	fi
}

mkdir -p "$fixture/repo/proof" "$fixture/sketch" "$fixture/decad"
git -C "$fixture/repo" init --quiet
cp "$here/run.sh" "$fixture/repo/proof/run.sh"
chmod +x "$fixture/repo/proof/run.sh"
proof_gomod="$fixture/repo/proof/go.mod"
module=github.com/lestrrat-3d/fusion360-gear-generator/proof
printf 'package proof\n' > "$fixture/repo/proof/proof.go"
printf 'package proof\n' > "$fixture/repo/proof/proof_test.go"

# Two packages, so a selected run can be told from a full one by which `ok` lines
# come back rather than by anything the runner says about itself. The space in the
# subtest name is the point of the quoting cases below: Go reports it as
# `name_with_space`, so a -run expression that still contains a real space is what
# proves the token reached go test whole.
mkdir -p "$fixture/repo/proof/alpha" "$fixture/repo/proof/beta"
printf 'package alpha\n' > "$fixture/repo/proof/alpha/alpha.go"
cat > "$fixture/repo/proof/alpha/alpha_test.go" <<'GO'
package alpha

import (
	"os"
	"testing"
)

func TestAlpha(t *testing.T) {}

func TestSpaced(t *testing.T) {
	t.Run("name with space", func(t *testing.T) {})
	t.Run("plain sibling", func(t *testing.T) {})
}

func TestConditionalFailure(t *testing.T) {
	if os.Getenv("PROOF_FIXTURE_FAIL") != "" {
		t.Fatal("deliberate fixture failure")
	}
}

func TestRunsOnce(t *testing.T) {
	path := os.Getenv("PROOF_FIXTURE_ONCE")
	if path == "" {
		return
	}
	file, err := os.OpenFile(path, os.O_WRONLY|os.O_CREATE|os.O_EXCL, 0600)
	if err != nil {
		t.Fatalf("focused test ran more than once: %v", err)
	}
	if err := file.Close(); err != nil {
		t.Fatal(err)
	}
}
GO
printf 'package beta\n' > "$fixture/repo/proof/beta/beta.go"
cat > "$fixture/repo/proof/beta/beta_test.go" <<'GO'
package beta

import "testing"

func TestBeta(t *testing.T) {}
GO

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

# No arguments still means the whole module. `./...` is named out loud because the
# runner reporting a narrower selection than it ran would be the dangerous failure.
printf '%s\n' "$output" | grep -Fqx 'running: go test ./...'
printf '%s\n' "$output" | grep -Eq "^ok[[:space:]]+$module/alpha([[:space:]]|\$)"
printf '%s\n' "$output" | grep -Eq "^ok[[:space:]]+$module/beta([[:space:]]|\$)"
test "$(printf '%s\n' "$output" | grep -c '^ok')" -eq 3

# One --package runs that package and nothing else.
output=$(cd "$fixture/repo/proof" && ./run.sh --package ./alpha)
printf '%s\n' "$output" | grep -Fqx 'running: go test ./alpha'
printf '%s\n' "$output" | grep -Eq "^ok[[:space:]]+$module/alpha([[:space:]]|\$)"
test "$(printf '%s\n' "$output" | grep -c '^ok')" -eq 1

# Repeating it selects exactly that set, in the order given.
output=$(cd "$fixture/repo/proof" && ./run.sh --package ./alpha --package ./beta)
printf '%s\n' "$output" | grep -Fqx 'running: go test ./alpha ./beta'
printf '%s\n' "$output" | grep -Eq "^ok[[:space:]]+$module/alpha([[:space:]]|\$)"
printf '%s\n' "$output" | grep -Eq "^ok[[:space:]]+$module/beta([[:space:]]|\$)"
test "$(printf '%s\n' "$output" | grep -c '^ok')" -eq 2

# A package pattern means the same thing from anywhere, because run.sh resolves it
# from proof/ rather than from the caller's directory.
output=$(cd "$fixture/repo" && ./proof/run.sh --package ./beta)
printf '%s\n' "$output" | grep -Eq "^ok[[:space:]]+$module/beta([[:space:]]|\$)"
test "$(printf '%s\n' "$output" | grep -c '^ok')" -eq 1

# Go-test flags with no --package still run the whole module. Selecting nothing must
# not quietly become selecting `.`, which would build the one package that holds no
# gear proof and report success.
output=$(cd "$fixture/repo/proof" && ./run.sh -- -count=1)
printf '%s\n' "$output" | grep -Fqx 'running: go test -count=1 ./...'
test "$(printf '%s\n' "$output" | grep -c '^ok')" -eq 3

# -run and its expression survive as two tokens. Split by the shell, `_]space` would
# reach go test as a package pattern and the run would fail instead of filtering.
output=$(cd "$fixture/repo/proof" && ./run.sh --package ./alpha -- -v -count=1 \
	-run 'TestSpaced/name_with[ _]space')
printf '%s\n' "$output" | grep -Fq 'running: go test -v -count=1 -run TestSpaced/name_with\[\ _\]space -v ./alpha'
printf '%s\n' "$output" | grep -Fq -- '--- PASS: TestSpaced/name_with_space'
refute "$output" plain_sibling

# A focused run from the repository root executes the selected test exactly once.
output=$(cd "$fixture/repo" && PROOF_FIXTURE_ONCE="$fixture/once" ./proof/run.sh \
	--package ./alpha -- -count=1 -run='^TestRunsOnce$')
test -f "$fixture/once"
printf '%s\n' "$output" | grep -Fq -- '--- PASS: TestRunsOnce'

# JSON callers retain structured output and the same selection check.
output=$(cd "$fixture/repo" && ./proof/run.sh --package ./alpha -- -json -count=1 -run='^TestAlpha$')
printf '%s\n' "$output" | grep -Fq '"Action":"run"'

# A misspelled focused selector is a runner failure rather than a false green.
set +e
output=$(cd "$fixture/repo/proof" && ./run.sh --package ./alpha -- -count=1 -run '^TestMissing$' 2>&1)
status=$?
set -e
test "$status" -eq 1
printf '%s\n' "$output" | grep -Fq 'focused selector matched zero tests'

set +e
output=$(cd "$fixture/repo" && ./proof/run.sh --package ./alpha -- -json -count=1 -run='^TestMissing$' 2>&1)
status=$?
set -e
test "$status" -eq 1
printf '%s\n' "$output" | grep -Fq 'focused selector matched zero tests'

# A selected test that fails fails the run; selection is not a way to be told
# everything passed.
set +e
output=$(cd "$fixture/repo/proof" && PROOF_FIXTURE_FAIL=1 ./run.sh --package ./alpha -- \
	-count=1 -run '^TestConditionalFailure$' 2>&1)
status=$?
set -e
test "$status" -ne 0
printf '%s\n' "$output" | grep -Fq 'deliberate fixture failure'

# A go-test flag run.sh does not recognise is Go's to reject, and Go rejecting it is
# still a failed run.
set +e
output=$(cd "$fixture/repo/proof" && ./run.sh --package ./alpha -- -nosuchflag 2>&1)
status=$?
set -e
test "$status" -ne 0
printf '%s\n' "$output" | grep -Fq 'flag provided but not defined'

# Runner-option mistakes fail before any setup output, so a typo never sits under a
# line saying the engines were verified.
assert_rejected() {
	local needle=$1
	shift
	local out status
	set +e
	out=$(cd "$fixture/repo/proof" && ./run.sh "$@" 2>&1)
	status=$?
	set -e
	test "$status" -eq 2
	printf '%s\n' "$out" | grep -Fq -- "$needle"
	printf '%s\n' "$out" | grep -Fq 'usage: run.sh'
	refute "$out" 'verified sketch revision'
	refute "$out" 'using sketch engine at'
	refute "$out" 'running: go test'
}

assert_rejected '--package needs a package pattern' --package
assert_rejected 'unknown option:' --bogus
assert_rejected 'unknown option:' -run TestAlpha
assert_rejected 'package pattern must begin with ./' --package alpha
assert_rejected 'package pattern must begin with ./' --package "$module/alpha"
assert_rejected 'package pattern must begin with ./' --package ''
assert_rejected 'package pattern must stay inside proof/' --package ./../sketch

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

# Selecting a package is not permission to run against an unpinned engine.
set +e
output=$(cd "$fixture/repo/proof" && ./run.sh --package ./alpha -- -count=1 2>&1)
status=$?
set -e
test "$status" -eq 2
printf '%s\n' "$output" | grep -Fq "Unexpected decad revision"
refute "$output" 'running: go test'

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
refute "$output" "verified sketch revision"

# Every run above built its go.work under .tmp and trapped it away again, including
# the ones that failed.
leftover=$(find "$fixture/repo/.tmp" -maxdepth 1 -name 'proof-go-work.*' -print)
test -z "$leftover"

printf '%s\n' 'proof/run.sh path fixture: OK'
