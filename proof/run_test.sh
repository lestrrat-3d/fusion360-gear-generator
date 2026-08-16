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
printf 'module github.com/lestrrat-3d/fusion360-gear-generator/proof\ngo 1.26.1\n' > "$fixture/repo/proof/go.mod"
printf 'package proof\n' > "$fixture/repo/proof/proof.go"
printf 'package proof\n' > "$fixture/repo/proof/proof_test.go"
printf 'module github.com/lestrrat-3d/sketch\ngo 1.26.1\n' > "$fixture/sketch/go.mod"
printf 'module github.com/lestrrat-3d/decad\ngo 1.26.1\n' > "$fixture/decad/go.mod"

output=$(cd "$fixture/repo/proof" && ./run.sh)
printf '%s\n' "$output" | rg -Fq "using sketch engine at: $fixture/repo/../sketch"
printf '%s\n' "$output" | rg -Fq "using decad engine at: $fixture/repo/../decad"

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
rg -Fq "using sketch engine at: $fixture/repo/../sketch" "$first_output"
rg -Fq "using sketch engine at: $fixture/repo/../sketch" "$second_output"
rg -Fq "using decad engine at: $fixture/repo/../decad" "$first_output"
rg -Fq "using decad engine at: $fixture/repo/../decad" "$second_output"

printf '%s\n' 'proof/run.sh path fixture: OK'
