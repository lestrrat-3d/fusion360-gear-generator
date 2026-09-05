#!/usr/bin/env bash
# Regenerate the README's gear example images from the proof geometry.
#
# Everything here runs against the engine revisions proof/go.mod pins, out of the
# module cache, and there is no local-checkout wiring for that reason: an image
# rendered against an engine the proofs were never run against would be a picture
# of a gear nothing checked. go.mod is the one place a revision is written, which
# is what run.sh verifies a local checkout against, so reading the images off the
# same file is what keeps a picture and its proof on one engine.
#
# Two stages, because the geometry lives in two places. The spur, helical and
# herringbone gears are drawn from proof/involute by cmd/genexamples. The bevel
# gear and the cycloidal drive are drawn by an opt-in test inside their own proof
# package, where the lattice and the bodies they are built from already live; the
# test is skipped whenever -render.out is not given, so an ordinary proof run
# writes no images.
set -euo pipefail

here=$(cd "$(dirname "$0")" && pwd)
repo=$(cd "$here/.." && pwd)
out=$repo/docs/images/gears

mkdir -p "$out"
cd "$here"
export GOWORK=off

go run ./cmd/genexamples -out "$out" "$@"
# -count=1 because a cached PASS writes no file.
go test ./bevelgear ./cycloidal -run TestRenderExample -render.out="$out" -count=1 -v |
	grep -E '^\s+render_test\.go:[0-9]+: wrote '
