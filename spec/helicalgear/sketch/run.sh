#!/usr/bin/env bash
# Run the spur Gear Profile sketch-first proof (main.go) against a local checkout
# of github.com/lestrrat-3d/sketch, without committing a machine-specific replace
# path. The sketch engine is source-available (not go-gettable from the public
# proxy), so it must be resolved from a local checkout.
#
# Resolution order for the sketch repo:
#   1. $SKETCH_DIR if set;
#   2. otherwise a sibling checkout of the MAIN gear repo: <repo>/../sketch,
#      located via `git --git-common-dir` so it works from a worktree too.
#
# The replace is injected through a throwaway GOWORK file, so the committed
# go.mod stays portable.
set -euo pipefail

here=$(cd "$(dirname "$0")" && pwd)

if [[ -n "${SKETCH_DIR:-}" ]]; then
  sketch_dir=$SKETCH_DIR
else
  common=$(cd "$here" && git rev-parse --git-common-dir)
  main_repo=$(cd "$(dirname "$common")" && pwd)
  sketch_dir="$(cd "$main_repo/.." && pwd)/sketch"
fi

if [[ ! -f "$sketch_dir/go.mod" ]]; then
  echo "sketch repo not found at: $sketch_dir" >&2
  echo "set SKETCH_DIR=/path/to/lestrrat-3d/sketch and re-run" >&2
  exit 2
fi

work=$(mktemp -d)
trap 'rm -rf "$work"' EXIT
cat > "$work/go.work" <<WORK
go 1.24

use $here

replace github.com/lestrrat-3d/sketch => $sketch_dir
WORK

echo "using sketch engine at: $sketch_dir"
cd "$here"
GOWORK="$work/go.work" go run .
