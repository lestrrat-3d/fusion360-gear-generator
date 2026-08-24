#!/usr/bin/env python3
"""Run one gear's sketch-first bench and turn its verdict into an exit code.

Why this exists: `/generate-gear` step 3 (the sketch-first gate, `[PB-SKETCH-FIRST]`) used to
tell the orchestrating LLM to run `spec/<gear>/sketch/run.sh` and *read the output* to confirm
`Status == FullyConstrained` and healthy conditioning. Reading output is the failure mode the
gate runners were built to remove: the verdict gets skimmed, the advisory lines
(`ProfilesValid`, `Probe.Ambiguous()`) get mistaken for the gate, and nothing pins the
bench's own contract. This script runs the bench once and reports the result in the same
0/1/2 convention the other scripts in this directory use, so the orchestrator acts on an exit
code instead of on prose.

Classification uses the bench's exit code **plus** a verdict sentinel scanned from its output
(a line whose stripped text starts with `ALL PASS` or with `FAIL`). The sentinel is what
separates "the constraint scheme failed the gate" from "the bench itself is broken": a `go`
build failure inside `run.sh` also exits 1, and without the sentinel that would read as a
scheme defect. It also forces a future gear's bench to actually print a verdict rather than
exiting 0 after printing diagnostics only.

Bench output is passed through untouched; no advisory line is ever parsed, and nothing but
the final `[GATE]`/`[SETUP]` line is added.

Usage:
    run_sketch_bench.py <gear> [--root PATH] [--timeout SECONDS]

positional:
  gear                  gear name, e.g. spurgear. Names spec/<gear>/sketch/run.sh.

options:
  --root PATH           repo root. Default: three levels above this script.
  --timeout SECONDS     wall-clock limit for the bench. Default 900. The first run compiles
                        the sketch engine from source, so do not lower it casually.

Exit codes:
    0  the primary gate passed: the bench exited 0 and printed an `ALL PASS` verdict line.
       The constraint scheme fully constrains; proceed to generation.
    1  the primary gate failed: the bench exited nonzero and printed a `FAIL` verdict line.
       The constraint scheme does not fully constrain. That is a spec/playbook defect to fix
       here, never inside Fusion; never proceed to generation.
    2  setup error: no bench for this gear yet, the sketch-engine checkout was not found, the
       bench did not build, the bench timed out, or the bench ran but printed no verdict
       line. Fix the environment or build the bench; this is not a verdict on the scheme.
"""
import argparse
import os
import subprocess
import sys

DEFAULT_TIMEOUT = 900
PASS_SENTINEL = "ALL PASS"
FAIL_SENTINEL = "FAIL"


def repo_root():
    """Three levels above this script, the same rule run_gates.py uses."""
    return os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                        "..", "..", ".."))


def parse_args(argv):
    p = argparse.ArgumentParser(
        prog="run_sketch_bench.py",
        description="Run a gear's sketch-first bench and report its verdict as an exit code.")
    p.add_argument("gear", help="gear name, e.g. spurgear")
    p.add_argument("--root", default=None, help="repo root (default: three levels above "
                                                "this script)")
    p.add_argument("--timeout", type=float, default=DEFAULT_TIMEOUT,
                   help="wall-clock limit in seconds (default %d)" % DEFAULT_TIMEOUT)
    return p.parse_args(argv)


def bench_paths(root, gear):
    """(bench directory, run.sh path, root-relative run.sh path) for one gear."""
    rel = os.path.join("spec", gear, "sketch", "run.sh")
    script = os.path.join(root, rel)
    return os.path.dirname(script), script, rel


def verdict_sentinel(output):
    """The bench's verdict as 'pass', 'fail', or None when it printed neither.

    Matching is prefix-based on each stripped line, so the spur bench's real
    `ALL PASS — the spur Gear Profile constraint scheme fully constrains across sizes.`
    matches. `ALL PASS` is checked first because it also starts no `FAIL` line. The last
    verdict line in the output wins, so a per-case line cannot outvote the final one.
    """
    verdict = None
    for line in (output or "").splitlines():
        stripped = line.strip()
        if stripped.startswith(PASS_SENTINEL):
            verdict = "pass"
        elif stripped.startswith(FAIL_SENTINEL):
            verdict = "fail"
    return verdict


def last_meaningful_line(output):
    """The last non-empty line of the bench output, for quoting in a setup message."""
    for line in reversed((output or "").splitlines()):
        if line.strip():
            return line.strip()
    return ""


def run_bench(bench_dir, script, timeout):
    """Run `bash run.sh` in the bench directory, capturing stdout and stderr together.

    Returns (exit_code, output) with exit_code None when the bench timed out.
    """
    try:
        proc = subprocess.run(["bash", script], cwd=bench_dir, capture_output=True,
                              text=True, timeout=timeout, env=os.environ)
    except subprocess.TimeoutExpired as exc:
        return None, _decoded(exc.stdout) + _decoded(exc.stderr)
    return proc.returncode, (proc.stdout or "") + (proc.stderr or "")


def _decoded(chunk):
    if chunk is None:
        return ""
    if isinstance(chunk, bytes):
        return chunk.decode("utf-8", "replace")
    return chunk


def classify(exit_code, output, rel_script, timeout):
    """(verdict line, exit code) for a bench that ran. `exit_code` None means it timed out."""
    if exit_code is None:
        return ("[SETUP] bench timed out after %gs; raise --timeout, or fix a bench that "
                "does not terminate" % timeout, 2)

    verdict = verdict_sentinel(output)

    if exit_code == 0 and verdict == "pass":
        return "[GATE] sketch bench: PASS", 0

    if exit_code != 0 and verdict == "fail":
        return ("[GATE] sketch bench: FAIL — the constraint scheme does not fully constrain; "
                "this is a spec/playbook defect, fix there and re-run (never proceed to "
                "generation)", 1)

    if exit_code == 0:
        return ("[SETUP] %s exited 0 but printed no 'ALL PASS' verdict line; the bench does "
                "not implement the [PB-SKETCH-FIRST] verdict contract" % rel_script, 2)

    if exit_code == 2:
        return ("[SETUP] %s exited 2 (environment trouble, usually the sketch-engine checkout "
                "not found): %s" % (rel_script, last_meaningful_line(output)), 2)

    return ("[SETUP] %s exited %d but printed no 'FAIL' verdict line, so this is not a gate "
            "verdict — most likely the bench did not build: %s"
            % (rel_script, exit_code, last_meaningful_line(output)), 2)


def main(argv):
    args = parse_args(argv)
    root = os.path.abspath(args.root) if args.root else repo_root()
    bench_dir, script, rel_script = bench_paths(root, args.gear)

    if not os.path.isfile(script):
        print("[SETUP] no sketch bench at %s; build it from the spec's sketch recipes "
              "([PB-SKETCH-FIRST]) — spec/spurgear/sketch/ is the worked example" % rel_script)
        return 2

    exit_code, output = run_bench(bench_dir, script, args.timeout)
    if output:
        sys.stdout.write(output if output.endswith("\n") else output + "\n")

    line, code = classify(exit_code, output, rel_script, args.timeout)
    print(line)
    return code


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
