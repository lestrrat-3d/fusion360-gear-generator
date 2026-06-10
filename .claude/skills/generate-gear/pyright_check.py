#!/usr/bin/env python3
"""Static-analysis gate for a generated gear file, using pyright + Fusion API stubs.

Supersedes the old `pyflakes` undefined-name check AND `check_adsk_modules.py`: with the
Autodesk Fusion API stubs on the path, pyright (standard mode) resolves `adsk.core` /
`adsk.fusion` and the framework's `from .misc import *` star-exports, so it catches BOTH
classes of bug the old tools split between them, generically and with no hand-maintained
name lists:
  - undefined names / typos      -> reportUndefinedVariable      (NameError at runtime)
  - wrong adsk submodule         -> reportAttributeAccessIssue   ("X is not a known
    e.g. adsk.fusion.SurfaceTypes                                 attribute of module ...")

Why standard mode, not --strict: the stubs are "intellisense only" (their own header says
so) and leave many Fusion return types unannotated, so --strict drowns the file in
thousands of reportUnknown* lines with zero extra real bugs. Standard mode is the usable
tier.

Two stub-driven false-positive families are filtered out (see classify()):
  - enum-as-int: stubs type enum MEMBERS as bare ints (NewBodyFeatureOperation = 3) but
    PARAMETERS as the enum class, so every idiomatic enum argument trips reportArgumentType.
  - reportUnknown* / missing-import noise for the un-stubbed bits.

Pyright must analyse the candidate from INSIDE the `lib/geargen/` package or its relative
imports (`from .misc import *`, `from ...lib import ...`) don't resolve and every
star-export becomes a phantom "undefined". So a candidate living elsewhere (the usual
`.tmp/<gear>.generated.py`) is copied to a throwaway module inside the package for the run.

Usage:
    python3 pyright_check.py <candidate.py> [--stubs <dir>] [--review]

Stubs dir resolution order:
    1. --stubs <dir>            (authoritative; a wrong path fails, no fallback)
    2. $FUSION_API_STUBS        (authoritative if set; a wrong path fails, no fallback)
    3. cached clone             (else clone the FusionAPIReference repo, sparse+shallow, into
                                 ~/.cache/fusion360-gear-generator/ and reuse it thereafter)
Any of these may point at the `defs` dir itself OR the FusionAPIReference checkout root; the
dir that contains `adsk/core.py` is located under it. The repo is 338M, the stubs 4M, so the
auto-clone fetches ONLY `Fusion_API_Python_Reference/defs` via a blobless sparse checkout.

Exit codes:
    0  no blocking findings (REVIEW items, if any, are printed for a human/agent glance)
    1  blocking findings (undefined name, wrong adsk module, or syntax error)
    2  setup error (stubs unresolvable/clone failed, pyright missing, candidate missing)
"""
import json
import os
import shutil
import subprocess
import sys

# Stub resolution/clone is shared with build_fusion_index.py (sibling module; sys.path[0] is
# this file's dir when run as a script). One clone, one resolution policy.
from fusion_stubs import resolve_defs, StubsUnavailable


def repo_root():
    # this file: <root>/.claude/skills/generate-gear/pyright_check.py
    here = os.path.dirname(os.path.abspath(__file__))
    return os.path.abspath(os.path.join(here, "..", "..", ".."))


def classify(diag):
    """Return 'BLOCK', 'REVIEW', or 'IGNORE' for one pyright diagnostic."""
    rule = diag.get("rule")
    msg = diag.get("message", "")
    sev = diag.get("severity", "error")

    # Syntax / parse errors carry no rule.
    if rule is None:
        return "BLOCK" if sev == "error" else "IGNORE"

    # Real bugs the old tools were there to catch.
    if rule == "reportUndefinedVariable":
        return "BLOCK"
    if rule == "reportAttributeAccessIssue" and "is not a known attribute of module" in msg:
        return "BLOCK"   # adsk.core vs adsk.fusion confusion

    # Known stub false positives.
    if rule == "reportArgumentType" and "None" not in msg:
        return "IGNORE"  # enum-member-typed-as-int vs enum-class parameter
    if rule.startswith("reportUnknown"):
        return "IGNORE"
    if rule in ("reportMissingImports", "reportMissingModuleSource"):
        return "IGNORE"  # un-stubbed adsk/futil sources

    # Everything else is genuine-but-needs-judgement: None-into-API
    # (reportArgumentType w/ None, reportOptionalMemberAccess), base-class attribute
    # access (forgot a .cast() / stub gap), etc.
    return "REVIEW"


def main():
    args = sys.argv[1:]
    stubs_arg = None
    if "--stubs" in args:
        i = args.index("--stubs")
        stubs_arg = args[i + 1]
        del args[i:i + 2]
    show_review = "--review" in args
    if show_review:
        args.remove("--review")
    if len(args) != 1:
        print(__doc__)
        sys.exit(2)
    candidate = os.path.abspath(args[0])
    if not os.path.isfile(candidate):
        print(f"ERROR: candidate not found: {candidate}")
        sys.exit(2)

    root = repo_root()
    pkg = os.path.join(root, "lib", "geargen")
    if not os.path.isdir(pkg):
        print(f"ERROR: lib/geargen not found under repo root {root}")
        sys.exit(2)

    # Resolve stubs (shared policy): --stubs and $FUSION_API_STUBS are authoritative (a wrong
    # path fails loudly, no silent fallback); otherwise clone the repo into the cache and reuse.
    try:
        stubs = resolve_defs(stubs_arg)
    except StubsUnavailable as e:
        print(f"ERROR: {e}")
        print("  Fall back to check_adsk_modules.py + pyflakes (stub-free) if stubs are "
              "unavailable.")
        sys.exit(2)

    tmp = os.path.join(root, ".tmp")
    os.makedirs(tmp, exist_ok=True)

    # Analyse from inside the package so relative imports resolve. If the candidate is
    # already in lib/geargen, use it in place; otherwise copy to a throwaway module.
    in_place = os.path.dirname(candidate) == pkg
    if in_place:
        target = candidate
        cleanup = None
    else:
        target = os.path.join(pkg, "__pyright_candidate__.py")
        shutil.copyfile(candidate, target)
        cleanup = target

    # pyright resolves `include` relative to the CONFIG FILE's directory and rejects
    # absolute include paths, so the config must sit at the repo root next to lib/.
    config_path = os.path.join(root, ".pyright_check.tmp.json")
    config = {
        "typeCheckingMode": "standard",
        "include": [os.path.relpath(target, root)],
        "extraPaths": [stubs],
        "reportMissingModuleSource": "none",
    }
    try:
        with open(config_path, "w") as fh:
            json.dump(config, fh)
        proc = subprocess.run(
            [sys.executable, "-m", "pyright", "-p", config_path, "--outputjson"],
            cwd=root, capture_output=True, text=True)
        if proc.returncode not in (0, 1) or not proc.stdout.strip():
            print("ERROR: pyright did not run. Install it with "
                  "`python3 -m pip install --break-system-packages pyright`.")
            print(proc.stderr.strip()[:500])
            sys.exit(2)
        data = json.loads(proc.stdout)
    finally:
        if cleanup and os.path.isfile(cleanup):
            os.remove(cleanup)
        if os.path.isfile(config_path):
            os.remove(config_path)

    blocking, review, ignored = [], [], 0
    for d in data.get("generalDiagnostics", []):
        verdict = classify(d)
        line = d.get("range", {}).get("start", {}).get("line", 0) + 1
        rec = (line, d.get("rule") or "syntax", d.get("message", "").splitlines()[0])
        if verdict == "BLOCK":
            blocking.append(rec)
        elif verdict == "REVIEW":
            review.append(rec)
        else:
            ignored += 1

    name = os.path.basename(candidate)
    if blocking:
        print(f"BLOCKING ({len(blocking)}) — fix the spec/playbook and regenerate:")
        for ln, rule, msg in sorted(blocking):
            print(f"  L{ln} [{rule}] {msg}")
    # REVIEW is advisory and does NOT gate: on correct, shipped code (spurgear.py) it runs
    # ~25-30, all stub pessimism / idiomatic downcasts. Collapse to a per-rule summary;
    # expand with --review only when chasing a specific runtime AttributeError/NoneType.
    if review:
        if show_review:
            print(f"REVIEW ({len(review)}) — advisory; mostly stub artifacts, NOT a gate:")
            for ln, rule, msg in sorted(review):
                print(f"  L{ln} [{rule}] {msg}")
        else:
            by_rule = {}
            for _, rule, _ in review:
                by_rule[rule] = by_rule.get(rule, 0) + 1
            summary = ", ".join(f"{r}×{n}" for r, n in sorted(by_rule.items()))
            print(f"REVIEW ({len(review)}) advisory (stub pessimism dominates; "
                  f"not a gate — re-run with --review to list): {summary}")
    print(f"pyright_check {name}: {len(blocking)} blocking, {len(review)} review, "
          f"{ignored} ignored (stub false positives)")
    sys.exit(1 if blocking else 0)


if __name__ == "__main__":
    main()
