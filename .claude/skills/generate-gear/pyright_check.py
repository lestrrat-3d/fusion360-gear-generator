#!/usr/bin/env python3
"""Static-analysis gate for a generated gear file, using pyright + Fusion API stubs.

Supersedes the old `pyflakes` undefined-name check: with the Autodesk Fusion API stubs on
the path, pyright (standard mode) resolves `adsk.core` / `adsk.fusion` and the framework's
`from .misc import *` star-exports, so it catches BOTH classes of bug the old tools split
between them, generically and with no hand-maintained name lists:
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
    python3 pyright_check.py <candidate.py> [--stubs <dir>] [--review] [--no-install]

Pyright resolution order:
    1. importable module        `import pyright` works -> run `python -m pyright` as-is
    2. PATH binary              a `pyright` executable on $PATH
    3. cached install           a previous `pip install --target` under
                                ~/.cache/fusion360-gear-generator/, put on PYTHONPATH
    4. auto-install             else pip-install the pyright wrapper into that cache
`--no-install` forbids step 4 (no network, no provisioning) and exits 2 instead. Nothing outside
the cache dir is ever written — the user's Python environment is left alone. The wrapper's first
run may additionally download node / the pyright npm bundle into its own cache.

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
    2  setup error (stubs unresolvable/clone failed, pyright unresolvable/bootstrap failed,
       candidate missing)
"""
import json
import os
import shutil
import subprocess
import sys
import tempfile
from dataclasses import dataclass, field

HERE = os.path.dirname(os.path.abspath(__file__))

# Stub resolution/clone lives in a sibling module (sys.path[0] is this file's dir when run as a
# script). One clone, one resolution policy.
try:
    from .fusion_stubs import resolve_defs, StubsUnavailable
    from .pyright_boot import resolve_pyright, PyrightUnavailable
except ImportError:
    if HERE not in sys.path:
        sys.path.insert(0, HERE)
    from fusion_stubs import resolve_defs, StubsUnavailable
    from pyright_boot import resolve_pyright, PyrightUnavailable


@dataclass
class AnalysisInvocation:
    """Metadata for one serial Pyright invocation."""

    source_path: str
    target_path: str
    config_path: str
    exit_code: int | None = None


@dataclass
class AnalysisMetadata:
    """Execution metadata returned separately from raw diagnostics."""

    root: str
    stubs: str | None = None
    pyright_argv: tuple[str, ...] = ()
    invocations: list[AnalysisInvocation] = field(default_factory=list)


@dataclass
class AnalysisResult:
    """Raw Pyright findings grouped by source path and an optional setup failure.

    ``diagnostics`` contains the unclassified JSON diagnostic objects returned by Pyright.
    Its keys are the original absolute paths passed to :func:`analyze_paths`, even when a
    candidate was copied into ``lib/geargen`` to resolve relative imports. ``metadata`` records
    the tool, config, and temporary target used for each invocation. ``setup_error`` is a
    human-readable string when analysis could not run; callers must treat that as an exit-2
    setup failure instead of interpreting an empty diagnostics mapping as success.
    """

    diagnostics: dict[str, list[dict]]
    metadata: AnalysisMetadata
    setup_error: str | None = None

    @property
    def findings(self):
        """Alias for callers that use findings as the name for raw diagnostics."""
        return self.diagnostics

    @property
    def ok(self):
        """Whether every requested path produced valid Pyright JSON."""
        return self.setup_error is None


class AnalysisSetupError(Exception):
    """Raised by adapters that need to turn an :class:`AnalysisResult` failure into exit 2."""


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


def _remap_diagnostic(diagnostic, target, source, root, config_directory):
    """Copy a diagnostic and restore the source path after an internal candidate copy."""
    result = dict(diagnostic)
    reported = result.get("file")
    reported_paths = set()
    if reported:
        if os.path.isabs(reported):
            reported_paths.add(os.path.abspath(reported))
        else:
            reported_paths.add(os.path.abspath(os.path.join(root, reported)))
            reported_paths.add(os.path.abspath(os.path.join(config_directory, reported)))
    if os.path.abspath(target) in reported_paths:
        result["file"] = source
    return result


def _valid_paths(paths):
    """Normalize and validate paths before resolving tools or creating scratch files."""
    if isinstance(paths, (str, bytes, os.PathLike)):
        paths = [paths]
    try:
        values = [os.path.abspath(os.fspath(path)) for path in paths]
    except (TypeError, ValueError):
        return None, "analysis paths must be an iterable of filesystem paths"
    if not values:
        return None, "analysis needs at least one source path"
    for path in values:
        if not os.path.isfile(path):
            return None, f"candidate not found: {path}"
        try:
            with open(path, "rb"):
                pass
        except OSError as error:
            return None, f"candidate is unreadable: {path}: {error}"
    return values, None


def _scratch_file(directory, prefix, suffix):
    """Create a unique empty scratch file and return its path."""
    descriptor, path = tempfile.mkstemp(prefix=prefix, suffix=suffix, dir=directory)
    os.close(descriptor)
    return path


def _remove_scratch(path):
    """Remove a scratch file owned by this invocation, tolerating a concurrent cleanup."""
    if not path:
        return
    try:
        os.remove(path)
    except FileNotFoundError:
        pass
    except OSError:
        pass


def analyze_paths(paths, *, stubs=None, no_install=False, root=None,
                  pyright_argv=None, quiet=True):
    """Analyze source paths and return raw findings grouped by their original paths.

    Pyright runs once per path in deterministic input order. Candidates outside ``root/lib``
    are copied into a unique temporary module under ``lib/geargen`` so package-relative imports
    continue to work. A unique config file is created for every invocation, includes are relative
    to that config's directory, and both files are removed on every return path. Callers receive
    :class:`AnalysisResult`; ``setup_error`` is explicit and means the empty diagnostics mapping
    is not a successful analysis.
    """
    normalized, path_error = _valid_paths(paths)
    resolved_root = os.path.abspath(root or repo_root())
    metadata = AnalysisMetadata(root=resolved_root)
    if path_error:
        return AnalysisResult({}, metadata, path_error)

    pkg = os.path.join(resolved_root, "lib", "geargen")
    tmp = os.path.join(resolved_root, ".tmp")
    if not os.path.isdir(pkg):
        return AnalysisResult({}, metadata,
                              f"lib/geargen not found under repo root {resolved_root}")
    try:
        os.makedirs(tmp, exist_ok=True)
    except OSError as error:
        return AnalysisResult({}, metadata, f"could not create scratch directory {tmp}: {error}")

    try:
        stubs_path = resolve_defs(stubs, quiet=quiet)
    except StubsUnavailable as error:
        return AnalysisResult({}, metadata, str(error))
    metadata.stubs = stubs_path

    if pyright_argv is None:
        try:
            pyright_argv, extra_env = resolve_pyright(no_install=no_install, quiet=quiet)
        except PyrightUnavailable as error:
            return AnalysisResult({}, metadata, str(error))
    else:
        pyright_argv = list(pyright_argv)
        extra_env = {}
    metadata.pyright_argv = tuple(pyright_argv)

    diagnostics = {source: [] for source in normalized}
    config_directory = tmp
    for source in normalized:
        target = source
        cleanup_target = None
        if os.path.dirname(source) != os.path.abspath(pkg):
            try:
                cleanup_target = _scratch_file(pkg, "__pyright_candidate_", ".py")
                shutil.copyfile(source, cleanup_target)
                target = cleanup_target
            except OSError as error:
                _remove_scratch(cleanup_target)
                return AnalysisResult(diagnostics, metadata,
                                      f"could not prepare candidate {source}: {error}")

        config_path = None
        invocation = None
        try:
            config_path = _scratch_file(config_directory, ".pyright-check-", ".json")
            invocation = AnalysisInvocation(source, target, config_path)
            metadata.invocations.append(invocation)
            config = {
                "typeCheckingMode": "standard",
                "include": [os.path.relpath(target, config_directory)],
                "extraPaths": [stubs_path],
                "reportMissingModuleSource": "none",
            }
            try:
                with open(config_path, "w", encoding="utf-8") as handle:
                    json.dump(config, handle)
            except (OSError, TypeError, ValueError) as error:
                return AnalysisResult(diagnostics, metadata,
                                      f"could not write Pyright config {config_path}: {error}")
            try:
                proc = subprocess.run(
                    list(pyright_argv) + ["-p", config_path, "--outputjson"],
                    cwd=resolved_root, capture_output=True, text=True,
                    env={**os.environ, **extra_env})
            except (OSError, subprocess.SubprocessError) as error:
                return AnalysisResult(diagnostics, metadata,
                                      f"pyright failed to execute: {error}")
            invocation.exit_code = proc.returncode
            if proc.returncode not in (0, 1):
                detail = str(proc.stderr or "").strip()[:500]
                message = "pyright exited with status %d" % proc.returncode
                if detail:
                    message += ": " + detail
                return AnalysisResult(diagnostics, metadata, message)
            try:
                data = json.loads(proc.stdout)
            except (TypeError, json.JSONDecodeError) as error:
                return AnalysisResult(diagnostics, metadata,
                                      f"pyright returned malformed JSON: {error}")
            if not isinstance(data, dict) or not isinstance(data.get("generalDiagnostics"), list):
                return AnalysisResult(diagnostics, metadata,
                                      "pyright returned malformed JSON: generalDiagnostics is missing")
            source_diagnostics = []
            for diagnostic in data["generalDiagnostics"]:
                if not isinstance(diagnostic, dict):
                    return AnalysisResult(diagnostics, metadata,
                                          "pyright returned malformed JSON: diagnostic is not an object")
                source_diagnostics.append(_remap_diagnostic(
                    diagnostic, target, source, resolved_root, config_directory))
            diagnostics.setdefault(source, []).extend(source_diagnostics)
        finally:
            _remove_scratch(config_path)
            _remove_scratch(cleanup_target)
    return AnalysisResult(diagnostics, metadata)


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
    no_install = "--no-install" in args
    if no_install:
        args.remove("--no-install")
    if len(args) != 1:
        print(__doc__)
        sys.exit(2)
    candidate = os.path.abspath(args[0])
    result = analyze_paths([candidate], stubs=stubs_arg, no_install=no_install, quiet=False)
    if result.setup_error:
        print(f"ERROR: {result.setup_error}")
        if "Fusion API stubs" in result.setup_error or "stubs" in result.setup_error:
            print("  Stub-free fallback: pyflakes for undefined names, and the fusion plugin's "
                  "compiled database")
            print("  ('query_fusion_api.py show <Name>', see fusion_api.py) for the adsk submodule.")
        sys.exit(2)

    blocking, review, ignored = [], [], 0
    for d in result.diagnostics.get(candidate, []):
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
