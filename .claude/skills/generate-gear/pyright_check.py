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
import time
import re
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
    """Metadata for one Pyright invocation."""

    source_path: str
    target_path: str
    config_path: str
    exit_code: int | None = None
    source_paths: tuple[str, ...] = ()
    target_paths: tuple[str, ...] = ()
    duration_s: float | None = None


@dataclass
class AnalysisMetadata:
    """Execution metadata returned separately from raw diagnostics."""

    root: str
    stubs: str | None = None
    pyright_argv: tuple[str, ...] = ()
    invocations: list[AnalysisInvocation] = field(default_factory=list)
    duration_s: float | None = None
    requested_source_count: int = 0
    diagnostic_count: int = 0


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


def report_diagnostics(diagnostics, name, *, show_review=False):
    """Format the CLI report and counts for one source's raw diagnostics."""
    blocking, review, ignored = [], [], 0
    for diagnostic in diagnostics:
        verdict = classify(diagnostic)
        line = diagnostic.get("range", {}).get("start", {}).get("line", 0) + 1
        record = (line, diagnostic.get("rule") or "syntax",
                  diagnostic.get("message", "").splitlines()[0])
        if verdict == "BLOCK":
            blocking.append(record)
        elif verdict == "REVIEW":
            review.append(record)
        else:
            ignored += 1

    lines = []
    if blocking:
        lines.append("BLOCKING (%d) — fix the spec/playbook and regenerate:" % len(blocking))
        lines.extend("  L%d [%s] %s" % record for record in sorted(blocking))
    if review:
        if show_review:
            lines.append("REVIEW (%d) — advisory; mostly stub artifacts, NOT a gate:" %
                         len(review))
            lines.extend("  L%d [%s] %s" % record for record in sorted(review))
        else:
            by_rule = {}
            for _, rule, _ in review:
                by_rule[rule] = by_rule.get(rule, 0) + 1
            summary = ", ".join("%s×%d" % (rule, count)
                                for rule, count in sorted(by_rule.items()))
            lines.append("REVIEW (%d) advisory (stub pessimism dominates; not a gate — "
                         "re-run with --review to list): %s" %
                         (len(review), summary))
    lines.append("pyright_check %s: %d blocking, %d review, %d ignored (stub false positives)"
                 % (name, len(blocking), len(review), ignored))
    return {
        "text": "\n".join(lines),
        "blocking": blocking,
        "review": review,
        "ignored": ignored,
    }


def _reported_paths(reported, root, config_directory):
    """Return absolute forms Pyright may use for a reported file name."""
    if not reported:
        return set()
    if os.path.isabs(reported):
        return {os.path.abspath(reported)}
    return {
        os.path.abspath(os.path.join(root, reported)),
        os.path.abspath(os.path.join(config_directory, reported)),
    }


def _remap_diagnostic(diagnostic, target_to_source, root, config_directory):
    """Copy a diagnostic and restore the original path after internal candidate copies."""
    result = dict(diagnostic)
    for reported_path in _reported_paths(result.get("file"), root, config_directory):
        source = target_to_source.get(reported_path)
        if source is None:
            source = target_to_source.get(os.path.realpath(reported_path))
        if source is not None:
            result["file"] = source
            break
    return result


def normalize_diagnostic(diagnostic, source, root=None):
    """Return stable, root-relative fields used to compare standalone and batch runs."""
    resolved_root = os.path.abspath(root or repo_root())
    reported = diagnostic.get("file") or source
    source_path = os.path.abspath(source)
    if reported:
        reported = os.path.abspath(os.path.normpath(reported))
    else:
        reported = source_path
    try:
        relative = os.path.relpath(reported, resolved_root)
    except ValueError:
        relative = reported
    if relative.startswith(".."):
        relative = reported
    relative = relative.replace(os.sep, "/")
    relative = re.sub(r"(?:^|/)(__pyright_candidate_|\.pyright-check-)[^/]+", "", relative)
    rng = diagnostic.get("range") or {}
    start = rng.get("start") or {}
    end = rng.get("end") or {}
    return {
        "source": relative,
        "severity": diagnostic.get("severity", "error"),
        "rule": diagnostic.get("rule"),
        "start": {"line": start.get("line", 0), "character": start.get("character", 0)},
        "end": {"line": end.get("line", 0), "character": end.get("character", 0)},
        "message": diagnostic.get("message", ""),
    }


def normalized_diagnostics(result, root=None):
    """Return deterministically sorted normalized diagnostics for all requested sources."""
    values = []
    for source in sorted(result.diagnostics):
        for diagnostic in result.diagnostics[source]:
            values.append(normalize_diagnostic(diagnostic, source, root or result.metadata.root))
    return sorted(values, key=lambda item: (
        item["source"], item["start"]["line"], item["start"]["character"],
        item["end"]["line"], item["end"]["character"], item["rule"] or "", item["message"]))


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
                  pyright_argv=None, quiet=True, timeout=None):
    """Analyze source paths in one Pyright invocation.

    Candidates outside ``root/lib`` are copied into unique temporary modules under
    ``lib/geargen`` so package-relative imports continue to work. The config includes every
    target in deterministic input order, and all files created by this call are removed on every
    return path. ``setup_error`` is explicit and means an empty diagnostics mapping is not a
    successful analysis.
    """
    normalized, path_error = _valid_paths(paths)
    resolved_root = os.path.abspath(root or repo_root())
    metadata = AnalysisMetadata(root=resolved_root)
    if path_error:
        return AnalysisResult({}, metadata, path_error)
    metadata.requested_source_count = len(normalized)

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
    targets = []
    cleanup_targets = []
    target_to_sources = {}
    source_realpaths = set()
    for source in normalized:
        target = source
        source_realpath = os.path.realpath(source)
        aliased_source = source_realpath in source_realpaths
        source_realpaths.add(source_realpath)
        if os.path.dirname(source) != os.path.abspath(pkg) or aliased_source:
            cleanup_target = None
            try:
                cleanup_target = _scratch_file(pkg, "__pyright_candidate_", ".py")
                shutil.copyfile(source, cleanup_target)
                target = cleanup_target
                cleanup_targets.append(cleanup_target)
            except OSError as error:
                _remove_scratch(cleanup_target)
                for path in cleanup_targets:
                    _remove_scratch(path)
                return AnalysisResult(diagnostics, metadata,
                                      f"could not prepare candidate {source}: {error}")
        targets.append(target)
        target_to_sources.setdefault(os.path.abspath(target), []).append(source)
        target_to_sources.setdefault(os.path.realpath(target), []).append(source)

    config_path = None
    invocation = None
    started = time.monotonic()
    try:
        config_path = _scratch_file(config_directory, ".pyright-check-", ".json")
        invocation = AnalysisInvocation(
            normalized[0], targets[0], config_path,
            source_paths=tuple(normalized), target_paths=tuple(targets))
        metadata.invocations.append(invocation)
        config = {
            "typeCheckingMode": "standard",
            "include": [os.path.relpath(target, config_directory) for target in targets],
            "extraPaths": [stubs_path],
            "reportMissingModuleSource": "none",
        }
        try:
            with open(config_path, "w", encoding="utf-8") as handle:
                json.dump(config, handle)
        except (OSError, TypeError, ValueError) as error:
            return AnalysisResult(diagnostics, metadata,
                                  f"could not write Pyright config {config_path}: {error}")
        run_kwargs = dict(cwd=resolved_root, capture_output=True, text=True,
                          env={**os.environ, **extra_env})
        if timeout is not None:
            run_kwargs["timeout"] = timeout
        try:
            proc = subprocess.run(
                list(pyright_argv) + ["-p", config_path, "--outputjson"], **run_kwargs)
        except subprocess.TimeoutExpired:
            return AnalysisResult(diagnostics, metadata,
                                  "pyright analysis timed out after %ss" % timeout)
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
        for diagnostic in data["generalDiagnostics"]:
            if not isinstance(diagnostic, dict):
                return AnalysisResult(diagnostics, metadata,
                                      "pyright returned malformed JSON: diagnostic is not an object")
            reported = _reported_paths(diagnostic.get("file"), resolved_root, config_directory)
            matching = []
            for reported_path in reported:
                matching.extend(target_to_sources.get(reported_path, ()))
            matching = list(dict.fromkeys(matching))
            if matching:
                for source in matching:
                    remapped = dict(diagnostic)
                    remapped["file"] = source
                    diagnostics[source].append(remapped)
            elif len(normalized) == 1:
                # Imported files are attributed to the sole requested source, matching the
                # standalone checker behavior from task 01.
                remapped = _remap_diagnostic(diagnostic, {}, resolved_root, config_directory)
                diagnostics[normalized[0]].append(remapped)
            else:
                return AnalysisResult(
                    diagnostics, metadata,
                    "pyright reported a diagnostic for an unknown source: %s" %
                    diagnostic.get("file"))
        metadata.diagnostic_count = sum(len(items) for items in diagnostics.values())
        return AnalysisResult(diagnostics, metadata)
    finally:
        metadata.duration_s = round(time.monotonic() - started, 2)
        if invocation is not None:
            invocation.duration_s = metadata.duration_s
        _remove_scratch(config_path)
        for path in cleanup_targets:
            _remove_scratch(path)


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

    name = os.path.basename(candidate)
    report = report_diagnostics(result.diagnostics.get(candidate, []), name,
                                show_review=show_review)
    print(report["text"])
    sys.exit(1 if report["blocking"] else 0)


if __name__ == "__main__":
    main()
