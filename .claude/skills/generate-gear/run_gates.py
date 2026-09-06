#!/usr/bin/env python3
"""Run the whole gate battery for one gear and print one verdict.

It serves both stages that validate a candidate module: `/emit-gear` step 3 and
`/generate-gear` step 5. Both used to ask the orchestrating LLM to run the gate commands one
at a time, then (at the emit stage) classify any failure by hand against the fault table in
`.claude/skills/emit-gear/SKILL.md`. Every part of that is mechanical — the commands are
fixed, their arguments are derived from `<gear>`, their exit codes already say pass/fail —
and leaving it to the model meant gates silently dropped under context pressure, argument
order improvised (`check_contract.py` takes the manifest first, `check_step_calls.py` takes
the step list first), and a retry loop that saw one failure at a time instead of all of
them. This script runs the fixed battery once, in a fixed order, and reports every result
plus a first-pass fault classification.

Usage:
    run_gates.py <gear> [candidate] [options]

positional:
  gear                  gear name, e.g. spurgear. Names spec/<gear>/steps.md and
                         spec/<gear>/contract.json.
  candidate              path to the candidate Python file.
                         Default: .tmp/<gear>.generated.py (relative to --root).

options:
  --root PATH            repo root. Default: three levels above this script.
  --only KEY[,KEY...]    run only these gates. Keys: parse, pyright, input_read,
                         contract, step_calls, anchors, api_calls, novel_types.
                         Unselected gates are reported with status "skip",
                         reason "not selected".
  --fail-fast            stop scheduling gates after the first failure. Off by default.
  --require-contract     a missing spec/<gear>/contract.json fails the run instead of
                         skipping the contract gate.
  --skip-missing-steps   a missing spec/<gear>/steps.md skips the step_calls gate instead
                         of being a setup error. This is what /generate-gear passes: it
                         runs from the prose spec, so a gear may have no compiled step
                         list. When steps.md exists the gate runs as usual.
  --gate-novel-types     pass --gate to check_novel_types.py, making its findings
                         blocking (this is what CI does).
  --no-advisory          do not run check_novel_types.py at all.
  --json-out PATH        also write the full JSON verdict (pretty-printed) to PATH.
  --format {text,json}   text (default) = human report followed by one JSON line.
                         json = the JSON line only, nothing else on stdout.
  --timeout SECONDS      per-gate timeout. Default 900. A gate that exceeds it gets
                         status "error".

Exit codes:
    0  every gate that ran passed. Skips are allowed. Advisory findings may exist
       (unless --gate-novel-types).
    1  at least one gate failed on content. Classify with the printed table: emit fault
       -> back to the drafter with the report text appended; compile fault -> stop.
    2  setup error: bad usage, missing candidate, missing spec/<gear>/steps.md, a gate
       script missing from the skill directory, a gate exited 2, a gate timed out, or
       --require-contract with no manifest. Fix the environment or the inputs; never
       retry the drafter.
"""
import argparse
import ast
import dataclasses
import importlib.util
import json
import os
import re
import subprocess
import sys
import time
from dataclasses import dataclass

# --- constants ---------------------------------------------------------------------------
SCHEMA = 1
DEFAULT_TIMEOUT = 900
GATE_ORDER = ("parse", "input_read", "contract", "step_calls",
              "anchors", "api_calls", "pyright", "novel_types")
CANDIDATE_READING = frozenset({"input_read", "contract", "step_calls",
                                "api_calls", "pyright", "novel_types"})
JSON_MARKER = "GATES_JSON: "

GATE_TITLES = {
    "parse": "Parse",
    "input_read": "Input reads",
    "contract": "Contract",
    "step_calls": "Step calls",
    "anchors": "Anchors",
    "api_calls": "API calls",
    "pyright": "Types",
    "novel_types": "Novel types",
}

GATE_SCRIPTS = {
    "input_read": "check_input_read.py",
    "contract": "check_contract.py",
    "step_calls": "check_step_calls.py",
    "anchors": "check_anchors.py",
    "api_calls": "check_api_calls.py",
    "pyright": "pyright_check.py",
    "novel_types": "check_novel_types.py",
}

FAULT_LABEL = {
    "emit": "EMIT FAULT",
    "compile": "COMPILE FAULT",
    "judgment": "NEEDS JUDGMENT",
    "setup": "SETUP ERROR",
}

NOT_DECIDED_NOTE = (
    'not decided here (SKILL.md rows the runner leaves to the reviewing agent): '
    '"A wrong argument type where the step gave the signature", "A step gives an argument '
    'type the signature rejects", "A step is too thin to act on", and "Two steps '
    'contradict each other".')

HEADLINE_PATTERN = re.compile(r'^\S.*check.*:')
API_CALL_NAME_RE = re.compile(r"calls '(\w+)\('")
NOVEL_COUNT_RE = re.compile(r'novel-type check: (\d+) complaint')


# --- small value types (dataclasses) ------------------------------------------------------
@dataclass
class GateResult:
    key: str
    title: str
    status: str
    advisory: bool
    exit_code: int | None
    duration_s: float | None
    command: list[str]
    stdout: str
    stderr: str
    skip_reason: str | None
    fault: str | None
    timing_note: str | None = None


@dataclass
class Paths:
    root: str            # absolute
    candidate: str        # root-relative when under root, else absolute
    steps: str            # root-relative "spec/<gear>/steps.md"
    contract: str          # root-relative "spec/<gear>/contract.json"


# --- path and argument plumbing ------------------------------------------------------------
def scripts_dir():
    """Directory holding the gate scripts. Its own __file__ dir. Patched by tests."""
    return os.path.dirname(os.path.abspath(__file__))


def repo_root():
    """Three levels above scripts_dir(), same rule pyright_check.repo_root() uses."""
    return os.path.abspath(os.path.join(scripts_dir(), "..", "..", ".."))


def _abs(root, path):
    return path if os.path.isabs(path) else os.path.join(root, path)


def _root_relative(abs_path, root):
    try:
        rel = os.path.relpath(abs_path, root)
    except ValueError:
        return abs_path
    return abs_path if rel.startswith("..") else rel


def parse_args(argv):
    p = argparse.ArgumentParser(
        prog="run_gates.py",
        description="Run the gate battery for one gear and print one verdict.")
    p.add_argument("gear")
    p.add_argument("candidate", nargs="?", default=None)
    p.add_argument("--root", default=None)
    p.add_argument("--only", default=None,
                    help="comma-separated gate keys to run; the rest are skipped")
    p.add_argument("--fail-fast", action="store_true")
    p.add_argument("--require-contract", action="store_true")
    p.add_argument("--skip-missing-steps", action="store_true",
                    help="a missing spec/<gear>/steps.md skips the step_calls gate instead "
                         "of being a setup error (generate stage)")
    p.add_argument("--gate-novel-types", action="store_true")
    p.add_argument("--no-advisory", action="store_true")
    p.add_argument("--json-out", default=None)
    p.add_argument("--format", choices=["text", "json"], default="text")
    p.add_argument("--timeout", type=float, default=DEFAULT_TIMEOUT)
    args = p.parse_args(argv)
    if args.only:
        args.only = [k.strip() for k in args.only.split(",") if k.strip()]
    return args


def resolve_paths(gear, candidate_arg, root_arg):
    """Absolutise the root, default the candidate to .tmp/<gear>.generated.py, and express
    every path root-relative so printed commands are copy-pasteable from the repo root."""
    root = os.path.abspath(root_arg) if root_arg else repo_root()
    if candidate_arg:
        cand_abs = os.path.abspath(candidate_arg)
    else:
        cand_abs = os.path.join(root, ".tmp", "%s.generated.py" % gear)
    candidate = _root_relative(cand_abs, root)
    steps = os.path.join("spec", gear, "steps.md")
    contract = os.path.join("spec", gear, "contract.json")
    return Paths(root=root, candidate=candidate, steps=steps, contract=contract)


def _active_keys(args):
    if args.only:
        keys = [k for k in GATE_ORDER if k in args.only]
    else:
        keys = list(GATE_ORDER)
    if args.no_advisory and "novel_types" in keys:
        keys = [k for k in keys if k != "novel_types"]
    return keys


def setup_errors(paths, args):
    """Pre-flight, returns human messages. Checks: root is a directory; candidate exists;
    steps.md exists unless --skip-missing-steps; every gate script the plan needs exists in
    scripts_dir();
    --require-contract implies contract.json exists; --only names are known keys."""
    errors = []
    if not os.path.isdir(paths.root):
        errors.append("repo root is not a directory: %s" % paths.root)
        return errors

    if args.only:
        unknown = sorted(set(args.only) - set(GATE_ORDER))
        if unknown:
            errors.append("unknown --only key(s): %s" % ", ".join(unknown))

    cand_abs = _abs(paths.root, paths.candidate)
    if not os.path.isfile(cand_abs):
        errors.append("candidate not found: %s" % paths.candidate)

    if not args.skip_missing_steps:
        steps_abs = _abs(paths.root, paths.steps)
        if not os.path.isfile(steps_abs):
            errors.append(
                "step list not found: %s -- /emit-gear cannot run without it; "
                "run /compile-gear first" % paths.steps)

    if args.require_contract:
        contract_abs = _abs(paths.root, paths.contract)
        if not os.path.isfile(contract_abs):
            errors.append(
                "--require-contract given but no manifest at %s" % paths.contract)

    for key in _active_keys(args):
        script_name = GATE_SCRIPTS.get(key)
        if script_name and not os.path.isfile(os.path.join(scripts_dir(), script_name)):
            errors.append("gate script missing: %s (needed for '%s')" % (script_name, key))

    return errors


# --- gate plan ------------------------------------------------------------------------------
def _script_path(name, paths):
    return _root_relative(os.path.join(scripts_dir(), name), paths.root)


def gate_command(key, paths, args):
    """The one place that knows each gate's argument order. Uses sys.executable, root-relative
    script and input paths."""
    py = sys.executable
    if key == "parse":
        return [py, "-c", "import ast; ast.parse(open(%r).read())" % paths.candidate]
    if key == "input_read":
        return [py, _script_path("check_input_read.py", paths), paths.candidate]
    if key == "contract":
        return [py, _script_path("check_contract.py", paths), paths.contract, paths.candidate]
    if key == "step_calls":
        return [py, _script_path("check_step_calls.py", paths), paths.steps, paths.candidate]
    if key == "anchors":
        return [py, _script_path("check_anchors.py", paths)]
    if key == "api_calls":
        return [py, _script_path("check_api_calls.py", paths), paths.candidate]
    if key == "pyright":
        return [py, _script_path("pyright_check.py", paths), paths.candidate]
    if key == "novel_types":
        cmd = [py, _script_path("check_novel_types.py", paths), paths.candidate]
        if args.gate_novel_types:
            cmd.append("--gate")
        return cmd
    raise ValueError("unknown gate key: %s" % key)


def build_plan(paths, args):
    """(key, title, command, skip_reason). Command is None exactly when skip_reason is set.
    Applies --only, --no-advisory, --gate-novel-types, the missing-contract skip, and (under
    --skip-missing-steps) the missing-step-list skip.
    Order is GATE_ORDER."""
    active = set(_active_keys(args))
    plan = []
    for key in GATE_ORDER:
        title = GATE_TITLES[key]
        if key not in active:
            plan.append((key, title, None, "not selected"))
            continue
        if key == "contract" and not os.path.isfile(_abs(paths.root, paths.contract)):
            reason = (
                "no manifest at %s -- the prose contract check is the reviewing agent's "
                "job (generate-gear/SKILL.md); pass --require-contract to make this fatal"
                % paths.contract)
            plan.append((key, title, None, reason))
            continue
        if key == "step_calls" and args.skip_missing_steps \
                and not os.path.isfile(_abs(paths.root, paths.steps)):
            reason = ("no compiled step list at %s -- the generate stage runs from the "
                      "prose spec; omit --skip-missing-steps to make this fatal"
                      % paths.steps)
            plan.append((key, title, None, reason))
            continue
        plan.append((key, title, gate_command(key, paths, args), None))
    return plan


# --- execution --------------------------------------------------------------------------------
def run_parse_gate(paths, timeout):
    """In-process ast.parse of the candidate. On SyntaxError, status 'fail', exit_code 1,
    stdout 'parse: SyntaxError at L<line>:<col>: <msg>'. Its `command` field still records the
    SKILL.md one-liner so the report shows a command a human can re-run."""
    command = [sys.executable, "-c", "import ast; ast.parse(open(%r).read())" % paths.candidate]
    start = time.monotonic()
    cand_abs = _abs(paths.root, paths.candidate)
    try:
        with open(cand_abs, encoding="utf-8") as fh:
            src = fh.read()
    except OSError as exc:
        duration = round(time.monotonic() - start, 2)
        return GateResult("parse", "Parse", "error", False, 2, duration, command,
                           "", str(exc), None, "setup")
    try:
        ast.parse(src, filename=cand_abs)
    except SyntaxError as exc:
        duration = round(time.monotonic() - start, 2)
        stdout = "parse: SyntaxError at L%s:%s: %s" % (exc.lineno, exc.offset, exc.msg)
        return GateResult("parse", "Parse", "fail", False, 1, duration, command,
                           stdout, "", None, None)
    duration = round(time.monotonic() - start, 2)
    return GateResult("parse", "Parse", "pass", False, 0, duration, command,
                       "parse: OK", "", None, None)


def run_script_gate(key, title, command, cwd, timeout, advisory):
    """subprocess.run(command, cwd=cwd, capture_output=True, text=True, timeout=timeout,
    env=os.environ). Maps: rc 0 -> pass, rc 1 -> fail, anything else -> error.
    TimeoutExpired -> status 'error' with a synthetic message."""
    start = time.monotonic()
    try:
        proc = subprocess.run(command, cwd=cwd, capture_output=True, text=True,
                               timeout=timeout, env=os.environ)
    except subprocess.TimeoutExpired as exc:
        duration = round(time.monotonic() - start, 2)
        stdout = exc.stdout or ""
        stderr = (exc.stderr or "") + "\ngate timed out after %ss" % timeout
        return GateResult(key, title, "error", advisory, None, duration, command,
                           stdout, stderr, None, "setup")
    duration = round(time.monotonic() - start, 2)
    if proc.returncode == 0:
        status = "pass"
    elif proc.returncode == 1:
        status = "fail"
    else:
        status = "error"
    fault = "setup" if status == "error" else None
    return GateResult(key, title, status, advisory, proc.returncode, duration, command,
                       proc.stdout, proc.stderr, None, fault)


def load_type_modules():
    """Load the reusable analysis and novel-type modules when this is a real gate suite."""
    module_paths = {
        "pyright": os.path.join(scripts_dir(), "pyright_check.py"),
        "novel_types": os.path.join(scripts_dir(), "check_novel_types.py"),
    }
    if not all(os.path.isfile(path) for path in module_paths.values()):
        return None
    try:
        with open(module_paths["pyright"], encoding="utf-8") as handle:
            if "def analyze_paths" not in handle.read():
                return None
        with open(module_paths["novel_types"], encoding="utf-8") as handle:
            if "def evaluate_analysis" not in handle.read():
                return None
    except OSError:
        return None
    modules = {}
    for key, path in module_paths.items():
        try:
            name = "_run_gates_%s_%d" % (key, id(path))
            spec = importlib.util.spec_from_file_location(name, path)
            module = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(module)
        except (ImportError, OSError, SyntaxError, SystemExit):
            return None
        modules[key] = module
    if not hasattr(modules["pyright"], "analyze_paths"):
        return None
    if not hasattr(modules["novel_types"], "evaluate_analysis"):
        return None
    return modules


def _type_error_result(key, title, command, message, duration, timing_note=None):
    return GateResult(key, title, "error", key == "novel_types", 2, duration, command,
                      "", "ERROR: " + message, None, "setup", timing_note)


def _pyright_result(module, diagnostics, paths, command, duration, timing_note=None):
    """Build the normal type gate row from raw candidate diagnostics."""
    report = module.report_diagnostics(diagnostics, os.path.basename(paths.candidate))
    status = "fail" if report["blocking"] else "pass"
    return GateResult("pyright", "Types", status, False,
                      1 if report["blocking"] else 0, duration, command, report["text"], "",
                      None, None, timing_note)


def run_shared_type_gates(selected, paths, args, commands, metadata):
    """Run selected type consumers from one run-local Pyright analysis result.

    ``None`` means the installed gate scripts are test doubles or an older checkout, so the
    caller should use the normal subprocess path. ``metadata`` receives additive timing fields.
    """
    modules = load_type_modules()
    if modules is None:
        return None
    pyright = modules["pyright"]
    novel_types = modules["novel_types"]
    candidate = _abs(paths.root, paths.candidate)
    references = []
    if "novel_types" in selected:
        references = novel_types.reference_gears(
            os.path.join(paths.root, "lib", "geargen"), candidate)
    analysis_paths = references + [candidate]
    started = time.monotonic()
    try:
        result = pyright.analyze_paths(analysis_paths, root=paths.root, timeout=args.timeout)
    except Exception as error:
        result = None
        setup_error = str(error)
    else:
        setup_error = getattr(result, "setup_error", None)
    outer_elapsed = round(time.monotonic() - started, 2)
    inner_elapsed = getattr(getattr(result, "metadata", None), "duration_s", None)
    elapsed = outer_elapsed
    metadata.update({
        "analysis_duration_s": elapsed,
        "analysis_pyright_duration_s": inner_elapsed,
        "analysis_invocations": (len(getattr(result.metadata, "invocations", []))
                                 if result is not None else 0),
        "analysis_shared": len(selected) > 1,
        "analysis_timing_note": ("The first selected type row owns the shared Pyright duration; "
                                  "later rows use that result and report 0 seconds."),
    })

    rows = {}
    first = selected[0]
    for key in selected:
        row_duration = elapsed if key == first else 0.0
        timing_note = ("shared analysis duration" if key == first
                       else "classification uses shared analysis; duration is in first type row")
        if setup_error:
            rows[key] = _type_error_result(
                key, GATE_TITLES[key], commands[key], setup_error, row_duration, timing_note)
            continue
        if key == "pyright":
            diagnostics = result.diagnostics.get(os.path.abspath(candidate), [])
            rows[key] = _pyright_result(
                pyright, diagnostics, paths, commands[key], row_duration, timing_note)
        elif key == "novel_types":
            if not references:
                rows[key] = _type_error_result(
                    key, GATE_TITLES[key], commands[key],
                    "no reference gears under %s, nothing to compare against" %
                    os.path.join(paths.root, "lib", "geargen"), row_duration, timing_note)
                continue
            try:
                evaluation = novel_types.evaluate_analysis(result, candidate, references)
                output, exit_code = novel_types.render_evaluation(
                    evaluation, gate=args.gate_novel_types)
                status = "fail" if exit_code == 1 else "pass"
                if status == "pass" and evaluation["novel"] and not args.gate_novel_types:
                    status = "note"
                rows[key] = GateResult(
                    key, GATE_TITLES[key], status, True,
                    exit_code, row_duration, commands[key], output, "", None, None,
                    timing_note)
            except Exception as error:
                rows[key] = _type_error_result(
                    key, GATE_TITLES[key], commands[key], str(error), row_duration, timing_note)
    return rows


def classify_advisory(result, args):
    """check_novel_types exits 0 both when clean and when it found complaints (without --gate).
    Detect findings with NOVEL_COUNT_RE and set status 'note' when the count is > 0 and the
    gate is not blocking. With --gate-novel-types the exit code is authoritative and rc 1 is a
    plain fail."""
    if args.gate_novel_types:
        return result
    if result.status != "pass":
        return result
    match = NOVEL_COUNT_RE.search(result.stdout or "")
    count = int(match.group(1)) if match else 0
    if count > 0:
        return dataclasses.replace(result, status="note")
    return result


def execute(plan, paths, args, analysis_metadata=None):
    """Walk the plan. If the parse gate fails, convert every not-yet-run CANDIDATE_READING gate
    to status 'skip', reason 'candidate does not parse'. With --fail-fast, convert every
    remaining gate to 'skip', reason 'not run (--fail-fast after <key> failed)'."""
    results = []
    analysis_metadata = analysis_metadata if analysis_metadata is not None else {}
    parse_failed = False
    fail_fast_stop = None
    type_keys = [key for key, _title, command, skip_reason in plan
                 if key in ("pyright", "novel_types") and command is not None
                 and skip_reason is None]
    shared_rows = None
    shared_attempted = False
    for key, title, command, skip_reason in plan:
        advisory = key == "novel_types"
        if skip_reason is not None:
            results.append(GateResult(key, title, "skip", advisory, None, None, [],
                                       "", "", skip_reason, None))
            continue
        if parse_failed and key in CANDIDATE_READING:
            results.append(GateResult(key, title, "skip", advisory, None, None, command,
                                       "", "", "candidate does not parse", None))
            continue
        if fail_fast_stop is not None:
            reason = "not run (--fail-fast after %s failed)" % fail_fast_stop
            results.append(GateResult(key, title, "skip", advisory, None, None, command,
                                       "", "", reason, None))
            continue

        if key == "parse":
            result = run_parse_gate(paths, args.timeout)
        elif key in ("pyright", "novel_types") and key in type_keys:
            if not shared_attempted:
                shared_attempted = True
                shared_rows = run_shared_type_gates(
                    type_keys, paths, args,
                    {plan_key: plan_command for plan_key, _plan_title, plan_command, _reason
                     in plan if plan_key in type_keys},
                    analysis_metadata)
            if shared_rows is not None:
                result = shared_rows[key]
            else:
                result = run_script_gate(key, title, command, paths.root, args.timeout, advisory)
            if key == "novel_types" and shared_rows is None:
                result = classify_advisory(result, args)
        else:
            result = run_script_gate(key, title, command, paths.root, args.timeout, advisory)
            if key == "novel_types":
                result = classify_advisory(result, args)
        results.append(result)

        if key == "parse" and result.status == "fail":
            parse_failed = True
        if args.fail_fast and result.status in ("fail", "error") and fail_fast_stop is None:
            fail_fast_stop = key
    return results


# --- classification -----------------------------------------------------------------------
def steps_named_calls(steps_path):
    """Load check_step_calls via importlib and return the bare names from named_call_shapes().
    None when unreadable. Always loads the real sibling script next to this file -- not the
    (possibly stubbed) scripts_dir() -- because this is a cross-check against the real step
    list's parser, not a subprocess gate."""
    try:
        module_path = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                    "check_step_calls.py")
        spec = importlib.util.spec_from_file_location("check_step_calls", module_path)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        with open(steps_path, encoding="utf-8") as fh:
            steps_src = fh.read()
        shapes = module.named_call_shapes(steps_src)
        return {name for name, _has_receiver in shapes}
    except Exception:
        return None


def unresolved_api_names(stdout):
    return [m.group(1) for m in API_CALL_NAME_RE.finditer(stdout or "")]


def classify(results, paths):
    """One row per non-passing gate: {gate, fault, certain, why}. Implements the table in
    section 3.5 of the design. Pure function of the results plus the step list, so it is
    directly unit-testable."""
    rows = []
    calls_cache = None
    calls_loaded = False

    for r in results:
        if r.status not in ("fail", "error", "note"):
            continue

        if r.status == "error":
            rows.append({
                "gate": r.key, "fault": "setup", "certain": True,
                "why": ("the gate exited with a setup error (bad exit code, missing "
                        "script, or a timeout) -- fix the environment, not the draft"),
            })
            continue

        if r.key == "parse":
            rows.append({
                "gate": "parse", "fault": "emit", "certain": True,
                "why": ('a syntax error is drafter behavior (SKILL.md: "Syntax error, '
                        'undefined name, wrong submodule -> Emit fault")'),
            })
        elif r.key == "pyright":
            rows.append({
                "gate": "pyright", "fault": "emit", "certain": True,
                "why": ("a BLOCKING pyright finding is exactly an undefined name or "
                        "wrong adsk submodule"),
            })
        elif r.key == "input_read":
            rows.append({
                "gate": "input_read", "fault": "emit", "certain": True,
                "why": ("the reader helper is chosen by the drafter at the read site; "
                        "the step list never dictates it"),
            })
        elif r.key == "step_calls":
            rows.append({
                "gate": "step_calls", "fault": "emit", "certain": True,
                "why": ("a named call never made, or a stub left behind, is drafter "
                        "behavior"),
            })
        elif r.key == "anchors":
            rows.append({
                "gate": "anchors", "fault": "compile", "certain": True,
                "why": ("the gate reads only spec/**/*.md and the playbook, never the "
                        "candidate -- no draft can fix it; fix the anchor and recompile"),
            })
        elif r.key == "contract":
            rows.append({
                "gate": "contract", "fault": "judgment", "certain": False,
                "why": ("a missing class or ctx field can be a drafter omission (emit) "
                        "or a manifest that no longer matches the spec (compile); a "
                        "source_guards hit can point at a hand-written source file the "
                        "draft does not own"),
            })
        elif r.key == "api_calls":
            names = unresolved_api_names(r.stdout)
            if not calls_loaded:
                calls_cache = steps_named_calls(_abs(paths.root, paths.steps))
                calls_loaded = True
            if calls_cache is None:
                rows.append({
                    "gate": "api_calls", "fault": "emit", "certain": True,
                    "why": ("could not read the step list to cross-check -- treating "
                            "as an emit fault by default"),
                })
            else:
                hit = [n for n in names if n in calls_cache]
                if hit:
                    rows.append({
                        "gate": "api_calls", "fault": "judgment", "certain": False,
                        "why": (
                            "'%s' is named in spec/<gear>/steps.md, so the step list may "
                            "be naming a call that does not exist (compile fault); if the "
                            "step list does not name it, it is an emit fault" % hit[0]),
                    })
                else:
                    rows.append({
                        "gate": "api_calls", "fault": "emit", "certain": True,
                        "why": ("a method that exists nowhere, and the step list does "
                                "not name it, is drafter behavior"),
                    })
        elif r.key == "novel_types":
            rows.append({
                "gate": "novel_types", "fault": "judgment", "certain": False,
                "why": "advisory only -- read each finding and decide",
            })

    return rows


# --- rendering ------------------------------------------------------------------------------
def headline(result):
    """The gate's headline line: its first stdout line matching ^\\S.*check.*: -- otherwise
    its first non-empty stdout line."""
    if result.status == "skip":
        return result.skip_reason or ""
    lines = (result.stdout or "").splitlines()
    for line in lines:
        if HEADLINE_PATTERN.match(line):
            return line
    for line in lines:
        if line.strip():
            return line.strip()
    return ""


def compute_counts(results):
    counts = {"pass": 0, "fail": 0, "skip": 0, "error": 0, "advisory_findings": 0}
    for r in results:
        if r.status in counts:
            counts[r.status] += 1
    for r in results:
        if r.key == "novel_types" and r.status == "note":
            match = NOVEL_COUNT_RE.search(r.stdout or "")
            if match:
                counts["advisory_findings"] = int(match.group(1))
    return counts


def _timing_policy(args, results):
    """Ask the timing module to describe this runner's real gate policy."""
    try:
        module_path = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                   "pipeline_timing.py")
        spec = importlib.util.spec_from_file_location("run_gates_pipeline_timing", module_path)
        module = importlib.util.module_from_spec(spec)
        sys.modules[spec.name] = module
        spec.loader.exec_module(module)
        return module.gate_policy_for_run(args, results)
    except Exception as error:
        return {"kind": "run_gates", "available": False, "error": str(error)}


def timing_metadata(args, results, wall_time_s, analysis_metadata=None):
    """Return additive elapsed-time metadata while retaining every gate row unchanged."""
    gate_duration = round(sum(result.duration_s for result in results
                              if result.duration_s is not None), 6)
    timing = {
        "schema": 1,
        "wall_time_s": round(float(wall_time_s), 6),
        "gate_duration_s": gate_duration,
        "gate_policy": _timing_policy(args, results),
    }
    if analysis_metadata:
        for key in ("analysis_duration_s", "analysis_pyright_duration_s",
                    "analysis_invocations", "analysis_shared", "analysis_timing_note"):
            if key in analysis_metadata:
                timing[key] = analysis_metadata[key]
    policy = timing["gate_policy"]
    timing["first_pass_eligible"] = bool(policy.get("first_pass_eligible", False))
    return timing


def overall(results, args):
    """('setup_error', 2) if any status is 'error'; else ('fail', 1) if any status is 'fail';
    else ('pass', 0)."""
    if any(r.status == "error" for r in results):
        return "setup_error", 2
    if any(r.status == "fail" for r in results):
        return "fail", 1
    return "pass", 0


def render_text(results, classification, paths, args):
    lines = ["run_gates: %s  candidate=%s  root=%s" % (args.gear, paths.candidate, paths.root),
             ""]

    status_tag = {"pass": "PASS", "fail": "FAIL", "error": "ERROR", "note": "NOTE"}
    for r in results:
        if r.status == "skip":
            lines.append("  SKIP  %-11s %s" % (r.key, r.skip_reason))
        else:
            tag = status_tag[r.status]
            dur = "(%.2fs)" % r.duration_s if r.duration_s is not None else ""
            timing = " [%s]" % r.timing_note if r.timing_note else ""
            lines.append(("  %-5s %-11s %s%s  %s" %
                          (tag, r.key, dur, timing, headline(r))).rstrip())
        if r.status in ("fail", "error", "note"):
            for line in (r.stdout or "").splitlines():
                lines.append(" " * 8 + line)
            if r.stderr:
                lines.append(" " * 8 + "stderr:")
                for line in r.stderr.splitlines():
                    lines.append(" " * 8 + line)

    counts = compute_counts(results)
    verdict, _exit_code = overall(results, args)
    lines.append("")
    lines.append(
        "verdict: %s -- %d passed, %d failed, %d skipped, %d errored, %d advisory finding(s)"
        % (verdict.upper(), counts["pass"], counts["fail"], counts["skip"], counts["error"],
           counts["advisory_findings"]))

    if classification:
        lines.append("")
        lines.append('first-pass fault classification (SKILL.md "Telling an emit fault from '
                      'a compile fault"):')
        for row in classification:
            label = FAULT_LABEL.get(row["fault"], row["fault"])
            lines.append("  %-11s %-16s %s" % (row["gate"], label, row["why"]))
        lines.append("  " + NOT_DECIDED_NOTE)

    rerun = [r for r in results if r.status in ("fail", "error", "note")]
    if rerun:
        lines.append("")
        lines.append("re-run one gate by hand:")
        for r in rerun:
            lines.append("  " + " ".join(r.command))

    return "\n".join(lines)


def build_json(results, classification, paths, args, analysis_metadata=None, timing=None):
    verdict, exit_code = overall(results, args)
    counts = compute_counts(results)
    gates = []
    for r in results:
        gates.append({
            "key": r.key,
            "title": r.title,
            "status": r.status,
            "advisory": r.advisory,
            "exit_code": r.exit_code,
            "duration_s": r.duration_s,
            "command": r.command,
            "headline": headline(r),
            "stdout": r.stdout,
            "stderr": r.stderr,
            "skip_reason": r.skip_reason,
            "fault": r.fault,
            "timing_note": r.timing_note,
        })
    return {
        "schema": SCHEMA,
        "gear": args.gear,
        "candidate": paths.candidate,
        "root": paths.root,
        "verdict": verdict,
        "exit_code": exit_code,
        "counts": counts,
        "gates": gates,
        "classification": classification,
        "metadata": analysis_metadata or {},
        "timing": timing or {},
    }


def _setup_error_json(errors, paths, args):
    return {
        "schema": SCHEMA,
        "gear": args.gear,
        "candidate": paths.candidate,
        "root": paths.root,
        "verdict": "setup_error",
        "exit_code": 2,
        "counts": {"pass": 0, "fail": 0, "skip": 0, "error": 0, "advisory_findings": 0},
        "gates": [],
        "classification": [],
        "setup_errors": errors,
        "timing": {},
    }


def _write_json_out(path, obj):
    with open(path, "w", encoding="utf-8") as fh:
        json.dump(obj, fh, indent=2)
        fh.write("\n")


def main(argv=None):
    args = parse_args(sys.argv[1:] if argv is None else argv)
    paths = resolve_paths(args.gear, args.candidate, args.root)
    started = time.monotonic()

    errors = setup_errors(paths, args)
    if errors:
        obj = _setup_error_json(errors, paths, args)
        obj["timing"] = timing_metadata(args, [], time.monotonic() - started)
        line = JSON_MARKER + json.dumps(obj)
        if args.json_out:
            _write_json_out(args.json_out, obj)
        if args.format == "json":
            print(line)
        else:
            report = ["run_gates: %s  candidate=%s  root=%s"
                      % (args.gear, paths.candidate, paths.root), "", "setup error:"]
            report.extend("  - %s" % e for e in errors)
            print("\n".join(report))
            print(line)
        return 2

    plan = build_plan(paths, args)
    analysis_metadata = {}
    results = execute(plan, paths, args, analysis_metadata)
    classification = classify(results, paths)
    fault_by_gate = {row["gate"]: row["fault"] for row in classification}
    results = [dataclasses.replace(r, fault=fault_by_gate.get(r.key, r.fault))
               for r in results]

    timing = timing_metadata(args, results, time.monotonic() - started, analysis_metadata)
    obj = build_json(results, classification, paths, args, analysis_metadata, timing)
    line = JSON_MARKER + json.dumps(obj)
    if args.json_out:
        _write_json_out(args.json_out, obj)
    if args.format == "json":
        print(line)
    else:
        print(render_text(results, classification, paths, args))
        print(line)
    return obj["exit_code"]


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
