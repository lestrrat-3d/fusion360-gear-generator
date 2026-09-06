#!/usr/bin/env python3
"""Run the whole compile-gear gate battery for one gear and print one verdict.

Why this exists: `/compile-gear` (`.claude/skills/compile-gear/SKILL.md`) steps 4-5 used to ask
the orchestrating LLM to run three commands by hand, in a fixed order, with one conditional —
the proof, then `check_compile.py`, then `check_step_calls.py` but only when
`lib/geargen/<gear>.py` already exists. Every part of that is mechanical: the commands are
fixed, their arguments are derived from `<gear>`, their exit codes already say pass/fail, and
the conditional is a file-existence test. Leaving it to the model meant a gate silently dropped
under context pressure, argument order improvised (`check_step_calls.py` takes the step list
first), and a retry loop that saw one failure at a time instead of all of them. This script runs
the battery once, in a fixed order, and reports every result plus a first-pass fault
classification.

It is the compile-stage counterpart of the emit-stage `run_gates.py`, and it is deliberately
standalone: it imports nothing from that script.

What it does not do: it never copies, moves, or indexes a file. The proof is expected to be in
`proof/<gear>/` already — `stage.py <gear> proof` puts it there — because a runner that also
placed artifacts would hide a failed placement behind a failed gate. The one file it writes is
the `playbook` stage's scratch extract under `.tmp/`, which is the extractor's own default
output path and is read by nothing this stage does.

The `playbook` stage is here because a step list that cites no `[PB-…]` anchor used to pass
every compile gate. `check_compile.py` gates the spec lines a step cites, not the playbook rules
it leans on, so a citation-free step list looked clean at compile time and handed the emit
drafter an extract holding the core sections and nothing else. This runner calls
`extract_playbook.py --min-anchors 1`, which turns that into a compile-time failure.

Usage:
    run_compile_gates.py <gear> [options]

positional:
  gear                   gear name, e.g. spurgear. Names spec/<gear>/steps.md, proof/<gear>/
                         and lib/geargen/<gear>.py.

options:
  --root PATH            repo root. Default: three levels above this script.
  --only KEY[,KEY...]    run only these stages. Keys: proof, compile, playbook, step_calls.
                         Unselected stages are reported with status "skip",
                         reason "not selected". An unknown key is a usage error.
  --fail-fast            stop scheduling stages after the first failure. Off by default, so
                         one run shows the drafter every failure at once.
  --format {text,json}   text (default) = human report followed by one JSON line.
                         json = the JSON line only, nothing else on stdout.
  --json-out PATH        also write the full JSON verdict (pretty-printed) to PATH.
  --timeout SECONDS      per-stage timeout. Default 900. A stage that exceeds it gets
                         status "error".

Exit codes:
    0  every stage that ran passed. Skips are allowed.
    1  at least one stage failed on content. Classify with the printed table: a draft fault
       goes back to the drafter with the report text appended; a prose fault stops the run.
    2  setup error: bad usage, a failed preflight check, a stage exited 2, or a stage timed
       out. Fix the environment or the inputs; never retry the drafter.
"""
import argparse
import dataclasses
import hashlib
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
STAGE_ORDER = ("proof", "compile", "playbook", "step_calls")
JSON_MARKER = "COMPILE_GATES_JSON: "

# The floor the `playbook` stage holds the step list to. One is deliberately the lowest number
# that still refuses the failure this stage was added for: a step list citing nothing, whose
# extract is the 8% of the playbook the core sections alone make up. A higher floor would be a
# guess about how many rules a gear ought to need, and the three compiled gears cite 25, 9 and 39
# playbook-defined anchors, so nothing near a real step list is being defended here.
MIN_PLAYBOOK_ANCHORS = 1

# The same gear-name rule stage.py and the other per-gear scripts use.
GEAR_NAME = re.compile(r'[a-z][a-z0-9_]*\Z')

STAGE_TITLES = {
    "proof": "Proof",
    "compile": "Compile check",
    "playbook": "Playbook extract",
    "step_calls": "Step calls",
}

STAGE_SCRIPTS = {
    "compile": "check_compile.py",
    "playbook": "extract_playbook.py",
    "step_calls": "check_step_calls.py",
}

PROOF_RUNNER = os.path.join("proof", "run.sh")

# The headline a stage's report line shows: `compile check: OK (...)`,
# `step-call check: BLOCKING (2)`, and the like.
HEADLINE_PATTERN = re.compile(r'^\S.*check.*:')

# check_compile's own wording for the three compile problems this runner can tell apart.
API_CALL_NAME_RE = re.compile(r"names '(\w+)\(', which the Fusion API database does not have")
PROVENANCE_DRIFT_RE = re.compile(r'has changed since the step list was compiled')

# extract_playbook's own wording for the too-few-anchors defect, as opposed to its other exit-1.
PLAYBOOK_UNCITED_RE = re.compile(r'playbook anchor\(s\); at least \d+ is required')

# The fault sentences, one per row of SKILL.md's "Telling a draft fault from a prose fault".
FAULT_PROOF = 'DRAFT FAULT by default ("The proof fails to build")'
FAULT_DRAFT = "DRAFT FAULT"
FAULT_DRIFT = "DRAFT FAULT, or someone edited the spec mid-run"
FAULT_JUDGMENT = ("NEEDS JUDGMENT: a call the spec itself names is a PROSE FAULT; one the "
                  "drafter invented is a DRAFT FAULT")
FAULT_STEP_CALLS = ("NEEDS CLASSIFICATION: read the output as call names and pass only the "
                    "names back to the drafter (SKILL.md step 5)")
FAULT_PLAYBOOK_UNCITED = ("DRAFT FAULT: the step list cites no playbook rule, so the emit "
                          "drafter would be handed the core sections and nothing else")
FAULT_PLAYBOOK_UNDEFINED = ("DRAFT FAULT, or someone edited the playbook: the step list cites "
                            "a [PB-...] anchor the playbook does not define")
FAULT_SETUP = "SETUP ERROR: fix the environment or inputs; never retry the drafter"

# Why a missing module is a skip rather than a failure. The wording is fixed because the
# drafter is told the same thing in SKILL.md step 5.
MODULE_ABSENT_REASON = ("lib/geargen/%s.py does not exist; the CI cross-check applies only "
                        "after /emit-gear places a module")


# --- small value types --------------------------------------------------------------------
@dataclass
class StageResult:
    key: str
    title: str
    status: str
    exit_code: int | None
    duration_s: float | None
    command: list[str]
    stdout: str
    stderr: str
    skip_reason: str | None
    fault: str | None
    skip_reason_code: str | None = None
    checker_json: dict | None = None
    policy_status: str | None = None
    disposition: str | None = None


@dataclass
class Paths:
    root: str          # absolute
    gear: str
    steps: str         # root-relative "spec/<gear>/steps.md"
    proof_dir: str     # root-relative "proof/<gear>"
    module: str        # root-relative "lib/geargen/<gear>.py"


# --- path and argument plumbing -------------------------------------------------------------
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
        prog="run_compile_gates.py",
        description="Run the compile-gear gate battery for one gear and print one verdict.")
    p.add_argument("gear")
    p.add_argument("--root", default=None)
    p.add_argument("--only", default=None,
                   help="comma-separated stage keys to run; the rest are skipped")
    p.add_argument("--fail-fast", action="store_true")
    p.add_argument("--json-out", default=None)
    p.add_argument("--format", choices=["text", "json"], default="text")
    p.add_argument("--timeout", type=float, default=DEFAULT_TIMEOUT)
    p.add_argument("--iteration-base", default=None,
                   help="run a focused retry based on changes after COMMIT")
    p.add_argument("--handoff-base", default=None,
                   help="immutable commit used to classify compile-to-emit changes")
    p.add_argument("--step-call-review", default=None,
                   help="reviewing-agent JSON record for missing step calls")
    args = p.parse_args(argv)
    if args.only:
        args.only = [k.strip() for k in args.only.split(",") if k.strip()]
    return args


def resolve_paths(gear, root_arg):
    """Absolutise the root and express every input root-relative, so a printed command is
    copy-pasteable from the repo root."""
    root = os.path.abspath(root_arg) if root_arg else repo_root()
    return Paths(root=root,
                 gear=gear,
                 steps=os.path.join("spec", gear, "steps.md"),
                 proof_dir=os.path.join("proof", gear),
                 module=os.path.join("lib", "geargen", "%s.py" % gear))


def resolve_iteration_base(root, base):
    """Resolve an iteration base to one full commit before any gate starts."""
    if not base:
        return None, "iteration base must be a non-empty commit or ref"
    command = ["git", "-C", root, "rev-parse", "--verify", "--end-of-options",
               "%s^{commit}" % base]
    try:
        proc = subprocess.run(command, capture_output=True, text=True)
    except OSError as exc:
        return None, "cannot resolve iteration base %r: %s" % (base, exc)
    resolved = proc.stdout.strip()
    if proc.returncode != 0 or not re.fullmatch(r"[0-9a-f]{40}", resolved):
        detail = proc.stderr.strip() or "unknown commit or ref"
        return None, "cannot resolve iteration base %r: %s" % (base, detail)
    return resolved, None


def sha256_bytes(value):
    return hashlib.sha256(value).hexdigest()


def _load_checker_module():
    path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "check_step_calls.py")
    spec = importlib.util.spec_from_file_location("compile_step_call_shapes", path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def _read_bytes(path):
    with open(path, "rb") as handle:
        return handle.read()


def _git_show(root, commit, path):
    proc = subprocess.run(["git", "-C", root, "show", "%s:%s" % (commit, path)],
                          capture_output=True)
    if proc.returncode:
        return None
    return proc.stdout


def _canonical(value):
    return json.dumps(value, sort_keys=True, separators=(",", ":"), ensure_ascii=True).encode()


def _review_requirements(missing, base_steps):
    old_shapes = _load_checker_module().named_call_shapes(base_steps.decode("utf-8")) \
        if base_steps is not None else set()
    old = {(name, bool(receiver)) for name, receiver in old_shapes}
    return [dict(record, origin=("baseline_unknown" if base_steps is None else
                                 "new_since_base" if (record["name"], record["has_receiver"]) not in old
                                 else "already_required"))
            for record in missing]


def _review_input_hash(root, steps_path, review_requirements):
    evidence = []
    for requirement in review_requirements:
        for item in requirement.get("evidence", ()):
            path = item.get("path")
            if not isinstance(path, str):
                continue
            absolute = os.path.join(root, path)
            try:
                evidence.append({"path": path, "line": item.get("line"),
                                 "sha256": item.get("sha256"),
                                 "bytes_sha256": sha256_bytes(_read_bytes(absolute))})
            except OSError:
                evidence.append({"path": path, "line": item.get("line"),
                                 "sha256": item.get("sha256"), "bytes_sha256": None})
    payload = {"steps": _read_bytes(os.path.join(root, steps_path)).decode("utf-8"),
               "evidence": sorted(evidence, key=lambda item: (item["path"], item["line"] or 0))}
    return sha256_bytes(_canonical(payload))


def _validate_review(path, gear, base, paths, requirements):
    if not path or not base:
        return None, "review record is required"
    try:
        with open(path, encoding="utf-8") as handle:
            review = json.load(handle)
    except (OSError, ValueError) as error:
        return None, "malformed step-call review: %s" % error
    if not isinstance(review, dict) or review.get("schema") != 1 or review.get("gear") != gear:
        return None, "malformed step-call review: schema and gear are required"
    binding = review.get("binding")
    records = review.get("requirements")
    if not isinstance(binding, dict) or not isinstance(records, list):
        return None, "malformed step-call review: binding and requirements are required"
    try:
        steps_bytes = _read_bytes(os.path.join(paths.root, paths.steps))
        module_bytes = _read_bytes(os.path.join(paths.root, paths.module))
    except OSError as error:
        return None, "cannot bind step-call review: %s" % error
    expected = {"comparison_base": base,
                "steps_sha256": sha256_bytes(steps_bytes),
                "module_sha256": sha256_bytes(module_bytes),
                "requirements_sha256": sha256_bytes(_canonical(requirements)),
                "review_inputs_sha256": _review_input_hash(paths.root, paths.steps, records)}
    if any(binding.get(key) != value for key, value in expected.items()):
        return None, "stale step-call review binding"
    wanted = {(item["name"], bool(item["has_receiver"])) for item in requirements}
    seen = set()
    for item in records:
        if (not isinstance(item, dict) or not isinstance(item.get("name"), str) or
                not isinstance(item.get("has_receiver"), bool) or
                (item.get("name"), item.get("has_receiver")) not in wanted):
            return None, "step-call review has an unknown requirement"
        key = (item["name"], bool(item["has_receiver"]))
        if key in seen or item.get("decision") not in ("emit_required", "draft_fault"):
            return None, "step-call review has duplicate or invalid decisions"
        seen.add(key)
        if not isinstance(item.get("reason"), str) or not item["reason"].strip():
            return None, "step-call review reason must be non-empty"
        evidence = item.get("evidence")
        if not isinstance(evidence, list) or not evidence:
            return None, "step-call review evidence is required"
        for ev in evidence:
            if not isinstance(ev, dict) or not isinstance(ev.get("path"), str):
                return None, "step-call review evidence is malformed"
            source = os.path.join(paths.root, ev["path"])
            if os.path.commonpath((os.path.realpath(paths.root), os.path.realpath(source))) != \
                    os.path.realpath(paths.root):
                return None, "step-call review evidence path is outside repository"
            try:
                source_bytes = _read_bytes(source)
                line_count = source_bytes.count(b"\n") + (1 if source_bytes else 0)
            except OSError:
                return None, "step-call review evidence path is unreadable"
            line = ev.get("line")
            if not isinstance(line, int) or isinstance(line, bool) or line < 1 or line > line_count:
                return None, "step-call review evidence line is out of range"
            if ev.get("sha256") != sha256_bytes(source_bytes):
                return None, "step-call review evidence is stale"
    if seen != wanted:
        return None, "step-call review must decide every missing requirement"
    return records, None


def _validate_checker_payload(payload):
    if not isinstance(payload, dict) or not isinstance(payload.get("ok"), bool):
        return "checker JSON must be an object with boolean ok"
    if (not isinstance(payload.get("named_calls"), int) or isinstance(payload["named_calls"], bool)
            or payload["named_calls"] < 0):
        return "checker JSON named_calls must be an integer"
    for key in ("missing", "stubs", "shared_point"):
        if not isinstance(payload.get(key), list):
            return "checker JSON %s must be a list" % key
    if payload.get("parse_error") is not None and not isinstance(payload.get("parse_error"), str):
        return "checker JSON parse_error must be a string or null"
    for item in payload["missing"]:
        if (not isinstance(item, dict) or not isinstance(item.get("name"), str) or
                not isinstance(item.get("has_receiver"), bool) or
                not isinstance(item.get("textual_match"), bool)):
            return "checker JSON missing records have invalid types"
    for key in ("stubs", "shared_point"):
        if not all(isinstance(item, dict) for item in payload[key]):
            return "checker JSON %s records have invalid types" % key
    return None


def _parse_name_status_z(payload):
    """Return paths named by `git diff --name-status -z`, including rename endpoints."""
    fields = payload.split(b"\0")
    if fields and fields[-1] == b"":
        fields.pop()
    paths = []
    index = 0
    while index < len(fields):
        status = fields[index]
        index += 1
        if not status:
            continue
        if status[:1] in (b"R", b"C"):
            if index + 1 >= len(fields):
                return None, "incomplete rename record from git"
            paths.extend((os.fsdecode(fields[index]), os.fsdecode(fields[index + 1])))
            index += 2
        else:
            if index >= len(fields):
                return None, "incomplete path record from git"
            paths.append(os.fsdecode(fields[index]))
            index += 1
    return paths, None


def _git_name_status(root, args):
    command = ["git", "-C", root] + list(args)
    try:
        proc = subprocess.run(command, capture_output=True)
    except OSError as exc:
        return [], "cannot inspect Git changes: %s" % exc
    if proc.returncode != 0:
        detail = os.fsdecode(proc.stderr).strip() or "git command failed"
        return [], "cannot inspect Git changes: %s" % detail
    return _parse_name_status_z(proc.stdout)


def changed_paths(root, base):
    """Collect committed, staged, unstaged, and relevant untracked paths via NUL output."""
    all_paths = []
    commands = [
        ["diff", "--name-status", "--find-renames", "-z", base, "HEAD", "--"],
        ["diff", "--name-status", "--find-renames", "-z", "HEAD", "--"],
        ["diff", "--cached", "--name-status", "--find-renames", "-z", "--"],
    ]
    for command_args in commands:
        paths, error = _git_name_status(root, command_args)
        if error:
            return None, error
        all_paths.extend(paths)

    command = ["git", "-C", root, "ls-files", "--others", "--exclude-standard", "-z", "--"]
    try:
        proc = subprocess.run(command, capture_output=True)
    except OSError as exc:
        return None, "cannot inspect untracked Git files: %s" % exc
    if proc.returncode != 0:
        detail = os.fsdecode(proc.stderr).strip() or "git command failed"
        return None, "cannot inspect untracked Git files: %s" % detail
    fields = proc.stdout.split(b"\0")
    if fields and fields[-1] == b"":
        fields.pop()
    all_paths.extend(os.fsdecode(field) for field in fields if field)
    return set(all_paths), None


def iteration_scope(root, gear, base):
    """Return proof scope metadata; unknown Git state deliberately expands to full."""
    paths, error = changed_paths(root, base)
    if error:
        return {
            "effective_proof_scope": "expanded-full",
            "selected_packages": [],
            "full_suite_reason": "git_state_unknown",
            "full_suite_detail": error,
        }
    allowed = ("spec/%s/" % gear, "proof/%s/" % gear)
    outside = sorted(path for path in paths if not path.startswith(allowed))
    if outside:
        return {
            "effective_proof_scope": "expanded-full",
            "selected_packages": [],
            "full_suite_reason": "shared_or_other_path_changed",
            "full_suite_detail": "changed paths outside the gear inputs: %s" % ", ".join(outside),
        }
    if not paths:
        return {
            "effective_proof_scope": "expanded-full",
            "selected_packages": [],
            "full_suite_reason": "no_relevant_changes",
            "full_suite_detail": "no non-ignored paths changed after the supplied base",
        }
    package = "./%s" % gear
    return {
        "effective_proof_scope": "selected",
        "selected_packages": [package],
        "full_suite_reason": None,
        "full_suite_detail": None,
    }


def active_keys(args):
    """The stages --only selects, in STAGE_ORDER."""
    if args.only:
        return [k for k in STAGE_ORDER if k in args.only]
    return list(STAGE_ORDER)


def module_exists(paths):
    return os.path.isfile(_abs(paths.root, paths.module))


def _proof_go_files(proof_abs):
    try:
        return [e for e in sorted(os.listdir(proof_abs)) if e.endswith(".go")]
    except OSError:
        return []


def setup_errors(paths, args):
    """Preflight, returning human messages. Every one of them is exit 2 and stops the run
    before a stage is started.

    check_step_calls.py raises an uncaught IOError on a missing input, so both of its paths are
    verified here rather than left to produce a traceback; the same care costs nothing for the
    other stages. Only the checks a selected stage actually needs are made, so `--only compile`
    does not demand a proof runner.
    """
    errors = []
    if not GEAR_NAME.match(args.gear):
        errors.append("gear name %r does not match [a-z][a-z0-9_]*" % args.gear)
        return errors

    if not os.path.isdir(paths.root):
        errors.append("repo root is not a directory: %s" % paths.root)
        return errors

    if args.only:
        unknown = sorted(set(args.only) - set(STAGE_ORDER))
        if unknown:
            errors.append("unknown --only key(s): %s (known: %s)"
                          % (", ".join(unknown), ", ".join(STAGE_ORDER)))
            return errors
        if not active_keys(args):
            errors.append("--only selected no stage")
            return errors

    keys = set(active_keys(args))

    # The playbook stage needs `PLAYBOOK.md` as well, and it is not checked here: the extractor
    # resolves that path itself and exits 2 on an unreadable one, which this runner already
    # reports as a setup error. Repeating the test would only let the two disagree.
    if keys & {"compile", "playbook", "step_calls"}:
        if not os.path.isfile(_abs(paths.root, paths.steps)):
            errors.append("step list not found: %s -- draft it before checking it" % paths.steps)

    if keys & {"proof", "compile"}:
        proof_abs = _abs(paths.root, paths.proof_dir)
        if not os.path.isdir(proof_abs):
            errors.append(
                "proof directory not found: %s -- the drafted proof was never placed there; "
                "run stage.py %s proof first" % (paths.proof_dir, args.gear))
        elif not _proof_go_files(proof_abs):
            errors.append("proof directory %s holds no .go file -- the drafted proof was never "
                          "placed there; run stage.py %s proof first"
                          % (paths.proof_dir, args.gear))

    if "proof" in keys and not os.path.isfile(_abs(paths.root, PROOF_RUNNER)):
        errors.append("proof runner not found: %s" % PROOF_RUNNER)

    for key in sorted(keys):
        if key == "step_calls" and not module_exists(paths):
            continue  # the stage skips, so its script is never needed
        script_name = STAGE_SCRIPTS.get(key)
        if script_name and not os.path.isfile(os.path.join(scripts_dir(), script_name)):
            errors.append("gate script missing: %s (needed for '%s')" % (script_name, key))

    return errors


def iteration_setup_errors(paths, args):
    """Check only inputs needed before the cheap iteration stages can begin."""
    errors = []
    if not GEAR_NAME.match(args.gear):
        errors.append("gear name %r does not match [a-z][a-z0-9_]*" % args.gear)
        return errors
    if not os.path.isdir(paths.root):
        errors.append("repo root is not a directory: %s" % paths.root)
        return errors
    if args.only:
        errors.append("--only cannot be combined with --iteration-base")
        return errors
    if not os.path.isfile(_abs(paths.root, paths.steps)):
        errors.append("step list not found: %s -- draft it before checking" % paths.steps)
    for key in ("compile", "playbook"):
        script_name = STAGE_SCRIPTS[key]
        if not os.path.isfile(os.path.join(scripts_dir(), script_name)):
            errors.append("gate script missing: %s (needed for '%s')" % (script_name, key))
    return errors


def iteration_proof_setup_errors(paths):
    """Return proof setup errors after cheap checks have passed."""
    errors = []
    proof_abs = _abs(paths.root, paths.proof_dir)
    if not os.path.isdir(proof_abs):
        errors.append("proof directory not found: %s" % paths.proof_dir)
    elif not _proof_go_files(proof_abs):
        errors.append("proof directory %s holds no .go file" % paths.proof_dir)
    if not os.path.isfile(_abs(paths.root, PROOF_RUNNER)):
        errors.append("proof runner not found: %s" % PROOF_RUNNER)
    return errors


# --- stage plan ------------------------------------------------------------------------------
def _script_path(name, paths):
    return _root_relative(os.path.join(scripts_dir(), name), paths.root)


def stage_command(key, paths):
    """The one place that knows each stage's argument order. `check_step_calls.py` takes the
    step list FIRST and the module second, which is the order a hand-run got wrong."""
    if key == "proof":
        return ["bash", PROOF_RUNNER]
    if key == "compile":
        return [sys.executable, _script_path("check_compile.py", paths), paths.gear]
    if key == "playbook":
        return [sys.executable, _script_path("extract_playbook.py", paths), paths.gear,
                "--min-anchors", str(MIN_PLAYBOOK_ANCHORS)]
    if key == "step_calls":
        return [sys.executable, _script_path("check_step_calls.py", paths),
                paths.steps, paths.module]
    raise ValueError("unknown stage key: %s" % key)


def build_plan(paths, args):
    """(key, title, command, skip_reason), in STAGE_ORDER. Command is None exactly when
    skip_reason is set. Applies --only and the missing-module skip."""
    active = set(active_keys(args))
    plan = []
    for key in STAGE_ORDER:
        title = STAGE_TITLES[key]
        if key not in active:
            plan.append((key, title, None, "not selected"))
            continue
        if key == "step_calls" and not module_exists(paths):
            plan.append((key, title, None, MODULE_ABSENT_REASON % args.gear))
            continue
        plan.append((key, title, stage_command(key, paths), None))
    return plan


def build_iteration_plan(paths, args, scope):
    """Build cheap-first commands; proof is appended only after their results are known."""
    plan = []
    for key in ("compile", "playbook", "step_calls"):
        title = STAGE_TITLES[key]
        if key == "step_calls" and not module_exists(paths):
            plan.append((key, title, None, MODULE_ABSENT_REASON % args.gear))
            continue
        plan.append((key, title, stage_command(key, paths), None))
    proof_args = [] if scope["effective_proof_scope"] == "expanded-full" else [
        "--package", scope["selected_packages"][0]]
    proof_command = ["bash", PROOF_RUNNER] + proof_args
    plan.append(("proof", STAGE_TITLES["proof"], proof_command, None))
    return plan


# --- execution --------------------------------------------------------------------------------
def status_for(key, returncode):
    """Exit code to status, per stage.

    `proof/run.sh` exits 2 for a setup problem — a missing sketch or decad checkout, a revision
    mismatch — and otherwise propagates `go test`, so anything else nonzero is a red proof. The
    two Python checkers use the repo-wide 0/1/2 = OK/BLOCKING/bad-input contract.
    """
    if returncode == 0:
        return "pass"
    if returncode == 2:
        return "error"
    return "fail"


def run_stage(key, title, command, cwd, timeout):
    """Run one stage as a subprocess, capturing stdout and stderr and timing it."""
    start = time.monotonic()
    try:
        effective_command = list(command)
        checker_path = os.path.realpath(os.path.join(scripts_dir(), "check_step_calls.py"))
        checker_run = key == "step_calls" and len(command) > 1 and \
            os.path.realpath(command[1]) == checker_path
        if checker_run:
            effective_command.append("--json")
        proc = subprocess.run(effective_command, cwd=cwd, capture_output=True, text=True,
                              timeout=timeout, env=os.environ)
    except subprocess.TimeoutExpired as exc:
        duration = round(time.monotonic() - start, 2)
        stdout = exc.stdout or ""
        if isinstance(stdout, bytes):
            stdout = stdout.decode("utf-8", "replace")
        stderr = exc.stderr or ""
        if isinstance(stderr, bytes):
            stderr = stderr.decode("utf-8", "replace")
        stderr += "\nstage timed out after %ss" % timeout
        return StageResult(key, title, "error", None, duration, command, stdout, stderr,
                           None, None)
    duration = round(time.monotonic() - start, 2)
    result = StageResult(key, title, status_for(key, proc.returncode), proc.returncode, duration,
                         command, proc.stdout, proc.stderr, None, None)
    if key == "step_calls":
        checker_path = os.path.realpath(os.path.join(scripts_dir(), "check_step_calls.py"))
        if not (len(command) > 1 and os.path.realpath(command[1]) == checker_path):
            return result
        try:
            payload = json.loads(proc.stdout)
            payload_error = _validate_checker_payload(payload)
            if payload_error:
                raise ValueError(payload_error)
            if payload["ok"] != (proc.returncode == 0):
                raise ValueError("checker JSON ok does not match exit status")
            result.checker_json = payload
        except (ValueError, json.JSONDecodeError) as error:
            result.status = "error"
            result.exit_code = 2
            result.stderr = (result.stderr + "\n" if result.stderr else "") + str(error)
    return result


def execute(plan, paths, args):
    """Walk the plan in order. With --fail-fast, every stage after the first failure becomes a
    skip naming the stage that stopped the run."""
    results = []
    stopped_by = None
    for key, title, command, skip_reason in plan:
        if skip_reason is not None:
            results.append(StageResult(key, title, "skip", None, None, [], "", "",
                                       skip_reason, None))
            continue
        if stopped_by is not None:
            reason = "not run (--fail-fast after %s failed)" % stopped_by
            results.append(StageResult(key, title, "skip", None, None, command, "", "",
                                       reason, None))
            continue
        result = run_stage(key, title, command, paths.root, args.timeout)
        results.append(result)
        if args.fail_fast and result.status in ("fail", "error") and stopped_by is None:
            stopped_by = key
    return results


def handoff_for(results, paths, args):
    """Classify the step-call disagreement without changing the checker contract."""
    module = paths.module
    step_result = next((item for item in results if item.key == "step_calls"), None)
    if step_result is None:
        return {"status": "not_applicable", "module": module, "requirements": []}
    if step_result.status == "skip":
        return {"status": "not_applicable" if "does not exist" in (step_result.skip_reason or "")
                else "blocked", "module": module, "requirements": []}
    payload = step_result.checker_json
    if payload is None:
        return {"status": "blocked", "module": module, "requirements": []}
    missing = payload.get("missing", [])
    if not isinstance(missing, list):
        return {"status": "blocked", "module": module, "requirements": []}
    base = getattr(args, "_handoff_base", None)
    base_steps = _git_show(paths.root, base, paths.steps) if base else None
    if payload.get("parse_error") or payload.get("stubs") or payload.get("shared_point"):
        return {"status": "blocked", "module": module, "requirements": []}
    requirements = _review_requirements(missing, base_steps)
    if not requirements:
        return {"status": "ready", "module": module, "requirements": []}
    records, error = _validate_review(getattr(args, "step_call_review", None), args.gear,
                                      base, paths, requirements)
    if error:
        return {"status": "review_required", "module": module, "requirements": requirements,
                "review_error": error,
                "review_binding": {"comparison_base": base,
                                    "steps_sha256": sha256_bytes(_read_bytes(os.path.join(paths.root, paths.steps))),
                                    "module_sha256": sha256_bytes(_read_bytes(os.path.join(paths.root, paths.module))),
                                    "requirements_sha256": sha256_bytes(_canonical(requirements)),
                                    "review_inputs_sha256": None}}
    for requirement in requirements:
        review = next(item for item in records if item["name"] == requirement["name"] and
                      bool(item["has_receiver"]) == bool(requirement["has_receiver"]))
        requirement["review"] = review["decision"]
    if any(item["review"] == "draft_fault" for item in requirements):
        return {"status": "blocked", "module": module, "requirements": requirements}
    return {"status": "emit_required", "module": module, "requirements": requirements}


def apply_handoff_policy(results, handoff):
    if handoff.get("status") != "emit_required":
        return results
    for index, result in enumerate(results):
        if result.key == "step_calls":
            results[index] = dataclasses.replace(result, status="pass", policy_status="pass",
                                                  disposition="emit_required")
        elif result.key == "compile":
            results[index] = dataclasses.replace(result, policy_status="pass",
                                                  disposition="emit_required")
    return results


def _iteration_failure_code(result):
    if result.status == "error":
        return "%s_setup_failure" % result.key
    return "%s_content_failure" % result.key


def execute_iteration(paths, args, scope):
    """Run compile/playbook first, retain step-call output, and gate proof execution."""
    preliminary = build_iteration_plan(paths, args, scope)[:3]
    results = execute(preliminary, paths, args)
    cheap_failures = [r for r in results if r.key in ("compile", "playbook") and
                      r.status in ("fail", "error")]
    failures = [r for r in results if r.status in ("fail", "error")]
    if cheap_failures or (args.fail_fast and failures):
        codes = [_iteration_failure_code(result) for result in
                 (cheap_failures if cheap_failures else failures)]
        reason = "proof_omitted_after_%s" % "_and_".join(codes)
        results.append(StageResult(
            "proof", STAGE_TITLES["proof"], "skip", None, None,
            build_iteration_plan(paths, args, scope)[-1][2], "", "",
            "proof omitted: %s" % ", ".join(codes), None, reason))
        return results, reason, codes

    proof_errors = iteration_proof_setup_errors(paths)
    proof_command = build_iteration_plan(paths, args, scope)[-1][2]
    if proof_errors:
        result = StageResult("proof", STAGE_TITLES["proof"], "error", 2, None,
                             proof_command, "", "\n".join(proof_errors), None, None,
                             "proof_setup_failure")
        results.append(result)
        return results, None, []

    results.append(run_stage("proof", STAGE_TITLES["proof"], proof_command,
                             paths.root, args.timeout))
    return results, None, []


# --- classification -----------------------------------------------------------------------
def unresolved_api_names(stdout):
    """The calls check_compile says the API database does not have."""
    return [m.group(1) for m in API_CALL_NAME_RE.finditer(stdout or "")]


def classify_compile(stdout):
    """Which SKILL.md row a BLOCKING compile check falls under.

    Precedence is by how much of a decision the row needs. A named call the database does not
    have is the only compile problem this runner cannot settle — the same failure is a prose
    fault when the spec named the call and a draft fault when the drafter invented it — so it
    wins over the rows that are always the drafter's. Which of the two it is stays unanswered
    here on purpose: `check_compile.py` is being taught to annotate it mechanically, and a
    second guess made by grepping the spec from this side would only disagree with that one.
    """
    names = unresolved_api_names(stdout)
    if names:
        return "%s (calls: %s)" % (FAULT_JUDGMENT, ", ".join(sorted(set(names))))
    if PROVENANCE_DRIFT_RE.search(stdout or ""):
        return FAULT_DRIFT
    return FAULT_DRAFT


def classify_playbook(stderr):
    """Which of `extract_playbook.py`'s two exit-1 defects a failed playbook stage hit.

    Both are the drafter's, so the distinction is only about what to tell it. The
    too-few-anchors message is matched on the phrase the extractor prints; anything else
    exit 1 can mean is the undefined-anchor defect, which is also what a playbook edit made
    after the step list was drafted looks like from here.
    """
    if PLAYBOOK_UNCITED_RE.search(stderr or ""):
        return FAULT_PLAYBOOK_UNCITED
    return FAULT_PLAYBOOK_UNDEFINED


def classify(results):
    """One fault sentence per stage that did not pass, mirroring SKILL.md's "Telling a draft
    fault from a prose fault" table. Pure function of the results, so it is directly testable."""
    faults = {}
    for r in results:
        if r.status == "error":
            faults[r.key] = FAULT_SETUP
        elif r.status == "fail":
            if r.key == "proof":
                faults[r.key] = FAULT_PROOF
            elif r.key == "compile":
                faults[r.key] = classify_compile(r.stdout)
            elif r.key == "playbook":
                faults[r.key] = classify_playbook(r.stderr)
            elif r.key == "step_calls":
                faults[r.key] = FAULT_STEP_CALLS
    return faults


# --- rendering ------------------------------------------------------------------------------
def headline(result):
    """The stage's headline: its first stdout line matching ^\\S.*check.*: -- otherwise its
    first non-empty stdout line."""
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
    counts = {"pass": 0, "fail": 0, "skip": 0, "error": 0}
    for r in results:
        if r.status in counts:
            counts[r.status] += 1
    return counts


def _timing_policy(args, results, metadata):
    """Ask the timing module to describe this runner's proof policy."""
    try:
        module_path = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                   "pipeline_timing.py")
        spec = importlib.util.spec_from_file_location("run_compile_pipeline_timing", module_path)
        module = importlib.util.module_from_spec(spec)
        sys.modules[spec.name] = module
        spec.loader.exec_module(module)
        return module.compile_policy_for_run(args, results, metadata)
    except Exception as error:
        return {"kind": "run_compile_gates", "available": False, "error": str(error)}


def timing_metadata(args, results, metadata, wall_time_s):
    """Return additive elapsed-time metadata while retaining the stage report unchanged."""
    stage_duration = round(sum(result.duration_s for result in results
                               if result.duration_s is not None), 6)
    policy = _timing_policy(args, results, metadata)
    return {
        "schema": 1,
        "wall_time_s": round(float(wall_time_s), 6),
        "stage_duration_s": stage_duration,
        "gate_policy": policy,
        "first_pass_eligible": bool(policy.get("first_pass_eligible", False)),
    }


def overall(results):
    """('error', 2) if any stage errored; else ('fail', 1) if any failed; else ('pass', 0)."""
    if any(r.status == "error" for r in results):
        return "error", 2
    if any(r.status == "fail" for r in results):
        return "fail", 1
    return "pass", 0


def render_text(results, paths, args, handoff=None):
    lines = ["run_compile_gates: %s  root=%s" % (args.gear, paths.root), ""]
    metadata = getattr(args, "_iteration_metadata", None)
    if metadata and metadata["iteration_mode"]:
        lines.append("iteration: base=%s" % metadata["iteration_base"])
        scope = metadata["effective_proof_scope"]
        lines.append("proof scope: %s" % scope)
        if metadata.get("full_suite_reason"):
            detail = metadata.get("full_suite_detail")
            lines.append("full-suite expansion reason: %s%s" % (
                metadata["full_suite_reason"], " (%s)" % detail if detail else ""))
        if metadata.get("proof_omission_reason"):
            lines.append("proof omission reason: %s" % metadata["proof_omission_reason"])
        lines.append("iteration proof is not a complete final proof")
        lines.append("")
    status_tag = {"pass": "PASS", "fail": "FAIL", "error": "ERROR"}
    for r in results:
        if r.status == "skip":
            reason = r.skip_reason or ""
            if r.skip_reason_code:
                reason = "%s [%s]" % (reason, r.skip_reason_code)
            lines.append("  SKIP  %-11s %s" % (r.key, reason))
            continue
        tag = "EMIT" if r.disposition == "emit_required" else status_tag[r.status]
        dur = "(%.2fs)" % r.duration_s if r.duration_s is not None else ""
        head = headline(r)
        lines.append(("  %-5s %-11s %s  %s" % (tag, r.key, dur, head)).rstrip())
        # Every stage that ran echoes its whole output, not just a failing one. check_compile
        # prints `coverage:` and `unverified:` advisories on a passing run, and those are read
        # by a human rather than gated, so hiding them behind a failure would lose them. The
        # headline itself is already on the line above, so it is not repeated.
        body = (r.stdout or "").splitlines()
        if head in body:
            body.remove(head)
        for line in body:
            lines.append(" " * 8 + line)
        if r.stderr:
            lines.append(" " * 8 + "stderr:")
            for line in r.stderr.splitlines():
                lines.append(" " * 8 + line)

    counts = compute_counts(results)
    verdict, _exit_code = overall(results)
    lines.append("")
    lines.append("verdict: %s -- %d passed, %d failed, %d skipped, %d errored"
                 % (verdict.upper(), counts["pass"], counts["fail"], counts["skip"],
                    counts["error"]))

    faulted = [r for r in results if r.fault]
    if faulted:
        lines.append("")
        lines.append('first-pass fault classification (SKILL.md "Telling a draft fault from a '
                     'prose fault"):')
        for r in faulted:
            lines.append("  %-11s %s" % (r.key, r.fault))

    rerun = [r for r in results if r.status in ("fail", "error")]
    if rerun:
        lines.append("")
        lines.append("re-run one stage by hand:")
        for r in rerun:
            lines.append("  " + " ".join(r.command))

    return "\n".join(lines)


def build_json(results, paths, args, metadata=None):
    verdict, exit_code = overall(results)
    stages = []
    for r in results:
        stages.append({
            "key": r.key,
            "title": r.title,
            "status": r.status,
            "exit_code": r.exit_code,
            "duration_s": r.duration_s,
            "command": r.command,
            "headline": headline(r),
            "stdout": r.stdout,
            "stderr": r.stderr,
            "skip_reason": r.skip_reason,
            "fault": r.fault,
            "skip_reason_code": r.skip_reason_code,
            "check_status": ("fail" if r.disposition == "emit_required" else None),
            "check_exit_code": (1 if r.disposition == "emit_required" else None),
            "checker_json": r.checker_json,
            "policy_status": r.policy_status,
            "disposition": r.disposition,
        })
    obj = {
        "schema": SCHEMA,
        "gear": args.gear,
        "root": paths.root,
        "verdict": verdict,
        "exit_code": exit_code,
        "counts": compute_counts(results),
        "stages": stages,
    }
    if metadata:
        obj.update(metadata)
    return obj


def _setup_error_json(errors, paths, args, metadata=None):
    obj = {
        "schema": SCHEMA,
        "gear": args.gear,
        "root": paths.root,
        "verdict": "error",
        "exit_code": 2,
        "counts": {"pass": 0, "fail": 0, "skip": 0, "error": 0},
        "stages": [],
        "setup_errors": errors,
    }
    if metadata:
        obj.update(metadata)
    return obj


def _write_json_out(path, obj):
    with open(path, "w", encoding="utf-8") as fh:
        json.dump(obj, fh, indent=2)
        fh.write("\n")


def _default_metadata():
    return {
        "iteration_mode": False,
        "iteration_base": None,
        "effective_proof_scope": "omitted",
        "planned_proof_scope": "full",
        "selected_packages": [],
        "full_suite_reason": None,
        "full_suite_detail": None,
        "proof_omission_reason": None,
        "proof_omission_reasons": [],
        "proof_is_complete": False,
    }


def _iteration_metadata(base):
    metadata = _default_metadata()
    metadata.update({
        "iteration_mode": True,
        "iteration_base": base,
        "proof_is_complete": False,
    })
    return metadata


def finalize_default_metadata(metadata, results):
    proof = next((result for result in results if result.key == "proof"), None)
    if proof and proof.status != "skip":
        metadata["effective_proof_scope"] = "full"
        metadata["proof_is_complete"] = proof.status == "pass" and "--package" not in proof.command
    else:
        metadata["effective_proof_scope"] = "omitted"
        metadata["proof_is_complete"] = False


def _emit(obj, text, args):
    line = JSON_MARKER + json.dumps(obj)
    if args.json_out:
        _write_json_out(args.json_out, obj)
    if args.format == "json":
        print(line)
    else:
        print(text)
        print(line)
    return obj["exit_code"]


def main(argv=None):
    args = parse_args(sys.argv[1:] if argv is None else argv)
    paths = resolve_paths(args.gear, args.root)
    started = time.monotonic()
    args._handoff_setup_error = None

    if args.handoff_base is not None:
        args._handoff_base, handoff_error = resolve_iteration_base(paths.root, args.handoff_base)
        if handoff_error:
            args._handoff_base = None
            args._handoff_setup_error = handoff_error
    else:
        args._handoff_base = None
    if args._handoff_setup_error:
        args._iteration_metadata = _default_metadata()
        obj = _setup_error_json([args._handoff_setup_error], paths, args, args._iteration_metadata)
        obj["handoff"] = {"status": "blocked", "module": paths.module, "requirements": []}
        obj["timing"] = timing_metadata(args, [], args._iteration_metadata,
                                         time.monotonic() - started)
        return _emit(obj, "run_compile_gates: %s  root=%s\n\nsetup error:\n  - %s" %
                     (args.gear, paths.root, args._handoff_setup_error), args)

    if args.iteration_base is not None:
        resolved_base, base_error = resolve_iteration_base(paths.root, args.iteration_base)
        metadata = _iteration_metadata(resolved_base)
        args._iteration_metadata = metadata
        if base_error:
            obj = _setup_error_json([base_error], paths, args, metadata)
            obj["timing"] = timing_metadata(args, [], metadata, time.monotonic() - started)
            report = ["run_compile_gates: %s  root=%s" % (args.gear, paths.root), "",
                      "setup error:", "  - %s" % base_error]
            return _emit(obj, "\n".join(report), args)

        errors = iteration_setup_errors(paths, args)
        if errors:
            obj = _setup_error_json(errors, paths, args, metadata)
            obj["timing"] = timing_metadata(args, [], metadata, time.monotonic() - started)
            report = ["run_compile_gates: %s  root=%s" % (args.gear, paths.root), "",
                      "setup error:"]
            report.extend("  - %s" % error for error in errors)
            return _emit(obj, "\n".join(report), args)

        scope = iteration_scope(paths.root, args.gear, resolved_base)
        metadata.update(scope)
        metadata["planned_proof_scope"] = metadata["effective_proof_scope"]
        results, omission_reason, omission_reasons = execute_iteration(paths, args, scope)
        metadata["proof_omission_reason"] = omission_reason
        metadata["proof_omission_reasons"] = omission_reasons
        proof_result = next(result for result in results if result.key == "proof")
        if omission_reason or proof_result.skip_reason_code == "proof_setup_failure":
            metadata["effective_proof_scope"] = "omitted"
        faults = classify(results)
        results = [dataclasses.replace(r, fault=faults.get(r.key)) for r in results]
        handoff = handoff_for(results, paths, args)
        args._handoff_status = handoff["status"]
        results = apply_handoff_policy(results, handoff)
        obj = build_json(results, paths, args, metadata)
        obj["handoff"] = handoff
        obj["timing"] = timing_metadata(args, results, metadata, time.monotonic() - started)
        return _emit(obj, render_text(results, paths, args, handoff), args)

    errors = setup_errors(paths, args)
    if errors:
        args._iteration_metadata = _default_metadata()
        obj = _setup_error_json(errors, paths, args, args._iteration_metadata)
        obj["timing"] = timing_metadata(args, [], args._iteration_metadata,
                                         time.monotonic() - started)
        report = ["run_compile_gates: %s  root=%s" % (args.gear, paths.root), "", "setup error:"]
        report.extend("  - %s" % e for e in errors)
        return _emit(obj, "\n".join(report), args)

    args._iteration_metadata = _default_metadata()
    results = execute(build_plan(paths, args), paths, args)
    finalize_default_metadata(args._iteration_metadata, results)
    faults = classify(results)
    results = [dataclasses.replace(r, fault=faults.get(r.key)) for r in results]
    handoff = handoff_for(results, paths, args)
    args._handoff_status = handoff["status"]
    results = apply_handoff_policy(results, handoff)

    obj = build_json(results, paths, args, args._iteration_metadata)
    obj["handoff"] = handoff
    obj["timing"] = timing_metadata(args, results, args._iteration_metadata,
                                     time.monotonic() - started)
    return _emit(obj, render_text(results, paths, args, handoff), args)


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
