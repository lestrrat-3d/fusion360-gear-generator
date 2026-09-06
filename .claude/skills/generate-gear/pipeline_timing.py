#!/usr/bin/env python3
"""Record small, append-only measurements for one gear pipeline run.

The event files are deliberately boring JSON.  ``run.json`` owns the run identity and
each event is written to a fresh file with an atomic rename.  A CLI invocation uses UTC
epoch seconds because its start and finish commands run in different processes.  Callers
that keep one process alive can pass a monotonic clock and receive monotonic intervals.
This module records no prompts, source text, or private session data.
"""
import argparse
import datetime as _datetime
import hashlib
import json
import math
import os
import subprocess
import sys
import tempfile
import time
import uuid
from dataclasses import dataclass


SCHEMA = 1
EVENT_PREFIX = "event-"
RUN_FILE = "run.json"
EVENT_DIR = "events"
PHASES = ("preflight", "input_reading", "drafting", "validation", "placement", "overall")
ACTIONS = ("start", "finish", "record")
TIMING_SCHEMA = 1


class TimingError(Exception):
    """Raised for malformed run data or an invalid timing operation."""


@dataclass
class SystemClock:
    """Clock used by the event owner.

    ``monotonic`` is process-local.  ``epoch`` is UTC epoch seconds and can be compared
    across the separate CLI calls used by an agent.
    """

    def monotonic(self):
        return time.monotonic()

    def epoch(self):
        return time.time()


def _utc(epoch):
    return _datetime.datetime.fromtimestamp(epoch, _datetime.timezone.utc).isoformat()


def _atomic_json(path, value):
    directory = os.path.dirname(path)
    os.makedirs(directory, exist_ok=True)
    fd, temporary = tempfile.mkstemp(prefix=".timing-", suffix=".tmp", dir=directory)
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as handle:
            json.dump(value, handle, indent=2, sort_keys=True)
            handle.write("\n")
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(temporary, path)
    except BaseException:
        try:
            os.unlink(temporary)
        except OSError:
            pass
        raise


def _read_json(path):
    try:
        with open(path, encoding="utf-8") as handle:
            return json.load(handle)
    except (OSError, ValueError) as error:
        raise TimingError("cannot read %s: %s" % (path, error)) from error


def _git_commit(root):
    try:
        result = subprocess.run(
            ["git", "-C", root, "rev-parse", "HEAD"],
            capture_output=True, text=True, check=False,
        )
    except OSError:
        return None
    if result.returncode:
        return None
    value = result.stdout.strip()
    return value or None


def _input_record(path, root):
    absolute = os.path.abspath(path if os.path.isabs(path) else os.path.join(root, path))
    try:
        size = os.path.getsize(absolute)
        digest = hashlib.sha256()
        with open(absolute, "rb") as handle:
            for block in iter(lambda: handle.read(1024 * 1024), b""):
                digest.update(block)
    except OSError as error:
        raise TimingError("cannot read timing input %s: %s" % (path, error)) from error
    relative = os.path.relpath(absolute, root)
    return {"path": relative, "size_bytes": size, "sha256": digest.hexdigest()}


def _load_run(run_dir):
    run_dir = os.path.abspath(os.fspath(run_dir))
    metadata = _read_json(os.path.join(run_dir, RUN_FILE))
    if not isinstance(metadata, dict) or metadata.get("schema") != SCHEMA:
        raise TimingError("%s has an unsupported run schema" % run_dir)
    if not metadata.get("run_id"):
        raise TimingError("%s has no run_id" % run_dir)
    return run_dir, metadata


def start_run(run_dir, gear, stage, *, root=None, inputs=(), model_role=None,
              model=None, run_id=None, git_commit=None, round=1, clock=None):
    """Create a run directory and its first overall start event.

    Reusing any existing directory is rejected, even when it contains no timing files.
    ``inputs`` contains paths whose size and SHA-256 digest should be recorded.
    """
    run_dir = os.path.abspath(os.fspath(run_dir))
    if os.path.exists(run_dir):
        raise TimingError("run directory already exists: %s" % run_dir)
    if not gear or not stage:
        raise TimingError("gear and stage are required")
    if not isinstance(round, int) or isinstance(round, bool) or round < 1:
        raise TimingError("round must be a positive integer")
    root = os.path.abspath(root or os.getcwd())
    clock = clock or SystemClock()
    epoch = float(clock.epoch())
    metadata = {
        "schema": SCHEMA,
        "run_id": run_id or str(uuid.uuid4()),
        "gear": gear,
        "stage": stage,
        "starting_git_commit": git_commit if git_commit is not None else _git_commit(root),
        "inputs": [_input_record(path, root) for path in inputs],
        "model_role": model_role,
        "model": model,
        "started_at_utc": _utc(epoch),
        "clock": "utc_epoch",
        "round": round,
    }
    os.makedirs(os.path.join(run_dir, EVENT_DIR))
    try:
        _atomic_json(os.path.join(run_dir, RUN_FILE), metadata)
        record_event(run_dir, "overall", "start", round=round, clock=clock)
    except BaseException:
        # A failed start must not leave a plausible run for a later invocation.
        for name in (RUN_FILE,):
            try:
                os.unlink(os.path.join(run_dir, name))
            except OSError:
                pass
        try:
            os.rmdir(os.path.join(run_dir, EVENT_DIR))
            os.rmdir(run_dir)
        except OSError:
            pass
        raise
    return metadata


def _event_files(run_dir):
    event_dir = os.path.join(run_dir, EVENT_DIR)
    try:
        names = sorted(name for name in os.listdir(event_dir)
                       if name.startswith(EVENT_PREFIX) and name.endswith(".json"))
    except OSError as error:
        raise TimingError("cannot read event directory %s: %s" % (event_dir, error)) from error
    return [os.path.join(event_dir, name) for name in names]


def load_events(run_dir):
    """Return ``(metadata, events, issues)`` while retaining malformed event evidence."""
    run_dir, metadata = _load_run(run_dir)
    events = []
    issues = []
    for path in _event_files(run_dir):
        try:
            event = _read_json(path)
            if not isinstance(event, dict):
                raise TimingError("event is not an object")
            if event.get("schema") != SCHEMA:
                raise TimingError("unsupported event schema")
            if event.get("run_id") != metadata["run_id"]:
                raise TimingError("event run_id does not match run.json")
            if event.get("gear") != metadata.get("gear") or event.get("stage") != metadata.get("stage"):
                raise TimingError("event gear/stage does not match run.json")
            sequence = event.get("sequence")
            if (not isinstance(sequence, int) or isinstance(sequence, bool) or sequence < 1):
                raise TimingError("event sequence must be a positive integer")
            if event.get("phase") not in PHASES:
                raise TimingError("unknown event phase")
            if event.get("action") not in ACTIONS:
                raise TimingError("unknown event action")
            if not isinstance(event.get("event"), str) or not event["event"]:
                raise TimingError("event name must be a non-empty string")
            if (not isinstance(event.get("round"), int) or isinstance(event.get("round"), bool)
                    or event["round"] < 1):
                raise TimingError("event round must be a positive integer")
            if not isinstance(event.get("timestamp_utc"), str) or not event["timestamp_utc"]:
                raise TimingError("event timestamp_utc must be a non-empty string")
            timestamp = event.get("timestamp_epoch_s")
            if (not isinstance(timestamp, (int, float)) or isinstance(timestamp, bool)
                    or not math.isfinite(timestamp) or timestamp < 0):
                raise TimingError("event timestamp_epoch_s must be finite and non-negative")
            duration = event.get("duration_s")
            if (duration is not None and
                    (not isinstance(duration, (int, float)) or isinstance(duration, bool)
                     or not math.isfinite(duration) or duration < 0)):
                raise TimingError("event duration_s must be finite and non-negative")
            if "metadata" in event and not isinstance(event["metadata"], dict):
                raise TimingError("event metadata must be an object")
            for token_key in ("input_tokens", "output_tokens"):
                token_value = event.get(token_key)
                if (token_value is not None and
                        (not isinstance(token_value, (int, float)) or isinstance(token_value, bool)
                         or not math.isfinite(token_value) or token_value < 0)):
                    raise TimingError("%s must be finite and non-negative" % token_key)
            events.append(event)
        except TimingError as error:
            issues.append({"file": os.path.basename(path), "error": str(error)})
    return metadata, events, issues


def _next_event_path(run_dir):
    # UUIDs avoid collisions between concurrent writers; the numeric prefix keeps reports
    # stable for ordinary runs and makes interrupted writes harmless.
    number = len(_event_files(run_dir)) + 1
    while True:
        path = os.path.join(run_dir, EVENT_DIR, "%s%06d-%s.json" %
                            (EVENT_PREFIX, number, uuid.uuid4().hex[:8]))
        if not os.path.exists(path):
            return path
        number += 1


def record_event(run_dir, phase, action, *, round=1, event_name=None, clock=None,
                 duration_s=None, model_role=None, model=None, input_tokens=None,
                 output_tokens=None, metadata=None):
    """Write one event and return its JSON object.

    A finish event derives its interval from UTC timestamps, which is the only clock that
    remains comparable across CLI processes.  A supplied non-negative ``duration_s`` is
    accepted for an interval measured in one process.  Negative intervals are rejected.
    """
    run_dir, run = _load_run(run_dir)
    if phase not in PHASES:
        raise TimingError("unknown phase %r" % phase)
    if action not in ACTIONS:
        raise TimingError("unknown action %r" % action)
    if not isinstance(round, int) or isinstance(round, bool) or round < 1:
        raise TimingError("round must be a positive integer")
    if duration_s is not None:
        duration_s = float(duration_s)
        if not math.isfinite(duration_s) or duration_s < 0:
            raise TimingError("duration_s must be finite and non-negative")
    supplied_clock = clock is not None
    clock = clock or SystemClock()
    epoch = float(clock.epoch())
    sequence = len(_event_files(run_dir)) + 1
    event = {
        "schema": SCHEMA,
        "sequence": sequence,
        "run_id": run["run_id"],
        "gear": run["gear"],
        "stage": run["stage"],
        "starting_git_commit": run.get("starting_git_commit"),
        "input_digests": run.get("inputs", []),
        "model_role": model_role if model_role is not None else run.get("model_role"),
        "model": model if model is not None else run.get("model"),
        "round": round,
        "phase": phase,
        "action": action,
        "event": event_name or "%s.%s" % (phase, action),
        "timestamp_utc": _utc(epoch),
        "timestamp_epoch_s": epoch,
        "clock": "monotonic_process" if supplied_clock else "utc_epoch",
        "clock_id": os.getpid() if supplied_clock else None,
        "monotonic_s": float(clock.monotonic()) if supplied_clock else None,
        "duration_s": duration_s,
        "input_tokens": input_tokens,
        "output_tokens": output_tokens,
    }
    if metadata:
        if not isinstance(metadata, dict):
            raise TimingError("event metadata must be an object")
        event["metadata"] = metadata
    if action == "finish" and duration_s is None:
        starts = [item for item in load_events(run_dir)[1]
                  if item.get("phase") == phase and item.get("action") == "start"
                  and item.get("round") == round]
        if starts:
            start = starts[-1]
            if (supplied_clock and start.get("clock") == "monotonic_process"
                    and start.get("clock_id") == os.getpid()):
                duration_s = float(clock.monotonic()) - float(start.get("monotonic_s"))
            else:
                duration_s = epoch - float(start.get("timestamp_epoch_s", epoch))
            if duration_s < 0:
                raise TimingError("finish timestamp precedes start timestamp")
            event["duration_s"] = round_seconds(duration_s)
    _atomic_json(_next_event_path(run_dir), event)
    return event


def round_seconds(value):
    return round(float(value), 6)


def _union(intervals):
    usable = sorted((float(start), float(end)) for start, end in intervals if end >= start)
    total = 0.0
    current = None
    for start, end in usable:
        if current is None:
            current = [start, end]
        elif start <= current[1]:
            current[1] = max(current[1], end)
        else:
            total += current[1] - current[0]
            current = [start, end]
    if current is not None:
        total += current[1] - current[0]
    return round_seconds(total)


def _gate_timing(report):
    timing = report.get("timing")
    return timing if isinstance(timing, dict) else {}


def gate_import(run_dir, round, file_path):
    """Import only counters and timing metadata from a gate runner JSON report."""
    if not isinstance(round, int) or isinstance(round, bool) or round < 1:
        raise TimingError("round must be a positive integer")
    run_dir, run = _load_run(run_dir)
    try:
        with open(file_path, "rb") as handle:
            raw = handle.read()
        report = json.loads(raw.decode("utf-8"))
    except (OSError, UnicodeDecodeError, ValueError) as error:
        raise TimingError("cannot read gate JSON %s: %s" % (file_path, error)) from error
    if (not isinstance(report, dict) or isinstance(report.get("schema"), bool)
            or not isinstance(report.get("schema"), int)):
        raise TimingError("gate JSON must be an object with an integer schema")
    source_digest = hashlib.sha256(raw).hexdigest()
    _metadata, events, _issues = load_events(run_dir)
    prior = [item for item in events if item.get("event") == "gate_import" and
             item.get("metadata", {}).get("source_sha256") == source_digest]
    if any(item.get("round") == round for item in prior):
        return {"duplicate": True, "source_sha256": source_digest}

    timing = _gate_timing(report)
    gates = report.get("gates", report.get("stages", []))
    if not isinstance(gates, list):
        raise TimingError("gate JSON gates/stages must be an array")
    failures = []
    skipped = []
    advisory_findings = 0
    gate_duration = 0.0
    for gate in gates:
        if not isinstance(gate, dict):
            raise TimingError("gate JSON contains a non-object gate row")
        status = gate.get("status")
        key = gate.get("key")
        if status in ("fail", "error"):
            failures.append({"key": key, "status": status, "advisory": bool(gate.get("advisory"))})
        if status == "skip":
            skipped.append(key)
        value = gate.get("duration_s")
        if isinstance(value, (int, float)) and value >= 0:
            gate_duration += float(value)
        if key in ("novel_types",) and status == "note":
            counts = report.get("counts", {})
            advisory_findings = max(advisory_findings, int(counts.get("advisory_findings", 0) or 0))
    shared_duration = timing.get("analysis_duration_s")
    if (isinstance(shared_duration, (int, float)) and not isinstance(shared_duration, bool)
            and math.isfinite(shared_duration) and shared_duration >= 0
            and timing.get("analysis_shared")):
        # Task02 reports the shared operation once in the first type row. This field is
        # still authoritative when an older report accidentally repeated both rows.
        type_duration = sum(float(gate.get("duration_s")) for gate in gates
                            if gate.get("key") in ("pyright", "novel_types")
                            and isinstance(gate.get("duration_s"), (int, float)))
        gate_duration = gate_duration - type_duration + float(shared_duration)
    gate_duration = round_seconds(max(0.0, gate_duration))
    policy = timing.get("gate_policy")
    if not isinstance(policy, dict):
        policy = {}
    first_pass = bool(timing.get("first_pass_eligible", False))
    if report.get("verdict") not in ("pass",):
        first_pass = False
    payload = {
        "source_sha256": source_digest,
        "source_name": os.path.basename(file_path),
        "report_schema": report["schema"],
        "verdict": report.get("verdict"),
        "exit_code": report.get("exit_code"),
        "gate_count": len(gates),
        "gate_failures": failures,
        "skipped_gates": skipped,
        "gate_duration_s": gate_duration,
        "wall_time_s": timing.get("wall_time_s"),
        "analysis_duration_s": timing.get("analysis_duration_s"),
        "analysis_pyright_duration_s": timing.get("analysis_pyright_duration_s"),
        "analysis_invocations": timing.get("analysis_invocations"),
        "analysis_shared": timing.get("analysis_shared"),
        "gate_policy": policy,
        "proof_is_complete": report.get("proof_is_complete"),
        "first_pass_eligible": first_pass,
        "advisory_findings": advisory_findings,
    }
    if prior:
        payload["source_reused_from_round"] = prior[0].get("round")
    return record_event(run_dir, "validation", "record", round=round,
                        event_name="gate_import", metadata=payload)


def _intervals(events):
    starts = {}
    intervals = []
    issues = []
    # Event files are loaded in their numeric filename order. Keep that recorded sequence
    # when timestamps tie; event names are descriptive and must not reorder the interval.
    for event in sorted(events, key=lambda item: (item["sequence"],
                                                  item["timestamp_epoch_s"])):
        key = (event.get("phase"), event.get("round"))
        action = event.get("action")
        if action == "start":
            if key in starts:
                issues.append("duplicate start for %s round %s" % key)
            starts[key] = event
        elif action == "finish":
            start = starts.pop(key, None)
            if start is None:
                issues.append("finish without start for %s round %s" % key)
                continue
            start_epoch = float(start.get("timestamp_epoch_s"))
            finish_epoch = float(event.get("timestamp_epoch_s"))
            duration = event.get("duration_s")
            if duration is None:
                duration = finish_epoch - start_epoch
            if float(duration) < 0 or finish_epoch < start_epoch:
                issues.append("negative interval for %s round %s" % key)
                continue
            intervals.append({"phase": key[0], "round": key[1],
                              "start": start_epoch, "end": start_epoch + float(duration),
                              "duration_s": round_seconds(duration)})
    issues.extend("missing finish for %s round %s" % key for key in sorted(starts))
    return intervals, issues


def summarize(run_dir):
    """Return a compact report. Any malformed or incomplete evidence makes it incomplete."""
    metadata, events, issues = load_events(run_dir)
    intervals, interval_issues = _intervals(events)
    issues.extend(interval_issues)
    imports = [event for event in events if event.get("event") == "gate_import"]
    phase_times = {}
    for phase in PHASES:
        phase_times[phase] = _union((item["start"], item["end"]) for item in intervals
                                    if item["phase"] == phase)
    gate_duration = sum(float(event.get("metadata", {}).get("gate_duration_s") or 0)
                        for event in imports)
    wall_times = [event.get("metadata", {}).get("wall_time_s") for event in imports
                  if isinstance(event.get("metadata", {}).get("wall_time_s"), (int, float))]
    phase_observed = {phase: any(item["phase"] == phase for item in intervals)
                      for phase in PHASES}
    validation_time = (phase_times["validation"] if phase_observed["validation"]
                       else round_seconds(sum(wall_times)) if wall_times else None)
    overall_intervals = [item for item in intervals if item["phase"] == "overall"]
    total_wall = (_union((item["start"], item["end"]) for item in overall_intervals)
                  if overall_intervals else None)
    if total_wall is None:
        issues.append("overall run completion is missing")

    for required_phase in ("drafting", "validation"):
        if not any(item["phase"] == required_phase for item in intervals):
            issues.append("%s boundary is missing" % required_phase)

    rounds = {event.get("round") for event in imports if isinstance(event.get("round"), int)}
    completed_rounds = sum(1 for number in rounds
                           if any(item["round"] == number and item["phase"] == "drafting"
                                  for item in intervals)
                           and any(event.get("round") == number for event in imports))
    failures = []
    advisory = []
    first_pass = False
    for event in imports:
        payload = event.get("metadata", {})
        failures.extend({"round": event.get("round"), **failure} for failure in payload.get("gate_failures", []))
        if payload.get("advisory_findings", 0):
            advisory.append({"round": event.get("round"), "findings": payload["advisory_findings"]})
        if event.get("round") == 1 and payload.get("first_pass_eligible"):
            first_pass = True
    token_events = [event for event in events if event.get("input_tokens") is not None or
                    event.get("output_tokens") is not None]
    input_tokens = sum(event["input_tokens"] for event in token_events
                       if isinstance(event.get("input_tokens"), (int, float)))
    output_tokens = sum(event["output_tokens"] for event in token_events
                        if isinstance(event.get("output_tokens"), (int, float)))
    have_input = any(event.get("input_tokens") is not None for event in token_events)
    have_output = any(event.get("output_tokens") is not None for event in token_events)
    round_one_drafts = [item for item in intervals
                        if item["phase"] == "drafting" and item["round"] == 1]
    round_one_imports = [event for event in imports if event.get("round") == 1]
    if (not round_one_drafts or not round_one_imports or
            any(event.get("metadata", {}).get("gate_failures") for event in round_one_imports) or
            any(event.get("metadata", {}).get("verdict") != "pass" for event in round_one_imports)):
        first_pass = False
    triage_states = [event.get("metadata", {}).get("advisory_triage") for event in events
                     if event.get("metadata", {}).get("advisory_triage") in ("complete", "incomplete")]
    triage_state = triage_states[-1] if triage_states else None
    if triage_state is None:
        issues.append("advisory triage completion was not recorded")
    elif triage_state == "incomplete":
        issues.append("advisory triage is incomplete")
    triage_complete = (True if triage_state == "complete" else
                       False if triage_state == "incomplete" else None)
    complete = not issues and bool(overall_intervals) and bool(imports)
    evidence_issues = [issue for issue in issues
                       if issue != "advisory triage completion was not recorded"
                       and issue != "advisory triage is incomplete"]
    if evidence_issues or not overall_intervals:
        first_pass = False
    reasons = list(issues)
    if not first_pass:
        reasons.append("round 1 is not a complete full-policy first pass")
    return {
        "schema": SCHEMA,
        "run_id": metadata["run_id"],
        "gear": metadata["gear"],
        "stage": metadata["stage"],
        "starting_git_commit": metadata.get("starting_git_commit"),
        "drafting_time_s": phase_times["drafting"] if phase_observed["drafting"] else None,
        "validation_time_s": validation_time,
        "gate_duration_s": round_seconds(gate_duration),
        "preflight_time_s": phase_times["preflight"] if phase_observed["preflight"] else None,
        "placement_time_s": phase_times["placement"] if phase_observed["placement"] else None,
        "total_wall_time_s": total_wall,
        "completed_rounds": completed_rounds,
        "gate_failures": len(failures),
        "gate_failure_details": failures,
        "token_counts": {"input": round_seconds(input_tokens) if have_input else None,
                         "output": round_seconds(output_tokens) if have_output else None},
        "first_pass": first_pass,
        "advisory_triage": {"complete": triage_complete, "status": triage_state or "unknown",
                            "findings": advisory},
        "complete": complete,
        "completeness": {"complete": complete, "reasons": reasons},
        "issues": issues,
    }


def gate_policy_for_run(args, results):
    """Build additive policy metadata for ``run_gates.py`` without changing its verdict."""
    skipped = [result.key for result in results if result.status == "skip"]
    required = [key for key in ("parse", "input_read", "contract", "step_calls", "anchors",
                                "api_calls", "pyright") if not args.only or key in args.only]
    full = args.only is None and not args.fail_fast and not args.no_advisory
    allowed = [result.key for result in results if result.status == "skip" and (
        result.key == "contract" and not args.require_contract and
        (result.skip_reason or "").startswith("no manifest at ") or
        result.key == "step_calls" and args.skip_missing_steps and
        "no compiled step list" in (result.skip_reason or ""))]
    required_ok = all(result.status == "pass" or
                      (result.status == "skip" and result.key in allowed)
                      for result in results if result.key in required)
    present = {result.key for result in results}
    required_ok = required_ok and set(required).issubset(present)
    eligible = full and required_ok and not any(result.status in ("fail", "error")
                                                for result in results)
    return {
        "kind": "run_gates",
        "mode": "full" if full else "selected",
        "required_gates": required,
        "skipped_gates": skipped,
        "allowed_skips": sorted(set(allowed)),
        "advisory_enabled": not args.no_advisory,
        "novel_types_blocking": bool(args.gate_novel_types),
        "fail_fast": bool(args.fail_fast),
        "first_pass_eligible": eligible,
    }


def compile_policy_for_run(args, results, metadata):
    """Build additive policy metadata for ``run_compile_gates.py``."""
    full = not metadata.get("iteration_mode") and not getattr(args, "only", None)
    allowed_skip = [result.key for result in results if result.status == "skip" and
                    result.key == "step_calls" and "module" in (result.skip_reason or "")]
    required_ok = bool(results) and all(result.status == "pass" or result.key in allowed_skip
                                        for result in results)
    eligible = full and bool(metadata.get("proof_is_complete")) and required_ok and not any(
        result.status in ("fail", "error") for result in results)
    return {
        "kind": "run_compile_gates",
        "mode": "full" if full else "selected",
        "proof_scope": metadata.get("effective_proof_scope"),
        "proof_is_complete": bool(metadata.get("proof_is_complete")),
        "first_pass_eligible": eligible,
    }


def cli_error(message):
    sys.stderr.write("pipeline_timing.py: %s\n" % message)
    return 2


def _parser():
    parser = argparse.ArgumentParser(description="Record one gear pipeline's timing events.")
    sub = parser.add_subparsers(dest="command", required=True)
    start = sub.add_parser("start")
    start.add_argument("--gear", required=True)
    start.add_argument("--stage", required=True)
    start.add_argument("--run-dir", required=True)
    start.add_argument("--root", default=None)
    start.add_argument("--input", action="append", default=[])
    start.add_argument("--model-role", default=None)
    start.add_argument("--model", default=None)
    start.add_argument("--run-id", default=None)
    start.add_argument("--round", type=int, default=1)

    event = sub.add_parser("event")
    event.add_argument("--run-dir", required=True)
    event.add_argument("--phase", choices=PHASES, required=True)
    event.add_argument("--action", choices=ACTIONS, required=True)
    event.add_argument("--round", type=int, default=1)
    event.add_argument("--event-name", default=None)
    event.add_argument("--duration-s", type=float, default=None)
    event.add_argument("--model-role", default=None)
    event.add_argument("--model", default=None)
    event.add_argument("--input-tokens", type=int, default=None)
    event.add_argument("--output-tokens", type=int, default=None)
    event.add_argument("--advisory-triage", choices=("complete", "incomplete"), default=None)

    imported = sub.add_parser("import-gates")
    imported.add_argument("--run-dir", required=True)
    imported.add_argument("--round", type=int, required=True)
    imported.add_argument("--file", required=True)

    summary = sub.add_parser("summarize")
    summary.add_argument("--run-dir", required=True)
    summary.add_argument("--format", choices=("text", "json"), default="text")
    return parser


def main(argv=None):
    args = _parser().parse_args(argv)
    try:
        if args.command == "start":
            result = start_run(args.run_dir, args.gear, args.stage, root=args.root,
                               inputs=args.input, model_role=args.model_role, model=args.model,
                               run_id=args.run_id, round=args.round)
        elif args.command == "event":
            result = record_event(args.run_dir, args.phase, args.action, round=args.round,
                                  event_name=args.event_name, duration_s=args.duration_s,
                                  model_role=args.model_role, model=args.model,
                                  input_tokens=args.input_tokens, output_tokens=args.output_tokens,
                                  metadata=({"advisory_triage": args.advisory_triage}
                                            if args.advisory_triage else None))
        elif args.command == "import-gates":
            result = gate_import(args.run_dir, args.round, args.file)
        else:
            result = summarize(args.run_dir)
    except TimingError as error:
        return cli_error(str(error))
    if args.command == "summarize" and args.format == "text":
        print("pipeline timing: %s/%s" % (result["gear"], result["stage"]))
        print("drafting: %s  validation: %s  total wall: %s" %
              (result["drafting_time_s"], result["validation_time_s"], result["total_wall_time_s"]))
        print("rounds: %d  gate failures: %d  complete: %s  first pass: %s" %
              (result["completed_rounds"], result["gate_failures"], result["complete"], result["first_pass"]))
        print("advisory triage complete: %s" % result["advisory_triage"]["complete"])
    else:
        print(json.dumps(result, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    sys.exit(main())
