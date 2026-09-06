#!/usr/bin/env python3
"""Run the complete gate policy for several gears with shared anchor and type work."""
import argparse
import ast
import dataclasses
import importlib.util
import json
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
if HERE not in sys.path:
    sys.path.insert(0, HERE)
import run_gates


def _module(name):
    spec = importlib.util.spec_from_file_location("ci_gates_" + name, os.path.join(HERE, name + ".py"))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def parse_args(argv):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--gear", action="append", required=True, metavar="NAME=CANDIDATE")
    parser.add_argument("--root", default=None)
    parser.add_argument("--require-contract", default="")
    parser.add_argument("--gate-novel-types", action="store_true")
    parser.add_argument("--timeout", type=float, default=run_gates.DEFAULT_TIMEOUT)
    parser.add_argument("--json-out", required=True)
    parser.add_argument("--report-dir", required=True)
    parser.add_argument("--format", choices=("text", "json"), default="text")
    parser.add_argument("--skip-anchors", action="store_true")
    return parser.parse_args(argv)


def _root_relative(path, root):
    path = os.path.abspath(path)
    rel = os.path.relpath(path, root)
    return path if rel.startswith("..") else rel


def _specs(args):
    if not args.gear:
        raise ValueError("at least one --gear mapping is required")
    seen = set()
    result = []
    for mapping in args.gear:
        if "=" not in mapping:
            raise ValueError("malformed --gear mapping: %s" % mapping)
        name, candidate = mapping.split("=", 1)
        if not name or not candidate or any(ch.isspace() for ch in name):
            raise ValueError("malformed --gear mapping: %s" % mapping)
        if name in seen:
            raise ValueError("duplicate gear name: %s" % name)
        seen.add(name)
        result.append((name, candidate))
    required = [item.strip() for item in args.require_contract.split(",") if item.strip()]
    unknown = sorted(set(required) - seen)
    if unknown:
        raise ValueError("--require-contract names are not listed: %s" % ", ".join(unknown))
    return result, set(required)


def _writable_destination(path, directory=False):
    target = os.path.abspath(path)
    parent = target if directory else os.path.dirname(target)
    if not parent or not os.path.isdir(parent):
        raise ValueError("report destination is not writable: %s" % path)
    if directory:
        if not os.access(parent, os.W_OK):
            raise ValueError("report destination is not writable: %s" % path)
    else:
        if os.path.isdir(target) or (os.path.exists(target) and not os.access(target, os.W_OK)):
            raise ValueError("report destination is not writable: %s" % path)


def _setup_report(args, paths, errors, anchor=None):
    rows = []
    for key in run_gates.GATE_ORDER:
        status = ((anchor.status if anchor is not None else "error") if key == "anchors" else
                  ("fail" if key == "parse" and
                                                     any("does not parse" in e for e in errors)
                                                     else "error"))
        rows.append({"key": key, "title": run_gates.GATE_TITLES[key], "status": status,
                     "advisory": key == "novel_types", "exit_code": 1 if status == "fail" else
                     (0 if status == "pass" else 2),
                     "duration_s": 0.0, "command": [], "headline": "setup error",
                     "stdout": "parse: SyntaxError" if status == "fail" else "",
                     "stderr": ("ERROR: " + "; ".join(errors) if status == "error" else
                                (anchor.stderr if key == "anchors" and anchor else "")),
                     "skip_reason": None, "fault": "setup", "timing_note": None})
    return {"schema": 1, "gear": args.gear, "candidate": paths.candidate, "root": paths.root,
            "verdict": "setup_error", "exit_code": 2,
            "counts": {"pass": 0, "fail": 0, "skip": 0, "error": len(rows),
                       "advisory_findings": 0}, "gates": rows, "classification": [],
            "setup_errors": errors, "metadata": {}, "timing": {}}


def _write(path, value):
    with open(path, "w", encoding="utf-8") as handle:
        json.dump(value, handle, indent=2)
        handle.write("\n")


def _aggregate(root, entries, shared, started, runner_started):
    counts = {"pass": 0, "fail": 0, "error": 0}
    for entry in entries:
        verdict = entry["verdict"]
        if verdict == "pass":
            counts["pass"] += 1
        elif verdict == "fail":
            counts["fail"] += 1
        else:
            counts["error"] += 1
    if counts["error"]:
        verdict, code = "error", 2
    elif counts["fail"]:
        verdict, code = "fail", 1
    else:
        verdict, code = "pass", 0
    return {"schema": 1, "kind": "cross_gear_gates", "root": root, "gears": entries,
            "shared": shared, "counts": counts, "verdict": verdict, "exit_code": code,
            "timing": {"wall_time_s": round(time.monotonic() - started, 6),
                        "runner_time_s": round(time.monotonic() - runner_started, 6)}}


def main(argv=None):
    try:
        args = parse_args(sys.argv[1:] if argv is None else argv)
        specs, required = _specs(args)
        root = os.path.abspath(args.root or run_gates.repo_root())
        report_dir = os.path.abspath(args.report_dir if os.path.isabs(args.report_dir)
                                     else os.path.join(root, args.report_dir))
        json_out = os.path.abspath(args.json_out if os.path.isabs(args.json_out)
                                   else os.path.join(root, args.json_out))
        try:
            os.makedirs(report_dir, exist_ok=True)
        except OSError as error:
            raise ValueError("report destination is not writable: %s (%s)" %
                             (report_dir, error))
        _writable_destination(report_dir, directory=True)
        _writable_destination(json_out)
    except (SystemExit, ValueError) as error:
        if isinstance(error, SystemExit) and error.code == 0:
            raise
        print("run_ci_gates: %s" % error, file=sys.stderr)
        return 2

    started = time.monotonic()
    runner_started = time.monotonic()
    prepared = []
    for gear, candidate in specs:
        options = argparse.Namespace(gear=gear, candidate=candidate, root=root, only=None,
                                     fail_fast=False, require_contract=gear in required,
                                     skip_missing_steps=False, gate_novel_types=args.gate_novel_types,
                                     no_advisory=False, json_out=None, format="json",
                                     timeout=args.timeout)
        prepared.append(run_gates.prepare_run(gear, candidate, options))

    anchor_start = time.monotonic()
    anchor_cmd = [sys.executable, os.path.join(HERE, "check_anchors.py")]
    anchor = (run_gates.GateResult("anchors", "Anchors", "skip", False, None, 0.0, [], "", "",
                                   "--skip-anchors", None) if args.skip_anchors else
              run_gates.run_script_gate("anchors", "Anchors", anchor_cmd, root, args.timeout, False))
    anchor_duration = round(time.monotonic() - anchor_start, 2)
    anchor = dataclasses.replace(anchor, duration_s=anchor_duration)

    novel = _module("check_novel_types")
    pyright = _module("pyright_check")
    plans = {}
    plan_errors = {}
    valid = []
    for gear, (candidate_args, paths, errors) in zip((item[0] for item in specs), prepared):
        if errors:
            continue
        try:
            with open(os.path.join(root, paths.candidate), encoding="utf-8") as handle:
                ast.parse(handle.read())
        except (OSError, SyntaxError) as error:
            plan_errors[gear] = "candidate does not parse: %s" % error
            continue
        try:
            plan = novel.plan_evaluation(os.path.join(root, "lib", "geargen"),
                                         os.path.join(root, paths.candidate))
        except Exception as error:
            plan_errors[gear] = str(error)
            continue
        plans[gear] = plan
        valid.append(plan)
    union = []
    seen = set()
    for plan in valid:
        for path in list(plan.references) + [plan.candidate]:
            absolute = os.path.abspath(path)
            if absolute not in seen:
                seen.add(absolute)
                union.append(absolute)
    analysis_start = time.monotonic()
    analysis = pyright.analyze_paths(union, root=root, timeout=args.timeout) if union else \
        pyright.AnalysisResult({}, pyright.AnalysisMetadata(root), "no valid candidates")
    if not analysis.setup_error and len(analysis.metadata.invocations) != 1:
        analysis.setup_error = "batch analysis expected one invocation, got %d" % \
            len(analysis.metadata.invocations)
    analysis_duration = round(time.monotonic() - analysis_start, 2)
    shared = {"anchors": {"invocations": 1, "duration_s": anchor_duration,
                           "status": anchor.status},
              "type_analysis": {"invocations": len(analysis.metadata.invocations),
                                 "duration_s": analysis_duration,
                                 "source_count": analysis.metadata.requested_source_count,
                                 "diagnostic_count": analysis.metadata.diagnostic_count}}
    reports = []
    for index, ((gear, candidate), (candidate_args, paths, errors)) in enumerate(zip(specs, prepared)):
        if errors:
            report = _setup_report(candidate_args, paths, errors, anchor)
        elif gear not in plans:
            report = _setup_report(candidate_args, paths,
                                   [plan_errors.get(gear, "could not form novelty plan")], anchor)
        else:
            report = run_gates.run_candidate(
                paths, candidate_args, shared_anchors=anchor, shared_analysis=analysis,
                novelty_plan=plans[gear], shared_owner_gear=next(iter(plans), specs[0][0]),
                owns_shared_duration=gear == next(iter(plans), specs[0][0]))
        report_path = os.path.join(report_dir, gear + ".gates.json")
        _write(report_path, report)
        reports.append({"gear": gear, "candidate": paths.candidate,
                        "report": _root_relative(report_path, root),
                        "verdict": report["verdict"], "exit_code": report["exit_code"]})
    aggregate = _aggregate(root, reports, shared, started, runner_started)
    _write(json_out, aggregate)
    if args.format == "json":
        print(json.dumps(aggregate))
    else:
        for entry in reports:
            print("%-20s %s" % (entry["gear"], entry["verdict"]))
        print("aggregate: %s" % aggregate["verdict"])
    return aggregate["exit_code"]


if __name__ == "__main__":
    sys.exit(main())
