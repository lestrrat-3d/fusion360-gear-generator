#!/usr/bin/env python3
"""Regression tests for the cross-gear gate runner."""
import importlib.util
import json
import os
import tempfile
import unittest
from types import SimpleNamespace
from unittest import mock

HERE = os.path.dirname(os.path.abspath(__file__))
spec = importlib.util.spec_from_file_location("run_ci_gates", os.path.join(HERE, "run_ci_gates.py"))
RUNNER = importlib.util.module_from_spec(spec)
spec.loader.exec_module(RUNNER)


class FakePyright:
    class AnalysisMetadata:
        def __init__(self, root):
            self.root = root
            self.invocations = [object()]
            self.requested_source_count = 3
            self.diagnostic_count = 2
            self.duration_s = 1.25

    class AnalysisResult:
        def __init__(self, diagnostics, metadata, setup_error=None):
            self.diagnostics = diagnostics
            self.metadata = metadata
            self.setup_error = setup_error

    def __init__(self, root):
        self.root = root
        self.calls = []

    def analyze_paths(self, paths, **_kwargs):
        self.calls.append(list(paths))
        return self.AnalysisResult({os.path.abspath(path): [] for path in paths},
                                   self.AnalysisMetadata(self.root))


def report(gear, paths, *, verdict="pass", durations=(0.1, 1.25, 0.0)):
    gates = []
    for key in RUNNER.run_gates.GATE_ORDER:
        duration = None
        note = None
        if key == "anchors":
            duration, note = durations[0], "shared anchor duration"
        elif key == "pyright":
            duration, note = durations[1], "shared analysis duration"
        elif key == "novel_types":
            duration, note = durations[2], "classification uses shared analysis"
        gates.append({"key": key, "title": RUNNER.run_gates.GATE_TITLES[key], "status": "pass",
                      "advisory": key == "novel_types", "exit_code": 0,
                      "duration_s": duration, "command": [], "headline": "ok", "stdout": "",
                      "stderr": "", "skip_reason": None, "fault": None, "timing_note": note})
    return {"schema": 1, "gear": gear, "candidate": paths.candidate, "root": paths.root,
            "verdict": verdict, "exit_code": 0 if verdict == "pass" else 1,
            "counts": {"pass": 8, "fail": 0, "skip": 0, "error": 0,
                       "advisory_findings": 0}, "gates": gates, "classification": [],
            "metadata": {}, "timing": {}}


class BatchRunnerTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.root = self.tmp.name
        os.makedirs(os.path.join(self.root, "lib", "geargen"))
        os.makedirs(os.path.join(self.root, "spec", "one"))
        os.makedirs(os.path.join(self.root, "spec", "two"))
        os.makedirs(os.path.join(self.root, "reports"))
        for gear in ("one", "two"):
            with open(os.path.join(self.root, "spec", gear, "steps.md"), "w") as handle:
                handle.write("steps\n")
            with open(os.path.join(self.root, "spec", gear, "contract.json"), "w") as handle:
                handle.write("{}\n")
        self.candidates = {}
        for gear in ("one", "two"):
            path = os.path.join(self.root, gear + ".py")
            with open(path, "w") as handle:
                handle.write("value = 1\n")
            self.candidates[gear] = path

    def tearDown(self):
        self.tmp.cleanup()

    def invoke(self, extra, *, prepare=None, candidate_reports=None):
        fake_pyright = FakePyright(self.root)
        plans = {gear: SimpleNamespace(candidate=path, references=tuple())
                 for gear, path in self.candidates.items()}
        paths = {gear: SimpleNamespace(root=self.root, candidate=os.path.basename(path),
                                       steps="spec/%s/steps.md" % gear,
                                       contract="spec/%s/contract.json" % gear)
                 for gear, path in self.candidates.items()}
        args = ["--root", self.root, "--gear", "one=" + self.candidates["one"],
                "--gear", "two=" + self.candidates["two"], "--json-out",
                os.path.join(self.root, "aggregate.json"), "--report-dir",
                os.path.join(self.root, "reports")] + list(extra)

        def fake_prepare(gear, _candidate, _options):
            return SimpleNamespace(gear=gear, candidate=_candidate, root=self.root), paths[gear], \
                (prepare or {}).get(gear, [])

        def fake_candidate(path, candidate_args, **kwargs):
            value = (candidate_reports or {}).get(candidate_args.gear,
                                                   report(candidate_args.gear, path))
            if candidate_args.gear == "two":
                for row in value["gates"]:
                    if row["key"] in ("anchors", "pyright"):
                        row["duration_s"] = 0.0
            value["metadata"] = {"analysis_invocations": 1 if candidate_args.gear == "one" else 0}
            return value

        anchor = RUNNER.run_gates.GateResult("anchors", "Anchors", "pass", False, 0, 0.1,
                                             [], "", "", None, None)
        with mock.patch.object(RUNNER.run_gates, "prepare_run", side_effect=fake_prepare), \
             mock.patch.object(RUNNER.run_gates, "run_script_gate", return_value=anchor), \
             mock.patch.object(RUNNER.run_gates, "run_candidate", side_effect=fake_candidate), \
             mock.patch.object(RUNNER, "_module", side_effect=[
                 SimpleNamespace(plan_evaluation=lambda _root, candidate: plans[
                     os.path.basename(candidate).split(".")[0]]), fake_pyright]):
            code = RUNNER.main(args)
        with open(os.path.join(self.root, "aggregate.json"), encoding="utf-8") as handle:
            aggregate = json.load(handle)
        return code, aggregate, fake_pyright

    def test_shared_analysis_and_complete_reports(self):
        code, aggregate, fake_pyright = self.invoke([])
        self.assertEqual(code, 0)
        self.assertEqual(len(fake_pyright.calls), 1)
        self.assertEqual(aggregate["shared"]["anchors"]["invocations"], 1)
        self.assertEqual(aggregate["shared"]["type_analysis"]["invocations"], 1)
        self.assertEqual([item["gear"] for item in aggregate["gears"]], ["one", "two"])
        for gear in ("one", "two"):
            with open(os.path.join(self.root, "reports", gear + ".gates.json")) as handle:
                data = json.load(handle)
                self.assertEqual([row["key"] for row in data["gates"]],
                                 list(RUNNER.run_gates.GATE_ORDER))
                shared_rows = {row["key"]: row for row in data["gates"]}
                self.assertEqual(shared_rows["anchors"]["duration_s"], 0.1 if gear == "one" else 0.0)
                self.assertEqual(data["metadata"]["analysis_invocations"], 1 if gear == "one" else 0)

    def test_candidate_setup_error_isolated(self):
        code, aggregate, fake_pyright = self.invoke([], prepare={"one": ["candidate missing"]})
        self.assertEqual(code, 2)
        self.assertEqual(aggregate["gears"][0]["verdict"], "setup_error")
        self.assertEqual(aggregate["gears"][1]["verdict"], "pass")
        self.assertEqual(len(fake_pyright.calls), 1)

    def test_shared_failure_fans_out_and_content_failure_precedes_setup(self):
        fake = FakePyright(self.root)
        fake.analyze_paths = lambda *_paths, **_kwargs: fake.AnalysisResult(
            {}, fake.AnalysisMetadata(self.root), "tool unavailable")
        candidate_reports = {"one": report("one", SimpleNamespace(root=self.root,
                    candidate="one.py"), verdict="fail")}
        with mock.patch.object(RUNNER, "_module", side_effect=[
                SimpleNamespace(plan_evaluation=lambda _root, candidate: SimpleNamespace(
                    candidate=candidate, references=tuple())), fake]):
            code, aggregate, _ = self.invoke([], candidate_reports=candidate_reports)
        self.assertEqual(code, 1)
        self.assertEqual(aggregate["counts"], {"pass": 1, "fail": 1, "error": 0})

    def test_cli_rejects_duplicate_before_execution(self):
        args = ["--root", self.root, "--gear", "one=x", "--gear", "one=y",
                "--json-out", os.path.join(self.root, "a.json"), "--report-dir",
                os.path.join(self.root, "reports")]
        with mock.patch.object(RUNNER.run_gates, "run_script_gate") as anchor:
            self.assertEqual(RUNNER.main(args), 2)
            anchor.assert_not_called()

    def test_novelty_plan_keeps_self_copy_and_symlink_exclusions(self):
        novel = RUNNER._module("check_novel_types")
        reference = os.path.join(self.root, "lib", "geargen")
        shipped = os.path.join(reference, "shipped.py")
        with open(shipped, "w") as handle:
            handle.write("value = 1\n")
        candidate = os.path.join(self.root, "candidate.py")
        with open(candidate, "w") as handle:
            handle.write("value = 1\n")
        os.symlink(shipped, os.path.join(reference, "alias.py"))
        with open(os.path.join(reference, "_private.py"), "w") as handle:
            handle.write("value = 2\n")
        with open(os.path.join(reference, "notes.txt"), "w") as handle:
            handle.write("ignored\n")
        self.assertEqual(novel.reference_gears(reference, candidate), [])
        with self.assertRaises(novel.AnalysisSetupError):
            novel.plan_evaluation(reference, candidate)


if __name__ == "__main__":
    unittest.main()
