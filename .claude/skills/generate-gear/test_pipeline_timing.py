"""Fixture tests for the pipeline timing event owner."""
import importlib.util
import json
import os
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace


MODULE_PATH = Path(__file__).with_name("pipeline_timing.py")
SPEC = importlib.util.spec_from_file_location("pipeline_timing_tests_module", MODULE_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class FakeClock:
    def __init__(self, epoch=100.0):
        self.value = float(epoch)

    def epoch(self):
        return self.value

    def monotonic(self):
        return self.value

    def advance(self, seconds):
        self.value += seconds


def gate_report(path, *, first_pass=True, failed=False, shared=False):
    rows = [
        {"key": "parse", "status": "fail" if failed else "pass", "duration_s": 1.0},
        {"key": "pyright", "status": "pass", "duration_s": 3.0},
        {"key": "novel_types", "status": "pass", "advisory": True, "duration_s": 3.0},
    ]
    report = {
        "schema": 1,
        "verdict": "fail" if failed else "pass",
        "exit_code": 1 if failed else 0,
        "counts": {"advisory_findings": 0},
        "gates": rows,
        "timing": {
            "schema": 1,
            "wall_time_s": 4.0,
            "analysis_duration_s": 3.0,
            "analysis_shared": shared,
            "first_pass_eligible": first_pass,
            "gate_policy": {"mode": "full", "first_pass_eligible": first_pass},
        },
    }
    with open(path, "w", encoding="utf-8") as handle:
        json.dump(report, handle)


class PipelineTimingTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.run_dir = self.root / "run"
        self.clock = FakeClock()
        (self.root / "input.md").write_text("input\n", encoding="utf-8")
        MODULE.start_run(self.run_dir, "spurgear", "emit", root=self.root,
                         inputs=["input.md"], clock=self.clock)

    def test_start_records_digest_and_rejects_reuse(self):
        run = json.loads((self.run_dir / "run.json").read_text(encoding="utf-8"))
        self.assertEqual(run["inputs"][0]["size_bytes"], 6)
        self.assertEqual(len(run["inputs"][0]["sha256"]), 64)
        with self.assertRaises(MODULE.TimingError):
            MODULE.start_run(self.run_dir, "spurgear", "emit", clock=self.clock)

    def _complete_run(self, report_first_pass=True, failed=False, triage="complete"):
        MODULE.record_event(self.run_dir, "drafting", "start", round=1, clock=self.clock)
        self.clock.advance(10)
        MODULE.record_event(self.run_dir, "drafting", "finish", round=1, clock=self.clock)
        MODULE.record_event(self.run_dir, "validation", "start", round=1, clock=self.clock)
        report_path = self.root / "gates.json"
        gate_report(report_path, first_pass=report_first_pass, failed=failed)
        MODULE.gate_import(self.run_dir, 1, report_path)
        self.clock.advance(4)
        MODULE.record_event(self.run_dir, "validation", "finish", round=1, clock=self.clock)
        MODULE.record_event(self.run_dir, "overall", "finish", round=1, clock=self.clock)
        if triage is not None:
            MODULE.record_event(self.run_dir, "validation", "record", round=99,
                                clock=self.clock, event_name="advisory_triage",
                                metadata={"advisory_triage": triage})

    def test_complete_first_pass_and_unknown_tokens(self):
        self._complete_run()
        summary = MODULE.summarize(self.run_dir)
        self.assertTrue(summary["complete"])
        self.assertTrue(summary["first_pass"])
        self.assertEqual(summary["drafting_time_s"], 10.0)
        self.assertEqual(summary["validation_time_s"], 4.0)
        self.assertEqual(summary["token_counts"], {"input": None, "output": None})

    def test_overlapping_drafting_intervals_are_unioned(self):
        MODULE.record_event(self.run_dir, "drafting", "start", round=1, clock=self.clock)
        self.clock.advance(10)
        MODULE.record_event(self.run_dir, "drafting", "finish", round=1, clock=self.clock)
        self.clock.value = 105
        MODULE.record_event(self.run_dir, "drafting", "start", round=2, clock=self.clock)
        self.clock.value = 115
        MODULE.record_event(self.run_dir, "drafting", "finish", round=2, clock=self.clock)
        self.assertEqual(MODULE._union(((100, 110), (105, 115))), 15.0)

    def test_zero_duration_interval_is_observed(self):
        MODULE.record_event(self.run_dir, "drafting", "start", clock=self.clock)
        MODULE.record_event(self.run_dir, "drafting", "finish", clock=self.clock)
        self.assertEqual(MODULE.summarize(self.run_dir)["drafting_time_s"], 0.0)

    def test_gate_policy_requires_full_runner_and_allows_real_inapplicable_skips(self):
        args = SimpleNamespace(only=None, fail_fast=False, no_advisory=False,
                               require_contract=False, skip_missing_steps=True,
                               gate_novel_types=False)
        results = [SimpleNamespace(key=key, status="pass", skip_reason=None)
                   for key in ("parse", "input_read", "anchors", "api_calls", "pyright")]
        results.extend([
            SimpleNamespace(key="contract", status="skip", skip_reason="no manifest at spec/x/contract.json"),
            SimpleNamespace(key="step_calls", status="skip", skip_reason="no compiled step list at spec/x/steps.md"),
        ])
        policy = MODULE.gate_policy_for_run(args, results)
        self.assertTrue(policy["first_pass_eligible"])
        args.no_advisory = True
        self.assertFalse(MODULE.gate_policy_for_run(args, results)["first_pass_eligible"])

    def test_gate_policy_rejects_selective_and_unexplained_skips(self):
        args = SimpleNamespace(only=["pyright"], fail_fast=False, no_advisory=False,
                               require_contract=False, skip_missing_steps=False,
                               gate_novel_types=False)
        results = [SimpleNamespace(key="pyright", status="pass", skip_reason=None)]
        self.assertFalse(MODULE.gate_policy_for_run(args, results)["first_pass_eligible"])
        args.only = None
        results.append(SimpleNamespace(key="contract", status="skip", skip_reason="optional"))
        self.assertFalse(MODULE.gate_policy_for_run(args, results)["first_pass_eligible"])

    def test_compile_policy_requires_complete_proof_and_full_scope(self):
        args = SimpleNamespace(only=None)
        results = [SimpleNamespace(key="proof", status="pass", skip_reason=None),
                   SimpleNamespace(key="compile", status="pass", skip_reason=None)]
        metadata = {"iteration_mode": False, "proof_is_complete": True, "effective_proof_scope": "full"}
        self.assertTrue(MODULE.compile_policy_for_run(args, results, metadata)["first_pass_eligible"])
        metadata["proof_is_complete"] = False
        self.assertFalse(MODULE.compile_policy_for_run(args, results, metadata)["first_pass_eligible"])

    def test_missing_finish_and_triage_are_incomplete(self):
        MODULE.record_event(self.run_dir, "drafting", "start", round=1, clock=self.clock)
        report_path = self.root / "gates.json"
        gate_report(report_path)
        MODULE.gate_import(self.run_dir, 1, report_path)
        summary = MODULE.summarize(self.run_dir)
        self.assertFalse(summary["complete"])
        self.assertIsNone(summary["advisory_triage"]["complete"])
        self.assertIn("missing finish", " ".join(summary["issues"]))

    def test_malformed_event_is_reported_as_incomplete(self):
        event_dir = self.run_dir / MODULE.EVENT_DIR
        (event_dir / "event-999999-bad.json").write_text("{bad", encoding="utf-8")
        summary = MODULE.summarize(self.run_dir)
        self.assertFalse(summary["complete"])
        self.assertTrue(any("bad" in issue["file"] for issue in summary["issues"]
                            if isinstance(issue, dict)))

    def test_malformed_event_shape_does_not_crash_intervals(self):
        event_dir = self.run_dir / MODULE.EVENT_DIR
        malformed = {
            "schema": 1,
            "run_id": json.loads((self.run_dir / "run.json").read_text())["run_id"],
            "gear": "spurgear",
            "stage": "emit",
            "sequence": "one",
            "round": 1,
            "phase": "drafting",
            "action": "start",
            "event": "drafting.start",
            "timestamp_utc": "now",
            "timestamp_epoch_s": 100.0,
            "duration_s": float("nan"),
            "metadata": [],
        }
        (event_dir / "event-999998-shape.json").write_text(json.dumps(malformed), encoding="utf-8")
        summary = MODULE.summarize(self.run_dir)
        self.assertFalse(summary["complete"])
        self.assertTrue(any("shape" in issue["file"] for issue in summary["issues"]
                            if isinstance(issue, dict)))

    def test_interrupted_run_cannot_claim_first_pass(self):
        self._complete_run()
        for path in (self.run_dir / MODULE.EVENT_DIR).glob("event-*.json"):
            event = json.loads(path.read_text(encoding="utf-8"))
            if event.get("phase") == "overall" and event.get("action") == "finish":
                path.unlink()
                break
        summary = MODULE.summarize(self.run_dir)
        self.assertFalse(summary["first_pass"])
        self.assertFalse(summary["complete"])

    def test_missing_draft_boundary_cannot_be_complete(self):
        MODULE.record_event(self.run_dir, "validation", "start", clock=self.clock)
        MODULE.record_event(self.run_dir, "validation", "finish", clock=self.clock, duration_s=1)
        report_path = self.root / "gates.json"
        gate_report(report_path)
        MODULE.gate_import(self.run_dir, 1, report_path)
        MODULE.record_event(self.run_dir, "overall", "finish", clock=self.clock, duration_s=1)
        MODULE.record_event(self.run_dir, "validation", "record", clock=self.clock,
                            event_name="advisory_triage", metadata={"advisory_triage": "complete"})
        summary = MODULE.summarize(self.run_dir)
        self.assertFalse(summary["complete"])
        self.assertEqual(summary["completed_rounds"], 0)
        self.assertIn("drafting boundary is missing", summary["issues"])

    def test_duplicate_gate_import_does_not_double_count(self):
        report_path = self.root / "gates.json"
        gate_report(report_path, shared=True)
        first = MODULE.gate_import(self.run_dir, 1, report_path)
        second = MODULE.gate_import(self.run_dir, 1, report_path)
        self.assertFalse(first.get("duplicate", False))
        self.assertTrue(second["duplicate"])
        imported = [event for event in MODULE.load_events(self.run_dir)[1]
                    if event.get("event") == "gate_import"]
        self.assertEqual(len(imported), 1)
        self.assertEqual(imported[0]["metadata"]["gate_duration_s"], 4.0)

    def test_same_evidence_on_later_round_is_explicit(self):
        report_path = self.root / "gates.json"
        gate_report(report_path)
        MODULE.gate_import(self.run_dir, 1, report_path)
        later = MODULE.gate_import(self.run_dir, 2, report_path)
        self.assertEqual(later["metadata"]["source_reused_from_round"], 1)

    def test_failed_round_cannot_be_first_pass(self):
        self._complete_run(failed=True)
        summary = MODULE.summarize(self.run_dir)
        self.assertFalse(summary["first_pass"])
        self.assertEqual(summary["gate_failures"], 1)

    def test_advisory_triage_state_is_separate(self):
        self._complete_run(triage="incomplete")
        summary = MODULE.summarize(self.run_dir)
        self.assertFalse(summary["advisory_triage"]["complete"])
        self.assertTrue(summary["first_pass"])
        self.assertFalse(summary["complete"])

    def test_pending_advisory_triage_does_not_change_first_pass(self):
        self._complete_run(triage=None)
        summary = MODULE.summarize(self.run_dir)
        self.assertTrue(summary["first_pass"])
        self.assertIsNone(summary["advisory_triage"]["complete"])
        self.assertFalse(summary["complete"])

    def test_negative_intervals_are_rejected(self):
        MODULE.record_event(self.run_dir, "drafting", "start", round=1, clock=self.clock)
        self.clock.value = 50
        with self.assertRaises(MODULE.TimingError):
            MODULE.record_event(self.run_dir, "drafting", "finish", round=1, clock=self.clock)


if __name__ == "__main__":
    unittest.main()
