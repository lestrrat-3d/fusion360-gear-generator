#!/usr/bin/env python3
"""Tests for the complete retry-feedback representation."""
import contextlib
import importlib.util
import io
import json
import tempfile
import unittest
from pathlib import Path
from unittest import mock


HERE = Path(__file__).resolve().parent


def load_module(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


RENDERER = load_module("render_retry_feedback", HERE / "render_retry_feedback.py")
PROMPT_RENDERER = load_module("render_prompt_for_feedback", HERE / "render_prompt.py")
PIPELINE_TIMING = load_module("pipeline_timing_for_feedback", HERE / "pipeline_timing.py")


def compile_report():
    """Return the complete top-level shape emitted by run_compile_gates.py."""
    return {
        "schema": 1,
        "gear": "fixturegear",
        "root": "/fixture",
        "verdict": "fail",
        "exit_code": 1,
        "counts": {"pass": 3, "fail": 1, "skip": 0, "error": 0},
        "stages": [{"key": "proof", "status": "fail", "stdout": "proof failed\n"}],
        "handoff": {"ready_for_emit": False, "reasons": ["proof_incomplete"]},
        "timing": {"schema": 1, "wall_time_s": 1.25},
    }


def emit_report():
    """Return the complete top-level shape emitted by run_gates.py."""
    return {
        "schema": 1,
        "gear": "fixturegear",
        "candidate": "/fixture/.tmp/fixturegear.generated.py",
        "root": "/fixture",
        "verdict": "fail",
        "exit_code": 1,
        "counts": {"pass": 6, "fail": 1, "skip": 0, "error": 0, "advisory_findings": 0},
        "gates": [{"key": "pyright", "status": "fail", "stderr": "type error\n"}],
        "classification": [{"gate": "pyright", "fault": "emit"}],
        "metadata": {},
        "timing": {"schema": 1, "wall_time_s": 0.5},
    }


class FeedbackCase(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.root = Path(self.temporary.name)
        self.report_path = self.root / "gates.json"
        self.output_path = self.root / "feedback.txt"

    def tearDown(self):
        self.temporary.cleanup()

    def write_report(self, report):
        self.report_path.write_text(json.dumps(report, ensure_ascii=False), encoding="utf-8")

    def run_main(self):
        stdout = io.StringIO()
        stderr = io.StringIO()
        with contextlib.redirect_stdout(stdout), contextlib.redirect_stderr(stderr):
            code = RENDERER.main([
                "--report", str(self.report_path), "--out", str(self.output_path)
            ])
        return code, stdout.getvalue(), stderr.getvalue()

    def render(self, report):
        self.write_report(report)
        code, stdout, stderr = self.run_main()
        self.assertEqual(code, 0, stderr)
        self.assertEqual(stdout, "")
        return self.output_path.read_text(encoding="utf-8")

    def assert_rejected_unchanged(self, raw):
        original = "existing feedback stays\n"
        self.report_path.write_bytes(raw)
        self.output_path.write_text(original, encoding="utf-8")
        code, stdout, stderr = self.run_main()
        self.assertEqual(code, 2)
        self.assertEqual(stdout, "")
        self.assertIn("render_retry_feedback:", stderr)
        self.assertEqual(self.output_path.read_text(encoding="utf-8"), original)


class RetryFeedbackTest(FeedbackCase):
    def test_lossless_report_round_trip(self):
        for report in (compile_report(), emit_report()):
            with self.subTest(shape="stages" if "stages" in report else "gates"):
                rendered = self.render(report)
                self.assertEqual(json.loads(rendered), report)
                self.assertEqual(
                    rendered,
                    json.dumps(report, sort_keys=True, ensure_ascii=False, indent=2) + "\n",
                )

    def test_preserves_unknown_fields(self):
        report = emit_report()
        report["future_runner_metadata"] = {
            "policy": "keep all fields", "numbers": [0, 1, 2.5], "optional": None
        }
        self.assertEqual(json.loads(self.render(report)), report)

    def test_preserves_nested_diagnostics(self):
        report = compile_report()
        diagnostic = "```json\n{\"brace\": \"}\"}\n```\n歯車 café 😀 {{gear}}\n"
        report["stages"][0]["stdout"] = diagnostic
        report["stages"][0]["details"] = {"nested": {"stderr": diagnostic}}
        rendered = self.render(report)
        self.assertIn("歯車 café 😀", rendered)
        decoded = json.loads(rendered)
        self.assertEqual(decoded["stages"][0]["stdout"], diagnostic)
        self.assertEqual(decoded["stages"][0]["details"]["nested"]["stderr"], diagnostic)

    def test_rejects_duplicate_keys(self):
        self.assert_rejected_unchanged(b'{"schema":1,"gates":[],"schema":1}\n')
        self.assert_rejected_unchanged(
            b'{"schema":1,"gates":[{"status":"fail","status":"pass"}]}\n'
        )

    def test_rejects_ambiguous_runner_shape(self):
        self.assert_rejected_unchanged(b'{"schema":1,"stages":[],"gates":[]}\n')

    def test_rejects_boolean_schema(self):
        self.assert_rejected_unchanged(b'{"schema":true,"gates":[]}\n')

    def test_rejects_wrong_shapes_and_malformed_inputs(self):
        cases = {
            "invalid UTF-8": b'\xff{"schema":1,"gates":[]}',
            "invalid JSON": b'{"schema":1,"gates":[]',
            "non-object root": b'[1, 2, 3]',
            "missing runner list": b'{"schema":1}',
            "wrong runner type": b'{"schema":1,"stages":{}}',
            "unsupported schema": b'{"schema":2,"gates":[]}',
            "float schema": b'{"schema":1.0,"gates":[]}',
            "nonstandard number": b'{"schema":1,"gates":[],"elapsed":NaN}',
            "overflow number": b'{"schema":1,"gates":[],"elapsed":1e400}',
        }
        for name, raw in cases.items():
            with self.subTest(case=name):
                self.assert_rejected_unchanged(raw)

    def test_atomic_write_error_preserves_output(self):
        self.write_report(emit_report())
        original = "existing feedback stays\n"
        self.output_path.write_text(original, encoding="utf-8")
        with mock.patch.object(RENDERER.os, "replace", side_effect=OSError("fixture failure")):
            code, stdout, stderr = self.run_main()
        self.assertEqual(code, 2)
        self.assertEqual(stdout, "")
        self.assertIn("fixture failure", stderr)
        self.assertEqual(self.output_path.read_text(encoding="utf-8"), original)
        leftovers = list(self.root.glob(".feedback.txt-*.tmp"))
        self.assertEqual(leftovers, [])

    def test_prompt_preserves_feedback(self):
        feedback = self.render(emit_report())
        skills_root = self.root / "skills"
        skill_dir = skills_root / "emit-gear"
        skill_dir.mkdir(parents=True)
        (skill_dir / "prompt.md").write_text("draft {{gear}}\n", encoding="utf-8")
        code, output, error = self._render_prompt(skills_root, feedback)
        self.assertEqual(code, 0, error)
        self.assertEqual(output.count(feedback), 1)
        self.assertIn(PROMPT_RENDERER.BEGIN_MARKER + "\n" + feedback, output)

    def _render_prompt(self, skills_root, feedback):
        feedback_path = self.root / "feedback-for-prompt.txt"
        feedback_path.write_text(feedback, encoding="utf-8")
        stdout = io.StringIO()
        stderr = io.StringIO()
        with contextlib.redirect_stdout(stdout), contextlib.redirect_stderr(stderr):
            code = PROMPT_RENDERER.main(
                [
                    "render_prompt.py", "emit-gear", "fixturegear",
                    "--failure-file", str(feedback_path),
                ],
                skills_root=skills_root,
            )
        return code, stdout.getvalue(), stderr.getvalue()

    def test_no_double_import(self):
        pilot = (HERE / "pipeline-timing-pilot.md").read_text(encoding="utf-8")
        self.assertEqual(pilot.count('--file "$TIMING/gates-round-1.json"'), 1)
        self.assertEqual(pilot.count('"$TIMING/compile-gates-round-<round>.json"'), 3)
        self.assertEqual(
            pilot.count('"$TIMING/final-compile-gates-round-<round>.json"'), 3
        )
        for line in pilot.splitlines():
            if "import-gates" in line and "--file" in line:
                self.assertNotIn("feedback", line)

        timing_dir = self.root / "timing"
        PIPELINE_TIMING.start_run(
            timing_dir, "fixturegear", "emit", root=self.root, git_commit="fixture"
        )
        raw_paths = []
        for round_number in (1, 2):
            report = emit_report()
            report["round_fixture"] = round_number
            raw_path = timing_dir / "gates-round-{}.json".format(round_number)
            raw_path.write_text(json.dumps(report), encoding="utf-8")
            raw_paths.append(raw_path)
            code = RENDERER.main([
                "--report", str(raw_path), "--out", str(timing_dir / "gates-feedback.txt")
            ])
            self.assertEqual(code, 0)
            PIPELINE_TIMING.gate_import(timing_dir, round_number, raw_path)

        self.assertTrue(all(path.is_file() for path in raw_paths))
        self.assertEqual(
            json.loads((timing_dir / "gates-feedback.txt").read_text(encoding="utf-8")),
            dict(emit_report(), round_fixture=2),
        )
        _metadata, events, issues = PIPELINE_TIMING.load_events(timing_dir)
        self.assertEqual(issues, [])
        imports = [event for event in events if event["event"] == "gate_import"]
        self.assertEqual(len(imports), 2)
        self.assertEqual(
            {event["metadata"]["source_name"] for event in imports},
            {"gates-round-1.json", "gates-round-2.json"},
        )


if __name__ == "__main__":
    unittest.main()
