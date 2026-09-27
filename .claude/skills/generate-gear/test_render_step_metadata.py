#!/usr/bin/env python3
"""Regression tests for the atomic step-citation renderer."""
import contextlib
import io
import json
import os
import sys
import tempfile
import unittest
from pathlib import Path


HERE = Path(__file__).parent
sys.path.insert(0, str(HERE))
import render_step_metadata as RENDERER  # noqa: E402
from test_step_metadata import call_declaration, version_two  # noqa: E402


def metadata(first=2, last=2, raw=None):
    if raw is not None:
        encoded = raw
    else:
        value = {
            'schema': 1,
            'citations': [{
                'path': 'spec/fixturegear/instructions.md',
                'first': first,
                'last': last,
            }],
        }
        encoded = json.dumps(value, sort_keys=True, ensure_ascii=False, indent=2)
    return '<!-- step-meta\n%s\n-->' % encoded


def step(step_id='1', first=2, last=2, from_line=None, raw=None):
    rendered_from = '' if from_line is None else '\n\n%s' % from_line
    return ('## %s `[PROSE]` Fixture step\n\n'
            '%s%s\n\n'
            'Keep this unrelated instruction.\n'
            % (step_id, metadata(first, last, raw=raw), rendered_from))


def document(steps, marker='<!-- step-metadata: 1 -->'):
    return ('# Fixture steps\n\n%s\n\n## Provenance\n\n'
            '## Compilation contract\n\n`````json\n{}\n`````\n\n%s'
            % (marker, steps))


class RenderStepMetadataTest(unittest.TestCase):
    def test_version_two_preserves_calls_and_is_idempotent(self):
        call = call_declaration(condition='When enabled.')
        original = version_two([call]).replace('**From:** `spec/fixturegear/instructions.md` L2.', '')
        target = self.repo(original)
        code, _, err = self.run_renderer(target)
        self.assertEqual(code, 0, err)
        rendered = target.read_text()
        self.assertIn('**From:** `spec/fixturegear/instructions.md` L2.', rendered)
        self.assertEqual(RENDERER.step_metadata.file_calls(rendered), [call])
        code, _, err = self.run_renderer(target)
        self.assertEqual(code, 0, err)
        self.assertEqual(target.read_text(), rendered)

    def test_version_two_missing_call_is_atomic(self):
        text = version_two().replace('Call `tools.addWidget(item)`.', 'No call.')
        self.assert_content_failure_unchanged(text, 'S1: call declaration')

    def test_version_two_preamble_call_is_atomic(self):
        self.assert_content_failure_unchanged(
            version_two(preamble='`tools.addWidget(item)`'), 'preamble contains a call-shaped span')
    def repo(self, text):
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        root = Path(directory.name)
        source = root / 'spec' / 'fixturegear' / 'instructions.md'
        source.parent.mkdir(parents=True)
        source.write_text('one\ntwo\nthree\n', encoding='utf-8')
        target = root / 'draft.md'
        target.write_text(text, encoding='utf-8')
        prior = os.getcwd()
        os.chdir(root)
        self.addCleanup(os.chdir, prior)
        return target

    def run_renderer(self, target, argv=None):
        arguments = argv or ['render_step_metadata.py', 'fixturegear', '--write', str(target)]
        out, err = io.StringIO(), io.StringIO()
        with contextlib.redirect_stdout(out), contextlib.redirect_stderr(err):
            code = RENDERER.main(arguments)
        return code, out.getvalue(), err.getvalue()

    def test_single_line(self):
        target = self.repo(document(step()))

        code, out, err = self.run_renderer(target)

        self.assertEqual(code, 0, err)
        self.assertIn('rendered', out)
        self.assertIn('**From:** `spec/fixturegear/instructions.md` L2.\n', target.read_text())

    def test_range(self):
        target = self.repo(document(step(first=1, last=3)))

        code, _, err = self.run_renderer(target)

        self.assertEqual(code, 0, err)
        self.assertIn('**From:** `spec/fixturegear/instructions.md` L1–3.', target.read_text())

    def assert_content_failure_unchanged(self, text, diagnostic):
        target = self.repo(text)
        before = target.read_bytes()

        code, out, err = self.run_renderer(target)

        self.assertEqual(code, 1, out)
        self.assertIn(diagnostic, err)
        self.assertEqual(target.read_bytes(), before)

    def test_out_of_bounds(self):
        self.assert_content_failure_unchanged(
            document(step(last=4)), '1: cites spec/fixturegear/instructions.md')

    def test_reversed_range(self):
        self.assert_content_failure_unchanged(document(step(first=3, last=2)), 'first line is after')

    def test_boolean_line(self):
        raw = ('{"schema": 1, "citations": [{"path": '
               '"spec/fixturegear/instructions.md", "first": true, "last": 2}]}')
        self.assert_content_failure_unchanged(document(step(raw=raw)), 'first must be an integer')

    def test_unknown_schema(self):
        self.assert_content_failure_unchanged(
            document(step(), marker='<!-- step-metadata: 99 -->'),
            'unsupported step-metadata version 99')

    def test_malformed_marker_has_no_legacy_fallback(self):
        self.assert_content_failure_unchanged(
            document(step(), marker='prefix <!-- step-metadata: 1 -->'), 'malformed')

    def test_duplicate_json_key(self):
        raw = ('{"schema": 1, "citations": [{"path": '
               '"spec/fixturegear/instructions.md", "first": 1, "first": 2, "last": 2}]}')
        self.assert_content_failure_unchanged(document(step(raw=raw)), 'repeats key')

    def test_duplicate_step_payload(self):
        duplicated = step().replace(metadata(), '%s\n\n%s' % (metadata(), metadata()))
        self.assert_content_failure_unchanged(document(duplicated), 'repeated step-meta payloads')

    def test_from_drift(self):
        target = self.repo(document(step(from_line=(
            '**From:** `spec/fixturegear/instructions.md` L1.'))))

        code, _, err = self.run_renderer(target)

        self.assertEqual(code, 0, err)
        self.assertNotIn('instructions.md` L1.', target.read_text())
        self.assertIn('instructions.md` L2.', target.read_text())

    def test_second_step_invalid_is_atomic(self):
        text = document(step('1') + '\n' + step('2', last=4))
        self.assert_content_failure_unchanged(text, '2: cites spec/fixturegear/instructions.md')

    def test_legacy_checker_compatibility(self):
        self.assert_content_failure_unchanged(
            document(step(), marker=''), 'legacy step list has no step-metadata marker')

    def test_repeated_render(self):
        target = self.repo(document(step()))
        first_code, _, first_err = self.run_renderer(target)
        self.assertEqual(first_code, 0, first_err)
        first = target.read_bytes()

        second_code, out, second_err = self.run_renderer(target)

        self.assertEqual(second_code, 0, second_err)
        self.assertIn('already current', out)
        self.assertEqual(target.read_bytes(), first)

    def test_adjacent_repeated_from_blocks_are_rejected(self):
        from_lines = ('**From:** `spec/fixturegear/instructions.md` L1.\n'
                      '**From:** `spec/fixturegear/instructions.md` L2.')
        self.assert_content_failure_unchanged(
            document(step(from_line=from_lines)), 'repeated **From:** blocks')

    def test_usage_and_missing_inputs_exit_2(self):
        target = self.repo(document(step()))
        code, _, err = self.run_renderer(target, ['render_step_metadata.py'])
        self.assertEqual(code, 2)
        self.assertIn('usage:', err)

        missing = target.with_name('missing.md')
        code, _, err = self.run_renderer(missing)
        self.assertEqual(code, 2)
        self.assertIn(str(missing), err)


if __name__ == '__main__':
    unittest.main()
