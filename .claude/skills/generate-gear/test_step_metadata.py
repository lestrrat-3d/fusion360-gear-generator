#!/usr/bin/env python3
"""Regression tests for compiled-step metadata parsing and rendering."""
import json
import os
import sys
import tempfile
import unittest
from pathlib import Path
from unittest import mock


HERE = Path(__file__).parent
sys.path.insert(0, str(HERE))
import step_metadata as METADATA  # noqa: E402


def payload(first=2, last=2, path='spec/fixturegear/instructions.md'):
    return {'schema': 1, 'citations': [{'path': path, 'first': first, 'last': last}]}


def comment(value):
    return '<!-- step-meta\n%s\n-->' % json.dumps(
        value, sort_keys=True, ensure_ascii=False, indent=2)


class StepMetadataTest(unittest.TestCase):
    def test_steps_of_keeps_the_existing_output(self):
        text = ('preamble\n\n## S1 `[PROSE]` First\n\nBody one.\n\n'
                '## S2 `[GO]` Second — `stepTwo`\n\nBody two.\n')

        self.assertEqual(METADATA.steps_of(text), [
            ('S1', 'PROSE', 'First\n\n\nBody one.\n\n'),
            ('S2', 'GO', 'Second — `stepTwo`\n\n\nBody two.\n'),
        ])

    def test_file_version_distinguishes_legacy_and_version_one(self):
        self.assertIsNone(METADATA.file_version('## S1 `[PROSE]` One\n'))
        self.assertEqual(
            METADATA.file_version('<!-- step-metadata: 1 -->\n\n## S1 `[PROSE]` One\n'), 1)

    def test_unknown_malformed_and_repeated_file_markers_do_not_fall_back(self):
        cases = (
            '<!-- step-metadata: 99 -->\n\n## S1 `[PROSE]` One\n',
            'prefix <!-- step-metadata: 1 -->\n\n## S1 `[PROSE]` One\n',
            '<!-- step-metadata:\n1 -->\n\n## S1 `[PROSE]` One\n',
            '<!-- step-metadata: 1 -->\n<!-- step-metadata: 1 -->\n## S1 `[PROSE]` One\n',
            '## S1 `[PROSE]` One\n\n<!-- step-metadata: 1 -->\n',
        )
        for text in cases:
            with self.subTest(text=text), self.assertRaises(METADATA.MetadataError):
                METADATA.file_version(text)

    def test_contract_fence_does_not_create_a_second_file_marker(self):
        text = (
            '<!-- step-metadata: 1 -->\n\n'
            '## Provenance\n\n'
            '## Compilation contract\n\n'
            '`````json\n{"note": "<!-- step-metadata: 99 -->"}\n`````\n\n'
            '## S1 `[PROSE]` One\n')

        self.assertEqual(METADATA.file_version(text), 1)

    def test_duplicate_json_key_is_rejected(self):
        body = ('One\n\n<!-- step-meta\n'
                '{"schema": 1, "citations": [{"path": '
                '"spec/fixturegear/instructions.md", "first": 1, "first": 2, "last": 2}]}\n'
                '-->\n')

        with self.assertRaisesRegex(METADATA.MetadataError, 'repeats key'):
            METADATA.parse_step(body, 1)

    def test_unknown_keys_and_boolean_lines_are_rejected(self):
        unknown = payload()
        unknown['future'] = True
        boolean = payload()
        boolean['citations'][0]['first'] = True
        for value in (unknown, boolean):
            with self.subTest(value=value), self.assertRaises(METADATA.MetadataError):
                METADATA.parse_step('One\n\n%s\n' % comment(value), 1)

    def test_duplicate_step_payload_is_rejected(self):
        body = 'One\n\n%s\n\n%s\n' % (comment(payload()), comment(payload()))

        with self.assertRaisesRegex(METADATA.MetadataError, 'repeated step-meta'):
            METADATA.parse_step(body, 1)

    def test_adjacent_from_blocks_are_rejected(self):
        body = ('One\n\n%s\n\n'
                '**From:** `spec/fixturegear/instructions.md` L1.\n'
                '**From:** `spec/fixturegear/instructions.md` L2.\n' % comment(payload()))

        with self.assertRaisesRegex(METADATA.MetadataError, r'repeated \*\*From'):
            METADATA.parse_step(body, 1)

    def test_validate_citations_reports_ranges_and_missing_sources(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            source = root / 'spec' / 'fixturegear' / 'instructions.md'
            source.parent.mkdir(parents=True)
            source.write_text('one\ntwo\nthree\n')
            self.assertEqual(METADATA.validate_citations(payload(1, 3), str(root)), [])
            self.assertIn('first line is after', METADATA.validate_citations(payload(3, 2), str(root))[0])
            self.assertIn('has 3 lines', METADATA.validate_citations(payload(1, 4), str(root))[0])
            self.assertIn('does not exist', METADATA.validate_citations(
                payload(1, 1, 'spec/fixturegear/missing.md'), str(root))[0])

    def test_unreadable_source_propagates_os_error(self):
        with mock.patch('builtins.open', side_effect=PermissionError('permission denied')):
            with self.assertRaisesRegex(PermissionError, 'permission denied'):
                METADATA.validate_citations(payload(), '.')

    def test_render_from_preserves_order_and_uses_canonical_ranges(self):
        value = payload(1, 3)
        value['citations'].append({
            'path': 'spec/fixturegear/fusion.md', 'first': 2, 'last': 2})

        self.assertEqual(
            METADATA.render_from(value),
            '**From:** `spec/fixturegear/instructions.md` L1–3; '
            '`spec/fixturegear/fusion.md` L2.')


if __name__ == '__main__':
    unittest.main()
