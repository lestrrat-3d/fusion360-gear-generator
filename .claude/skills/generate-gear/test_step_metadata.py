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


def call_declaration(**changes):
    call = dict(span='tools.addWidget(item)', name='addWidget', receiver='tools',
                owner='adsk.fusion.WidgetTools', role='required', condition=None, reason=None)
    call.update(changes)
    return call


def version_two(calls=None, body=None, preamble='', title='Fixture step'):
    calls = [call_declaration()] if calls is None else calls
    value = dict(payload(), schema=2, calls=calls)
    body = 'Call `tools.addWidget(item)`.' if body is None else body
    return ('<!-- step-metadata: 2 -->\n%s\n## S1 `[PROSE]` %s\n\n%s\n\n%s\n\n%s\n'
            % (preamble, title, comment(value), METADATA.render_from(payload()), body))


class VersionTwoMetadataTest(unittest.TestCase):
    def parse(self, calls=None, body=None, **kwargs):
        return METADATA.file_calls(version_two(calls, body, **kwargs))

    def test_required_call_present(self):
        self.assertEqual(self.parse(), [call_declaration()])

    def test_prose_parenthesis(self):
        span = "dimensionless (units '')"
        call = call_declaration(span=span, name='dimensionless', receiver=None, owner=None,
                                role='prose', reason='This describes the unit string.')
        self.assertEqual(self.parse([call], '`%s`' % span), [call])

    def test_real_call_cannot_be_prose(self):
        for span in ('tools.addWidget(item)', 'dimensionless(units)', '(tools.addWidget(item))'):
            name, receiver = next(iter(METADATA.call_shapes(span)))
            with self.subTest(span=span), self.assertRaisesRegex(METADATA.MetadataError, 'valid Python'):
                self.parse([call_declaration(span=span, name=name, receiver=receiver, owner=None,
                                              role='prose', reason='Description.')], '`%s`' % span)

    def test_example_has_reason(self):
        with self.assertRaisesRegex(METADATA.MetadataError, 'reason'):
            self.parse([call_declaration(role='example')])

    def test_missing_declaration(self):
        with self.assertRaisesRegex(METADATA.MetadataError, 'S1: missing.*otherWidget'):
            self.parse(body='`tools.addWidget(item)` and `tools.otherWidget(item)`.')

    def test_declaration_without_span(self):
        with self.assertRaisesRegex(METADATA.MetadataError, 'no matching inline span'):
            self.parse(body='The call was deleted.')

    def test_declaration_not_self_evidence(self):
        call = call_declaration(reason=None)
        with self.assertRaisesRegex(METADATA.MetadataError, 'no matching inline span'):
            self.parse([call], '')

    def test_metadata_reason_backticks_are_not_evidence(self):
        call = call_declaration(role='example', reason='Example `tools.addWidget(item)`.')
        with self.assertRaisesRegex(METADATA.MetadataError, 'no matching inline span'):
            self.parse([call], '')

    def test_conflicting_duplicate(self):
        for duplicate in (call_declaration(), call_declaration(role='example', reason='An example.')):
            with self.subTest(duplicate=duplicate), self.assertRaisesRegex(METADATA.MetadataError, 'repeats'):
                self.parse([call_declaration(), duplicate])

    def test_invalid_call_fields(self):
        cases = [dict(owner='WidgetTools'), dict(owner='adsk.fusion.Widgeté'), dict(owner=True),
                 dict(name='otherWidget'), dict(receiver=None), dict(span='addWidget(item)'),
                 dict(role='optional'), dict(condition=''), dict(reason='Not null.'),
                 dict(role='inherited', reason='Framework.'), dict(name='addWidget\n'),
                 dict(role='example', reason='Example.', condition='Only sometimes.'),
                 dict(role='forbidden', reason='-->'), dict(future='unknown')]
        for change in cases:
            with self.subTest(change=change), self.assertRaises(METADATA.MetadataError):
                self.parse([call_declaration(**change)])
        value = dict(payload(), schema=2, calls={})
        with self.assertRaisesRegex(METADATA.MetadataError, 'array'):
            METADATA.parse_step(comment(value), 2)
        value['calls'] = [dict(call_declaration())]
        del value['calls'][0]['condition']
        with self.assertRaisesRegex(METADATA.MetadataError, 'omits required'):
            METADATA.parse_step(comment(value), 2)

    def test_v2_global_ignore_rejected(self):
        for checker in ('check-compile', 'check-step-calls'):
            with self.subTest(checker=checker), self.assertRaisesRegex(METADATA.MetadataError, 'global ignore'):
                self.parse(body='`tools.addWidget(item)`\n<!-- %s: ignore addWidget -->' % checker)

    def test_preamble_call_rejected(self):
        with self.assertRaisesRegex(METADATA.MetadataError, 'preamble.*relevant step'):
            self.parse(preamble='Use `tools.addWidget(item)`.')

    def test_repeated_spans_share_a_declaration(self):
        self.assertEqual(self.parse(body='`tools.addWidget(item)` twice: `tools.addWidget(item)`.'),
                         [call_declaration()])

    def test_nested_calls_need_separate_declarations(self):
        span = 'tools.addWidget(items.copy())'
        outer = call_declaration(span=span)
        inner = call_declaration(span=span, name='copy', receiver='items', owner=None)
        self.assertEqual(self.parse([outer, inner], '`%s`' % span), [outer, inner])
        with self.assertRaisesRegex(METADATA.MetadataError, 'missing.*copy'):
            self.parse([outer], '`%s`' % span)

    def test_title_calls_require_declarations(self):
        self.assertEqual(self.parse(body='', title='Use `tools.addWidget(item)`'), [call_declaration()])

    def test_ordinary_comment_calls_require_declarations(self):
        with self.assertRaisesRegex(METADATA.MetadataError, 'missing.*addWidget'):
            self.parse([], '<!-- note: `tools.addWidget(item)` -->')

    def test_fenced_calls_do_not_require_declarations(self):
        for fence in ('```', '`````', '~~~'):
            body = '%spython\n`tools.addWidget(item)`\n%s\n' % (fence, fence)
            if len(fence) > 3:
                body = body.replace('`tools', '```\n`tools')
            with self.subTest(fence=fence):
                self.assertEqual(self.parse([], body), [])

    def test_generated_contract_is_masked(self):
        preamble = ('## Compilation contract\n\n`````json\n'
                    '{"description": "`tools.addWidget(item)`", '
                    '"note": "<!-- check-step-calls: ignore addWidget -->"}\n`````\n')
        self.assertEqual(self.parse([], '', preamble=preamble), [])

    def test_empty_calls_are_valid_only_without_call_spans(self):
        self.assertEqual(self.parse([], 'No call here.'), [])
        with self.assertRaisesRegex(METADATA.MetadataError, 'missing'):
            self.parse([])


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
