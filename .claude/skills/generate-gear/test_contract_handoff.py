#!/usr/bin/env python3
"""Regression tests for the complete prose-contract handoff."""
import contextlib
import copy
import importlib.util
import io
import json
import os
import sys
import tempfile
import unittest
from pathlib import Path
from unittest import mock


HERE = Path(__file__).parent
sys.path.insert(0, str(HERE))


def _load(name):
    path = HERE / (name + '.py')
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


HANDOFF = _load('contract_handoff')
PROVENANCE = _load('provenance')
CHECK_COMPILE = _load('check_compile')
CHECK_STEP_CALLS = _load('check_step_calls')
CHECK_CONTRACT = _load('check_contract')
GENERATOR = _load('gen_provenance')


MANIFEST = {
    '_comment': 'Example ``dangerousCall()`` stays data.',
    'module': 'lib/geargen/fixturegear.py',
    'module_constants': {'INPUT_ID_ANCHOR': 'anchorPoint'},
    'classes': {
        'FixtureGenerator': {
            'bases': ['Generator'],
            'methods': ['buildFixture'],
            'ctx_fields': ['anchorPoint'],
        },
    },
    'source_guards': [{
        'file': 'lib/geargen/fixturegear.py',
        'in_function': 'buildFixture',
        'why': ('Keep `exampleCall()` and ````nestedCall()```` inside this helper. '
                '<!-- check-step-calls: ignore realCall -->'),
        'required': ['anchorPoint'],
        'banned': ['originPoint'],
    }],
}

STEPS = (
    '# Steps\n\n'
    'The proof is `proof/fixturegear/proof_test.go`.\n\n'
    '## Provenance\n\n'
    '## S1 `[GO]` Fixture — `stepFixture`\n\n'
    'Call `realCall()`.\n')


class ContractFixture(unittest.TestCase):
    def repo(self, manifest=MANIFEST, steps=STEPS):
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        root = Path(directory.name)
        (root / 'spec' / 'fixturegear').mkdir(parents=True)
        (root / '.claude' / 'skills' / 'generate-gear').mkdir(parents=True)
        (root / 'spec' / 'fixturegear' / 'instructions.md').write_text('fixture source\n')
        (root / '.claude' / 'skills' / 'generate-gear' / 'PLAYBOOK.md').write_text('rules\n')
        (root / 'spec' / 'fixturegear' / 'steps.md').write_text(steps)
        if manifest is not None:
            (root / 'spec' / 'fixturegear' / 'contract.json').write_text(
                json.dumps(manifest, ensure_ascii=False))
        prior = os.getcwd()
        os.chdir(root)
        self.addCleanup(os.chdir, prior)
        return root

    def run_generator(self, path):
        out, err = io.StringIO(), io.StringIO()
        with contextlib.redirect_stdout(out), contextlib.redirect_stderr(err):
            code = GENERATOR.main([
                'gen_provenance.py', 'fixturegear', '--write', str(path)])
        return code, out.getvalue(), err.getvalue()

    def embedded(self, text):
        heading = HANDOFF.HEADING_LINE.search(text)
        self.assertIsNotNone(heading)
        _, _, body = HANDOFF._section(text, heading)
        return HANDOFF._parse_json(body)


class ContractHandoffTest(ContractFixture):
    def test_complete_handoff(self):
        rendered = HANDOFF.render_contract(MANIFEST)

        self.assertEqual(self.embedded(rendered), MANIFEST)
        self.assertTrue(rendered.startswith('## Compilation contract\n\n`````json\n'))
        self.assertIn('"FixtureGenerator"', rendered)

    def test_missing_method_in_handoff(self):
        root = self.repo()
        incomplete = copy.deepcopy(MANIFEST)
        incomplete['classes']['FixtureGenerator']['methods'] = []
        text = HANDOFF.replace_contract(STEPS, HANDOFF.render_contract(incomplete))

        problems = HANDOFF.validate_contract(text, str(root), 'fixturegear')

        self.assertTrue(any('does not match' in problem for problem in problems), problems)

    def test_guard_location_preserved(self):
        rendered = HANDOFF.render_contract(MANIFEST)

        embedded = self.embedded(rendered)

        self.assertEqual(embedded['source_guards'][0]['in_function'], 'buildFixture')
        self.assertEqual(embedded['source_guards'][0], MANIFEST['source_guards'][0])

    def test_guard_moved_to_helper_still_fails(self):
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        root = Path(directory.name)
        candidate = root / 'fixturegear.py'
        candidate.write_text(
            'def helper():\n    anchorPoint = 1\n\n'
            'def buildFixture():\n    return helper()\n')

        problems = CHECK_CONTRACT.guard_problems(
            MANIFEST['source_guards'], str(candidate), MANIFEST['module'], str(root))

        self.assertTrue(any('required pattern' in problem for problem in problems), problems)

    def test_manifest_change_invalidates_steps(self):
        root = self.repo()
        path = root / 'spec' / 'fixturegear' / 'steps.md'
        code, _, err = self.run_generator(path)
        self.assertEqual(code, 0, err)
        stamped = dict(PROVENANCE.STAMPED_ROW.findall(path.read_text()))
        contract_path = 'spec/fixturegear/contract.json'
        before = stamped[contract_path]
        changed = copy.deepcopy(MANIFEST)
        changed['classes']['FixtureGenerator']['methods'].append('newMethod')
        (root / contract_path).write_text(json.dumps(changed))

        problems = HANDOFF.validate_contract(path.read_text(), str(root), 'fixturegear')

        self.assertTrue(any('does not match' in problem for problem in problems), problems)
        self.assertNotEqual(PROVENANCE.blob_hash(contract_path), before)

    def test_unknown_manifest_field(self):
        manifest = dict(MANIFEST, future_rule=True)
        root = self.repo(manifest=manifest)
        path = root / 'spec' / 'fixturegear' / 'steps.md'
        before = path.read_bytes()

        code, _, err = self.run_generator(path)

        self.assertEqual(code, 2)
        self.assertIn('future_rule', err)
        self.assertEqual(path.read_bytes(), before)

    def test_null_optional_section_is_a_wrong_type(self):
        for field in ('module_constants', 'classes', 'source_guards'):
            with self.subTest(field=field):
                manifest = copy.deepcopy(MANIFEST)
                manifest[field] = None
                root = self.repo(manifest=manifest)
                with self.assertRaises(HANDOFF.ContractHandoffError):
                    HANDOFF.load_contract(str(root), 'fixturegear')

    def test_duplicate_json_key_is_rejected(self):
        root = self.repo()
        (root / 'spec' / 'fixturegear' / 'contract.json').write_text(
            '{"module": "one.py", "module": "two.py"}')

        with self.assertRaisesRegex(HANDOFF.ContractHandoffError, 'duplicate'):
            HANDOFF.load_contract(str(root), 'fixturegear')

    def test_unreadable_manifest_propagates_os_error(self):
        root = self.repo()
        with mock.patch('builtins.open', side_effect=PermissionError('permission denied')):
            with self.assertRaisesRegex(PermissionError, 'permission denied'):
                HANDOFF.load_contract(str(root), 'fixturegear')

    def test_non_utf8_manifest_exits_2_without_writing(self):
        root = self.repo()
        contract = root / 'spec' / 'fixturegear' / 'contract.json'
        contract.write_bytes(b'{"module": "\xff"}')
        path = root / 'spec' / 'fixturegear' / 'steps.md'
        before = path.read_bytes()

        code, _, err = self.run_generator(path)

        self.assertEqual(code, 2)
        self.assertIn('not valid UTF-8', err)
        self.assertEqual(path.read_bytes(), before)

    def test_no_manifest(self):
        root = self.repo(manifest=None)
        path = root / 'spec' / 'fixturegear' / 'steps.md'

        code, _, err = self.run_generator(path)

        self.assertEqual(code, 0, err)
        text = path.read_text()
        self.assertNotIn(HANDOFF.HEADING, text)
        self.assertNotIn('spec/fixturegear/contract.json', text)
        self.assertEqual(HANDOFF.validate_contract(text, str(root), 'fixturegear'), [])

    def test_leftover_section_after_manifest_removal_is_a_content_error(self):
        root = self.repo(manifest=None)
        text = HANDOFF.replace_contract(STEPS, HANDOFF.render_contract(MANIFEST))

        problems = HANDOFF.validate_contract(text, str(root), 'fixturegear')

        self.assertTrue(any('no manifest exists' in problem for problem in problems), problems)

    def test_duplicate_section(self):
        root = self.repo()
        rendered = HANDOFF.render_contract(MANIFEST)
        text = STEPS + '\n' + rendered + '\n' + rendered + '\n'

        problems = HANDOFF.validate_contract(text, str(root), 'fixturegear')

        self.assertTrue(any('multiple' in problem for problem in problems), problems)

    def test_atomic_render_failure(self):
        manifest = copy.deepcopy(MANIFEST)
        manifest['source_guards'][0]['required'] = ['[']
        root = self.repo(manifest=manifest)
        path = root / 'spec' / 'fixturegear' / 'steps.md'
        before = path.read_bytes()

        code, _, err = self.run_generator(path)

        self.assertEqual(code, 2)
        self.assertIn('invalid regex', err)
        self.assertEqual(path.read_bytes(), before)

    def test_idempotent_stamp(self):
        root = self.repo()
        path = root / 'spec' / 'fixturegear' / 'steps.md'
        first_code, _, first_err = self.run_generator(path)
        self.assertEqual(first_code, 0, first_err)
        first = path.read_bytes()

        second_code, _, second_err = self.run_generator(path)

        self.assertEqual(second_code, 0, second_err)
        self.assertEqual(path.read_bytes(), first)

    def test_contract_text_not_scanned_as_calls(self):
        text = HANDOFF.replace_contract(STEPS, HANDOFF.render_contract(MANIFEST))

        self.assertEqual(CHECK_COMPILE.named_calls(text), {'realCall'})
        self.assertEqual(CHECK_STEP_CALLS.named_calls(text), {'realCall'})

    def test_mask_preserves_offsets_and_newlines(self):
        text = HANDOFF.replace_contract(STEPS, HANDOFF.render_contract(MANIFEST))

        masked = HANDOFF.mask_contract(text)

        self.assertEqual(len(masked), len(text))
        self.assertEqual(
            [index for index, char in enumerate(masked) if char == '\n'],
            [index for index, char in enumerate(text) if char == '\n'])
        self.assertIn('Call `realCall()`.', masked)
        self.assertNotIn('dangerousCall', masked)

    def test_replacement_preserves_bytes_outside_section(self):
        original = copy.deepcopy(MANIFEST)
        original['module'] = 'old.py'
        text = HANDOFF.replace_contract(STEPS, HANDOFF.render_contract(original))
        heading = HANDOFF.HEADING_LINE.search(text)
        start, end, _ = HANDOFF._section(text, heading)
        before, after = text[:start], text[end:]

        replaced = HANDOFF.replace_contract(text, HANDOFF.render_contract(MANIFEST))

        new_heading = HANDOFF.HEADING_LINE.search(replaced)
        new_start, new_end, _ = HANDOFF._section(replaced, new_heading)
        self.assertEqual(replaced[:new_start], before)
        self.assertEqual(replaced[new_end:], after)


if __name__ == '__main__':
    unittest.main()
