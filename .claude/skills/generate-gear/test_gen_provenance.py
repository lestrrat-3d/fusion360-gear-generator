#!/usr/bin/env python3
"""Regression tests for the provenance-table generator.

`gen_provenance.py` exists so no language model ever hand-types a `git hash-object` value into a
step list's `## Provenance` table. These tests hold it to reproducing the same table
`check_compile.py`'s gate would accept, and to never touching its target file on a failure.
"""
import contextlib
import importlib.util
import io
import os
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path
from unittest import mock

HERE = Path(__file__).parent


def _load(name, filename):
    spec = importlib.util.spec_from_file_location(name, HERE / filename)
    module = importlib.util.module_from_spec(spec)
    # Registered before exec so that check_compile.py's own `from provenance import (...)` and
    # gen_provenance.py's own `import provenance` resolve to this exact module object rather than
    # a second copy, which is what test_checker_and_generator_share_one_input_set pins.
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


PROVENANCE = _load('provenance', 'provenance.py')
COMPILE_CHECKER = _load('check_compile', 'check_compile.py')
GENERATOR = _load('gen_provenance', 'gen_provenance.py')

DEFAULT_STEPS = (
    '# Steps\n\n'
    'The proof is `proof/gear/proof_test.go`.\n\n'
    '## Provenance\n\n'
    '## S1 `[GO]` One — `stepOne`\n\n'
    'Build the thing.\n\n'
    # A `[GO]` step declares the run its registration is generated from, and
    # `check_compile.py` blocks on a step that carries no such annotation, so the fixture that
    # is fed to that checker below has to carry one.
    '<!-- proof-run: proofkit.Run(profileCases, stepOne) -->\n\n'
    '**From:** `spec/gear/instructions.md` L1\n')

PROOF_BODY = (
    'func TestOne(t *testing.T) {\n'
    '\tproofkit.Run(t, profileCases, stepOne)\n'
    '}\n\n'
    'func stepOne() {}\n')


def git_hash_object(path):
    """The independent ground truth this test file computes for itself.

    Deliberately not `PROVENANCE.blob_hash`: reusing the function under test to check its own
    output would not catch a regression in that function.
    """
    return subprocess.run(
        ['git', 'hash-object', str(path)], capture_output=True, text=True).stdout.strip()


class GenProvenanceTest(unittest.TestCase):
    def repo(self, fusion=True, auxiliary=False, steps=None):
        """Build a miniature repo and chdir into it.

        `spec/gear/instructions.md`, an optional `spec/gear/fusion.md`, the shared
        `PLAYBOOK.md`, an optional `trace.md` referenced by `instructions.md` (`auxiliary=True`)
        or by `fusion.md` (`auxiliary='fusion'`), and a drafted `spec/gear/steps.md` whose
        provenance section is empty unless `steps` overrides it.

        cwd is restored and the tree is removed on test teardown via addCleanup. Returns the
        repo root as a Path.
        """
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        root = Path(directory.name)
        (root / 'spec' / 'gear').mkdir(parents=True)
        (root / 'proof' / 'gear').mkdir(parents=True)
        (root / '.claude' / 'skills' / 'generate-gear').mkdir(parents=True)

        instructions_text = 'source\nline two\nline three\n'
        fusion_text = 'source\nline two\nline three\n'
        if auxiliary is True:
            instructions_text = 'source\nSee `trace.md` for details.\nline three\n'
        elif auxiliary == 'fusion':
            fusion_text = 'source\nSee `trace.md` for details.\nline three\n'
        if auxiliary:
            (root / 'spec' / 'gear' / 'trace.md').write_text('trace\n')
        (root / 'spec' / 'gear' / 'instructions.md').write_text(instructions_text)
        if fusion:
            (root / 'spec' / 'gear' / 'fusion.md').write_text(fusion_text)
        (root / '.claude' / 'skills' / 'generate-gear' / 'PLAYBOOK.md').write_text(
            'playbook\nline two\nline three\n')

        (root / 'spec' / 'gear' / 'steps.md').write_text(DEFAULT_STEPS if steps is None else steps)

        prior = os.getcwd()
        os.chdir(root)
        self.addCleanup(os.chdir, prior)
        return root

    def run_generator(self, argv):
        out = io.StringIO()
        err = io.StringIO()
        with contextlib.redirect_stdout(out), contextlib.redirect_stderr(err):
            result = GENERATOR.main(argv)
        return result, out.getvalue(), err.getvalue()

    def parse_rows(self, output):
        return PROVENANCE.STAMPED_ROW.findall(output)

    # -- table content and shape --------------------------------------------------------------

    def test_table_lists_every_provenance_input(self):
        self.repo()

        result, out, err = self.run_generator(['gen_provenance.py', 'gear'])

        self.assertEqual(result, 0, err)
        rows = self.parse_rows(out)
        self.assertEqual(len(rows), 3)
        for path, digest in rows:
            self.assertEqual(digest, git_hash_object(path))

    def test_rows_are_ordered_instructions_fusion_auxiliary_playbook(self):
        self.repo(auxiliary=True)

        result, out, err = self.run_generator(['gen_provenance.py', 'gear'])

        self.assertEqual(result, 0, err)
        paths = [path for path, _ in self.parse_rows(out)]
        self.assertEqual(paths, [
            'spec/gear/instructions.md',
            'spec/gear/fusion.md',
            'spec/gear/trace.md',
            '.claude/skills/generate-gear/PLAYBOOK.md',
        ])

    def test_absent_fusion_sidecar_is_omitted(self):
        self.repo(fusion=False)

        result, out, err = self.run_generator(['gen_provenance.py', 'gear'])

        self.assertEqual(result, 0, err)
        rows = self.parse_rows(out)
        self.assertEqual(len(rows), 2)
        self.assertNotIn('spec/gear/fusion.md', out)

    def test_auxiliary_document_referenced_by_the_fusion_sidecar_is_included(self):
        self.repo(auxiliary='fusion')

        result, out, err = self.run_generator(['gen_provenance.py', 'gear'])

        self.assertEqual(result, 0, err)
        paths = [path for path, _ in self.parse_rows(out)]
        self.assertEqual(paths, [
            'spec/gear/instructions.md',
            'spec/gear/fusion.md',
            'spec/gear/trace.md',
            '.claude/skills/generate-gear/PLAYBOOK.md',
        ])

    def test_header_and_separator_match_the_committed_step_list(self):
        self.repo()

        result, out, err = self.run_generator(['gen_provenance.py', 'gear'])

        self.assertEqual(result, 0, err)
        lines = out.splitlines()
        self.assertEqual(lines[2], '| file | `git hash-object` |')
        self.assertEqual(lines[3], '|---|---|')

    def test_stdout_form_prints_the_heading_and_touches_no_file(self):
        root = self.repo()
        steps_path = root / 'spec' / 'gear' / 'steps.md'
        before = steps_path.read_bytes()

        result, out, err = self.run_generator(['gen_provenance.py', 'gear'])

        self.assertEqual(result, 0, err)
        self.assertTrue(out.startswith('## Provenance\n\n'))
        self.assertEqual(steps_path.read_bytes(), before)

    # -- agreement with the checker ------------------------------------------------------------

    def test_generated_table_satisfies_the_compile_check(self):
        root = self.repo()
        (root / 'proof' / 'gear' / 'proof_test.go').write_text(PROOF_BODY)
        # check_compile.py requires the proof path the summary names to be tracked or committed,
        # which a plain tempdir is neither; `git add` (no commit needed) satisfies `git ls-files`.
        subprocess.run(['git', 'init', '--quiet'], cwd=root, check=True)
        subprocess.run(['git', 'add', 'proof/gear/proof_test.go'], cwd=root, check=True)
        steps_path = root / 'spec' / 'gear' / 'steps.md'

        write_result, _, write_err = self.run_generator(
            ['gen_provenance.py', 'gear', '--write', str(steps_path)])
        self.assertEqual(write_result, 0, write_err)

        out = io.StringIO()
        lookup_patch = mock.patch.object(COMPILE_CHECKER.fusion_api, 'lookup_many', return_value={})
        findings_patch = mock.patch.object(
            COMPILE_CHECKER.fusion_api, 'unverified_findings', return_value=[])
        with lookup_patch, findings_patch, contextlib.redirect_stdout(out):
            result = COMPILE_CHECKER.main(['check_compile.py', 'gear'])

        self.assertEqual(result, 0, out.getvalue())
        self.assertIn('compile check: OK', out.getvalue())

    def test_generated_rows_parse_with_the_checker_row_pattern(self):
        self.repo(auxiliary=True)

        result, out, err = self.run_generator(['gen_provenance.py', 'gear'])

        self.assertEqual(result, 0, err)
        parsed = dict(PROVENANCE.STAMPED_ROW.findall(out))
        self.assertEqual(set(parsed), PROVENANCE.provenance_inputs('gear'))

    def test_checker_and_generator_share_one_input_set(self):
        self.assertIs(COMPILE_CHECKER.provenance_inputs, PROVENANCE.provenance_inputs)
        self.assertIs(COMPILE_CHECKER.blob_hash, PROVENANCE.blob_hash)

    # -- writing into a step list --------------------------------------------------------------

    def test_write_fills_an_empty_provenance_section(self):
        root = self.repo()
        steps_path = root / 'spec' / 'gear' / 'steps.md'

        result, out, err = self.run_generator(
            ['gen_provenance.py', 'gear', '--write', str(steps_path)])

        self.assertEqual(result, 0, err)
        text = steps_path.read_text()
        self.assertIn('The proof is `proof/gear/proof_test.go`.', text)
        self.assertIn('## S1 `[GO]` One — `stepOne`', text)
        self.assertEqual(len(PROVENANCE.STAMPED_ROW.findall(text)), 3)

    def test_write_replaces_a_stale_table_rather_than_appending(self):
        stale = (
            '# Steps\n\n'
            'The proof is `proof/gear/proof_test.go`.\n\n'
            '## Provenance\n\n'
            '| file | `git hash-object` |\n'
            '|---|---|\n'
            '| `spec/gear/instructions.md` | `0000000000000000000000000000000000000000` |\n\n'
            '## S1 `[GO]` One — `stepOne`\n\n'
            'Build the thing.\n\n'
            '**From:** `spec/gear/instructions.md` L1\n')
        root = self.repo(steps=stale)
        steps_path = root / 'spec' / 'gear' / 'steps.md'

        result, out, err = self.run_generator(
            ['gen_provenance.py', 'gear', '--write', str(steps_path)])

        self.assertEqual(result, 0, err)
        text = steps_path.read_text()
        self.assertEqual(text.count('## Provenance'), 1)
        self.assertNotIn('0000000000000000000000000000000000000000', text)

    def test_write_is_idempotent(self):
        root = self.repo()
        steps_path = root / 'spec' / 'gear' / 'steps.md'

        first_result, _, first_err = self.run_generator(
            ['gen_provenance.py', 'gear', '--write', str(steps_path)])
        self.assertEqual(first_result, 0, first_err)
        first = steps_path.read_bytes()

        second_result, _, second_err = self.run_generator(
            ['gen_provenance.py', 'gear', '--write', str(steps_path)])
        self.assertEqual(second_result, 0, second_err)
        second = steps_path.read_bytes()

        self.assertEqual(first, second)

    def test_write_keeps_the_proof_summary_readable_by_the_checker(self):
        root = self.repo()
        steps_path = root / 'spec' / 'gear' / 'steps.md'

        result, out, err = self.run_generator(
            ['gen_provenance.py', 'gear', '--write', str(steps_path)])

        self.assertEqual(result, 0, err)
        text = steps_path.read_text()
        self.assertEqual(COMPILE_CHECKER.proof_paths(text), ['proof/gear/proof_test.go'])

    def test_write_prints_a_summary_line(self):
        root = self.repo()
        steps_path = root / 'spec' / 'gear' / 'steps.md'

        result, out, err = self.run_generator(
            ['gen_provenance.py', 'gear', '--write', str(steps_path)])

        self.assertEqual(result, 0, err)
        self.assertIn('gen_provenance: stamped 3 source(s) into %s' % steps_path, out)

    # -- failure modes: exit 2, target untouched -------------------------------------------------

    def test_missing_gear_instructions_exit_2(self):
        self.repo()

        result, out, err = self.run_generator(['gen_provenance.py', 'nosuchgear'])

        self.assertEqual(result, 2)
        self.assertIn('spec/nosuchgear/instructions.md', err)

    def test_missing_target_file_exits_2(self):
        root = self.repo()
        missing = root / 'spec' / 'gear' / 'nope.md'

        result, out, err = self.run_generator(
            ['gen_provenance.py', 'gear', '--write', str(missing)])

        self.assertEqual(result, 2)
        self.assertIn(str(missing), err)
        self.assertFalse(missing.exists())

    def test_target_without_a_provenance_heading_exits_2(self):
        root = self.repo(steps='# Steps\n\nNo heading here.\n')
        steps_path = root / 'spec' / 'gear' / 'steps.md'
        before = steps_path.read_bytes()

        result, out, err = self.run_generator(
            ['gen_provenance.py', 'gear', '--write', str(steps_path)])

        self.assertEqual(result, 2)
        self.assertIn('## Provenance', err)
        self.assertEqual(steps_path.read_bytes(), before)

    def test_unhashable_source_exits_2_without_writing(self):
        root = self.repo()
        steps_path = root / 'spec' / 'gear' / 'steps.md'
        before = steps_path.read_bytes()

        with mock.patch.object(PROVENANCE, 'blob_hash', return_value=''):
            result, out, err = self.run_generator(
                ['gen_provenance.py', 'gear', '--write', str(steps_path)])

        self.assertEqual(result, 2)
        self.assertIn('spec/gear/instructions.md', err)
        self.assertEqual(steps_path.read_bytes(), before)

    def test_usage_errors_exit_2(self):
        self.repo()
        cases = (
            [],
            ['gen_provenance.py'],
            ['gen_provenance.py', 'gear', 'extra'],
            ['gen_provenance.py', 'gear', '--wat'],
            ['gen_provenance.py', 'gear', '--write'],
        )
        for argv in cases:
            with self.subTest(argv=argv):
                result, out, err = self.run_generator(argv)

                self.assertEqual(result, 2)
                self.assertIn('usage:', err)

    def test_write_accepts_both_flag_spellings(self):
        root = self.repo()
        steps_path = root / 'spec' / 'gear' / 'steps.md'
        space_result, _, space_err = self.run_generator(
            ['gen_provenance.py', 'gear', '--write', str(steps_path)])
        self.assertEqual(space_result, 0, space_err)
        space_form = steps_path.read_bytes()

        root2 = self.repo()
        steps_path2 = root2 / 'spec' / 'gear' / 'steps.md'
        equals_result, _, equals_err = self.run_generator(
            ['gen_provenance.py', 'gear', '--write=%s' % steps_path2])
        self.assertEqual(equals_result, 0, equals_err)
        equals_form = steps_path2.read_bytes()

        self.assertEqual(space_form, equals_form)


if __name__ == '__main__':
    unittest.main()
