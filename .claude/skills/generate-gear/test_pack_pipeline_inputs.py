#!/usr/bin/env python3
import contextlib
import hashlib
import importlib.util
import io
import json
import os
import tempfile
import unittest
from pathlib import Path
from unittest import mock


PACKER_PATH = Path(__file__).with_name('pack_pipeline_inputs.py')
MODULE_SPEC = importlib.util.spec_from_file_location('pack_pipeline_inputs', PACKER_PATH)
PACKER = importlib.util.module_from_spec(MODULE_SPEC)
MODULE_SPEC.loader.exec_module(PACKER)
REPO_ROOT = PACKER_PATH.resolve().parents[3]


@contextlib.contextmanager
def working_directory(path):
    previous = Path.cwd()
    os.chdir(str(path))
    try:
        yield
    finally:
        os.chdir(str(previous))


def write(root, logical, content='fixture\n'):
    path = root / logical
    path.parent.mkdir(parents=True, exist_ok=True)
    if isinstance(content, bytes):
        path.write_bytes(content)
    else:
        path.write_text(content, encoding='utf-8')
    return path


def manifest(out):
    return json.loads((out / 'manifest.json').read_text(encoding='utf-8'))


def reconstruct(out, entry):
    return b''.join((out / chunk['path']).read_bytes() for chunk in entry['chunks'])


def paths_in(out):
    return [entry['path'] for entry in manifest(out)['files']]


class FixtureCase(unittest.TestCase):
    gear = 'fixturegear'

    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.root = Path(self.temporary.name)

    def tearDown(self):
        self.temporary.cleanup()

    def compile_fixture(self, instructions=None):
        if instructions is None:
            instructions = 'Read PLAYBOOK.md and auxiliary.md.\n'
        write(self.root, 'spec/fixturegear/instructions.md', instructions)
        write(self.root, 'spec/fixturegear/auxiliary.md', 'auxiliary π\r\n'.encode('utf-8'))
        write(self.root, PACKER.provenance.PLAYBOOK, 'playbook\n')
        write(self.root, PACKER.FORMATS, 'format owner\n')
        write(self.root, PACKER.CONSTRUCTION, 'recipes\n')
        write(self.root, PACKER.CONSTRUCTION_EXAMPLE, 'package examples_test\n')
        write(self.root, PACKER.INVOLUTE, 'package involute\n')
        write(self.root, 'proof/proofkit/proofkit.go', 'package proofkit\n')
        write(self.root, 'proof/proofkit/nested/extra.go', 'package nested\n')
        write(self.root, 'proof/proofkit3d/proofkit3d.go', 'package proofkit3d\n')
        write(
            self.root,
            '.claude/skills/compile-gear/prompt.md',
            'Compile {{gear}} from its declared inputs.\n',
        )

    def emit_fixture(self):
        write(self.root, 'spec/fixturegear/steps.md', 'steps\n')
        write(self.root, 'proof/fixturegear/proof.go', 'package fixturegear_test\n')
        write(
            self.root,
            'proof/fixturegear/zz_registrations_test.go',
            'package fixturegear_test\n',
        )
        write(self.root, 'proof/fixturegear/stage-manifest.json', '{}\n')
        write(self.root, '.tmp/fixturegear.playbook-extract.md', 'extract\n')
        write(self.root, '.tmp/fixturegear.emitter-interfaces.md', 'interfaces\n')
        write(self.root, PACKER.FORMATS, 'format owner\n')
        for logical in PACKER.EMIT_FIXED_INPUTS:
            write(self.root, logical, '# framework\n')
        write(self.root, 'lib/fusion360utils/root.py', '# root\n')
        write(self.root, 'lib/fusion360utils/nested/member.py', '# nested\n')
        write(self.root, 'lib/fusion360utils/ignored.txt', 'not Python\n')
        write(
            self.root,
            '.claude/skills/emit-gear/prompt.md',
            'Emit {{gear}} from its declared inputs.\n',
        )


class BundleFormatTest(FixtureCase):
    def test_round_trip_all_bytes(self):
        sources = [
            ('a/empty.txt', b''),
            ('b/unicode.txt', ('αβγ\n' + '界' * 5000 + '\nend\n').encode('utf-8')),
            ('c/plain.txt', b'plain bytes\n'),
        ]
        out = self.root / 'bundle'
        PACKER.write_bundle(out, sources, [])

        entries = manifest(out)['files']
        self.assertEqual([entry['path'] for entry in entries], sorted(path for path, _ in sources))
        original = dict(sources)
        for entry in entries:
            data = reconstruct(out, entry)
            self.assertEqual(data, original[entry['path']])
            self.assertEqual(entry['bytes'], len(data))
            self.assertEqual(entry['sha256'], hashlib.sha256(data).hexdigest())
            next_start = 0
            for chunk in entry['chunks']:
                packed = (out / chunk['path']).read_bytes()
                self.assertEqual(chunk['start_byte'], next_start)
                self.assertEqual(chunk['sha256'], hashlib.sha256(packed).hexdigest())
                next_start = chunk['end_byte']
            self.assertEqual(next_start, len(data))
        self.assertEqual(entries[0]['chunks'], [])

    def test_chunk_bound(self):
        data = ('界' * 9000).encode('utf-8')
        out = self.root / 'bundle'
        PACKER.write_bundle(out, [('long.txt', data)], [])

        entry = manifest(out)['files'][0]
        self.assertGreater(len(entry['chunks']), 1)
        for chunk in entry['chunks']:
            packed = (out / chunk['path']).read_bytes()
            self.assertLessEqual(len(packed), PACKER.CHUNK_BYTES)
            packed.decode('utf-8')
            self.assertEqual(chunk['end_byte'] - chunk['start_byte'], len(packed))
        self.assertEqual(reconstruct(out, entry), data)

    def test_deterministic_bundle(self):
        sources = [('z.txt', b'z\n'), ('a.txt', ('å' * 7000).encode('utf-8'))]
        first = self.root / 'first'
        second = self.root / 'second'
        PACKER.write_bundle(first, sources, [])
        PACKER.write_bundle(second, reversed(sources), [])

        self.assertEqual(
            (first / 'manifest.json').read_bytes(),
            (second / 'manifest.json').read_bytes(),
        )
        first_chunks = sorted((first / 'chunks').iterdir())
        second_chunks = sorted((second / 'chunks').iterdir())
        self.assertEqual([path.name for path in first_chunks], [path.name for path in second_chunks])
        self.assertEqual(
            [path.read_bytes() for path in first_chunks],
            [path.read_bytes() for path in second_chunks],
        )

    def test_existing_output_is_not_replaced(self):
        out = self.root / 'bundle'
        write(out, 'owned.txt', 'keep\n')
        with self.assertRaisesRegex(PACKER.PackError, 'already exists'):
            PACKER.write_bundle(out, [('source.txt', b'new\n')], [])
        self.assertEqual((out / 'owned.txt').read_text(encoding='utf-8'), 'keep\n')

    def test_dangling_output_symlink_is_not_followed(self):
        self.compile_fixture()
        target = self.root / 'missing-target'
        out = self.root / 'bundle'
        out.symlink_to(target, target_is_directory=True)
        with working_directory(self.root):
            code = PACKER.main([
                'fixturegear',
                '--stage',
                'compile',
                '--out',
                'bundle',
            ])
        self.assertEqual(code, 2)
        self.assertTrue(out.is_symlink())
        self.assertFalse(target.exists())


class DiscoveryTest(FixtureCase):
    def test_required_input_missing(self):
        self.emit_fixture()
        (self.root / '.tmp/fixturegear.playbook-extract.md').unlink()
        out = self.root / 'bundle'
        with working_directory(self.root):
            code = PACKER.main([
                self.gear,
                '--stage',
                'emit',
                '--out',
                str(out),
            ])
        self.assertEqual(code, 2)
        self.assertFalse(out.exists())

    def test_required_input_manifest_agreement(self):
        self.compile_fixture()
        (self.root / PACKER.FORMATS).unlink()
        out = self.root / 'bundle'
        with self.assertRaisesRegex(PACKER.PackError, PACKER.FORMATS):
            PACKER.pack(self.root, self.gear, 'compile', out)
        self.assertFalse(out.exists())

    def test_auxiliary_reference_resolution(self):
        self.compile_fixture('Read PLAYBOOK.md and missing-auxiliary.md.\n')
        with self.assertRaisesRegex(PACKER.PackError, 'missing-auxiliary.md'):
            PACKER.discover_compile_inputs(self.root, self.gear)

        write(self.root, 'spec/fixturegear/missing-auxiliary.md', 'now present\n')
        sources = dict(PACKER.discover_compile_inputs(self.root, self.gear))
        self.assertIn(PACKER.provenance.PLAYBOOK, sources)
        self.assertIn('spec/fixturegear/missing-auxiliary.md', sources)

    def test_compile_excludes_old_outputs(self):
        self.compile_fixture()
        write(self.root, 'spec/fixturegear/steps.md', 'old steps\n')
        write(self.root, 'proof/fixturegear/old_test.go', 'package fixturegear_test\n')
        write(self.root, 'lib/geargen/fixturegear.py', '# old module\n')
        out = self.root / 'bundle'
        PACKER.pack(self.root, self.gear, 'compile', out)
        bundled = paths_in(out)
        self.assertNotIn('spec/fixturegear/steps.md', bundled)
        self.assertNotIn('proof/fixturegear/old_test.go', bundled)
        self.assertNotIn('lib/geargen/fixturegear.py', bundled)

    def test_compile_complete_manifest_input_set(self):
        self.compile_fixture()
        write(self.root, 'spec/fixturegear/fusion.md', 'Read PLAYBOOK.md.\n')
        write(self.root, 'spec/fixturegear/contract.json', '{}\n')
        out = self.root / 'bundle'
        PACKER.pack(self.root, self.gear, 'compile', out)

        expected = {
            '@rendered-prompt',
            PACKER.provenance.PLAYBOOK,
            PACKER.FORMATS,
            PACKER.CONSTRUCTION,
            PACKER.CONSTRUCTION_EXAMPLE,
            PACKER.INVOLUTE,
            'proof/proofkit/proofkit.go',
            'proof/proofkit/nested/extra.go',
            'proof/proofkit3d/proofkit3d.go',
            'spec/fixturegear/instructions.md',
            'spec/fixturegear/fusion.md',
            'spec/fixturegear/contract.json',
            'spec/fixturegear/auxiliary.md',
        }
        report = manifest(out)
        self.assertEqual([entry['path'] for entry in report['files']], sorted(expected))
        auxiliary = next(
            entry for entry in report['files']
            if entry['path'] == 'spec/fixturegear/auxiliary.md'
        )
        self.assertEqual(reconstruct(out, auxiliary), 'auxiliary π\r\n'.encode('utf-8'))

    def test_compile_rejects_referenced_old_steps(self):
        self.compile_fixture('Read PLAYBOOK.md and steps.md.\n')
        write(self.root, 'spec/fixturegear/steps.md', 'old steps\n')
        out = self.root / 'bundle'
        with self.assertRaisesRegex(
                PACKER.PackError, 'forbidden old output: spec/fixturegear/steps.md'):
            PACKER.pack(self.root, self.gear, 'compile', out)
        self.assertFalse(out.exists())

    def test_non_utf8_input_is_rejected_without_output(self):
        self.compile_fixture()
        write(self.root, PACKER.CONSTRUCTION, b'\xff\xfe')
        out = self.root / 'bundle'
        with self.assertRaisesRegex(PACKER.PackError, 'UTF-8'):
            PACKER.pack(self.root, self.gear, 'compile', out)
        self.assertFalse(out.exists())

    def test_emit_expansions_and_exclusions(self):
        self.emit_fixture()
        out = self.root / 'bundle'
        PACKER.pack(self.root, self.gear, 'emit', out)
        bundled = paths_in(out)
        self.assertEqual(bundled, sorted(bundled))
        self.assertIn('proof/fixturegear/zz_registrations_test.go', bundled)
        self.assertNotIn('proof/fixturegear/stage-manifest.json', bundled)
        self.assertIn('lib/fusion360utils/root.py', bundled)
        self.assertIn('lib/fusion360utils/nested/member.py', bundled)
        self.assertNotIn('lib/fusion360utils/ignored.txt', bundled)

    def test_matching_nonregular_directory_input_is_rejected(self):
        self.compile_fixture()
        (self.root / 'proof/proofkit/broken.go').symlink_to(
            self.root / 'proof/proofkit/missing.go')
        with self.assertRaisesRegex(PACKER.PackError, 'broken.go'):
            PACKER.discover_compile_inputs(self.root, self.gear)

    def test_nonregular_emit_proof_input_is_rejected(self):
        self.emit_fixture()
        (self.root / 'proof/fixturegear/broken').symlink_to(
            self.root / 'proof/fixturegear/missing')
        with self.assertRaisesRegex(PACKER.PackError, 'proof/fixturegear/broken'):
            PACKER.discover_emit_inputs(self.root, self.gear)


class RegistrationOmissionTest(FixtureCase):
    def test_checked_registration_omission(self):
        self.emit_fixture()
        calls = []

        def checked(root, gear):
            calls.append((root, gear))

        out = self.root / 'bundle'
        PACKER.pack(
            self.root,
            self.gear,
            'emit',
            out,
            omit_registrations=True,
            scaffold_check=checked,
        )
        report = manifest(out)
        self.assertEqual(calls, [(self.root, self.gear)])
        self.assertNotIn('proof/fixturegear/zz_registrations_test.go', paths_in(out))
        self.assertEqual(len(report['omitted']), 1)
        self.assertEqual(
            report['omitted'][0]['path'],
            'proof/fixturegear/zz_registrations_test.go',
        )
        self.assertIn('--omit-registrations', report['omitted'][0]['reason'])

    def test_modified_registration_not_omitted(self):
        self.emit_fixture()
        out = self.root / 'bundle'

        def stale(root, gear):
            raise PACKER.ScaffoldCheckError('registration is stale')

        with mock.patch.object(PACKER, 'run_scaffold_check', side_effect=stale):
            with working_directory(self.root):
                code = PACKER.main([
                    self.gear,
                    '--stage',
                    'emit',
                    '--out',
                    str(out),
                    '--omit-registrations',
                ])
        self.assertEqual(code, 1)
        self.assertFalse(out.exists())

    def test_omit_flag_is_invalid_for_compile(self):
        self.compile_fixture()
        out = self.root / 'bundle'
        with self.assertRaises(PACKER.PackError):
            PACKER.pack(
                self.root,
                self.gear,
                'compile',
                out,
                omit_registrations=True,
            )
        self.assertFalse(out.exists())


class BundleRetryTest(FixtureCase):
    def test_fresh_retry_delivers_bundled_prompt_once_with_verbatim_feedback(self):
        self.compile_fixture()
        out = self.root / 'bundle'
        PACKER.pack(self.root, self.gear, 'compile', out)
        report = 'GATE proof FAIL\n```\nUnicode π and {{gear}} stay literal.\n```\n'
        failure = write(self.root, '.tmp/fixturegear.compile-gates.txt', report)

        prompt_entry = next(
            entry for entry in manifest(out)['files']
            if entry['path'] == '@rendered-prompt'
        )
        bundled_prompt = reconstruct(out, prompt_entry).decode('utf-8')
        rendered, errors = io.StringIO(), io.StringIO()
        with contextlib.redirect_stdout(rendered), contextlib.redirect_stderr(errors):
            code = PACKER.render_prompt.main(
                [
                    'render_prompt.py',
                    'compile-gear',
                    self.gear,
                    '--failure-file',
                    str(failure),
                ],
                skills_root=self.root / '.claude/skills',
            )
        retry = rendered.getvalue()

        self.assertEqual(code, 0, errors.getvalue())
        self.assertEqual(retry[:len(bundled_prompt)], bundled_prompt)
        self.assertEqual(retry.count(bundled_prompt), 1)
        feedback = retry.split(PACKER.render_prompt.BEGIN_MARKER + '\n', 1)[1]
        feedback = feedback.split('\n' + PACKER.render_prompt.END_MARKER, 1)[0]
        self.assertEqual(feedback + '\n', report)


class PromptAgreementTest(unittest.TestCase):
    @staticmethod
    def declared_paths(prompt_path, start, end):
        text = prompt_path.read_text(encoding='utf-8')
        block = text.split(start, 1)[1].split(end, 1)[0]
        paths = []
        for line in block.splitlines():
            if not line.startswith('- `'):
                continue
            paths.append(line.split('`', 2)[1])
        return paths

    def test_prompt_input_manifest_agreement(self):
        compile_prompt = REPO_ROOT / '.claude/skills/compile-gear/prompt.md'
        emit_prompt = REPO_ROOT / '.claude/skills/emit-gear/prompt.md'
        compile_declared = self.declared_paths(
            compile_prompt,
            '**Read, in full, only these:**',
            '**Bundle consumption:**',
        )
        emit_declared = self.declared_paths(
            emit_prompt,
            '**Read, in this order:**',
            '**Bundle consumption:**',
        )

        self.assertEqual(
            compile_declared,
            [
                'spec/{{gear}}/instructions.md',
                'spec/{{gear}}/fusion.md',
                'spec/{{gear}}/contract.json',
                PACKER.provenance.PLAYBOOK,
                PACKER.FORMATS,
                PACKER.CONSTRUCTION,
                PACKER.CONSTRUCTION_EXAMPLE,
                *['{}/'.format(path) for path in PACKER.COMPILE_HARNESS_DIRECTORIES],
                PACKER.INVOLUTE,
            ],
        )
        self.assertIn(
            'Every Markdown document either prose source references by name',
            compile_prompt.read_text(encoding='utf-8'),
        )
        self.assertEqual(
            emit_declared,
            [
                'spec/{{gear}}/steps.md',
                'proof/{{gear}}/',
                '.tmp/{{gear}}.playbook-extract.md',
                '.tmp/{{gear}}.emitter-interfaces.md',
                PACKER.FORMATS,
                *PACKER.EMIT_FIXED_INPUTS[1:],
                *['{}/'.format(path) for path in PACKER.EMIT_FRAMEWORK_DIRECTORIES],
            ],
        )

    def test_bundle_consumption_instructions(self):
        for skill in ('compile-gear', 'emit-gear'):
            prompt = (REPO_ROOT / '.claude/skills' / skill / 'prompt.md').read_text(
                encoding='utf-8')
            owner = (REPO_ROOT / '.claude/skills' / skill / 'SKILL.md').read_text(
                encoding='utf-8')
            with self.subTest(skill=skill):
                for text in (prompt, owner):
                    normalized = ' '.join(text.split())
                    self.assertIn('@rendered-prompt', normalized)
                    self.assertIn('manifest.json', normalized)
                    self.assertIn('manifest order', normalized)
                    self.assertIn('Do not reopen', normalized)
                self.assertIn('sorted manifest file entries beneath that prefix', prompt)
                normalized_owner = ' '.join(owner.split())
                self.assertIn('complete outputs unchanged', normalized_owner)
                self.assertIn('undocumented dynamic input', normalized_owner)
                self.assertIn('Compare the first N bytes', normalized_owner)
                self.assertIn('complete retry rendering once', normalized_owner)
                self.assertIn('do not also send `@rendered-prompt` separately', normalized_owner)
                self.assertIn('does not reread packed sources', normalized_owner)

    def test_bundle_retry_uses_feedback_file_designated_by_owner(self):
        forbidden = ('.compile-gates.txt', '.gates.txt',
                     '.compile-feedback.txt', '.gates-feedback.txt')
        for skill in ('compile-gear', 'emit-gear'):
            owner = (REPO_ROOT / '.claude/skills' / skill / 'SKILL.md').read_text(
                encoding='utf-8')
            bundle_retry = owner.split('In the bundle condition,', 1)[1]
            bundle_retry = bundle_retry.split('\n\n', 1)[0]
            normalized = ' '.join(bundle_retry.split())
            with self.subTest(skill=skill):
                self.assertIn(
                    'retry feedback file that **Diagnose and loop** supplies to `--failure-file`',
                    normalized)
                self.assertIn('Render one fresh retry', normalized)
                for path in forbidden:
                    self.assertNotIn(path, bundle_retry)


if __name__ == '__main__':
    unittest.main()
