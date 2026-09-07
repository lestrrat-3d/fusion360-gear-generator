#!/usr/bin/env python3
"""Regressions for the typed emitter interface sheet."""
import contextlib
import importlib.util
import io
import json
import tempfile
import unittest
from pathlib import Path
from unittest import mock


RENDERER_PATH = Path(__file__).with_name('render_emitter_interfaces.py')
MODULE_SPEC = importlib.util.spec_from_file_location('render_emitter_interfaces', RENDERER_PATH)
RENDERER = importlib.util.module_from_spec(MODULE_SPEC)
MODULE_SPEC.loader.exec_module(RENDERER)


BASE = """
class Generator:
    def generate(self, inputs: adsk.core.CommandInputs):
        raise NotImplementedError
"""

COMMAND = """
class GearCommand:
    def command_created(self, args: adsk.core.CommandCreatedEventArgs):
        self.configurator.configure(args.command)

    def command_execute(self, args: adsk.core.CommandEventArgs):
        self.generator_class().generate(args.command.commandInputs)
"""

MANIFEST = {
    'module': 'lib/geargen/fixturegear.py',
    'classes': {
        'FixtureConfigurator': {
            'bases': ['ParentConfigurator'],
            'methods': ['configure'],
        },
        'FixtureGenerator': {
            'bases': ['Generator'],
            'methods': ['__init__', 'generate', 'cleanup'],
        },
    },
}


class RenderEmitterInterfacesTest(unittest.TestCase):
    def repo(self, base=BASE, command=COMMAND, manifest=MANIFEST):
        directory = tempfile.TemporaryDirectory()
        root = Path(directory.name)
        (root / 'lib' / 'geargen').mkdir(parents=True)
        (root / 'commands').mkdir()
        (root / 'spec' / 'fixturegear').mkdir(parents=True)
        (root / 'lib' / 'geargen' / 'base.py').write_text(base, encoding='utf-8')
        (root / 'commands' / '_gear_command.py').write_text(command, encoding='utf-8')
        if manifest is not None:
            (root / 'spec' / 'fixturegear' / 'contract.json').write_text(
                json.dumps(manifest), encoding='utf-8')
        self.addCleanup(directory.cleanup)
        return root

    def run_main(self, root, output=None, gear='fixturegear'):
        output = output or root / 'sheet.md'
        stdout, stderr = io.StringIO(), io.StringIO()
        with contextlib.redirect_stdout(stdout), contextlib.redirect_stderr(stderr):
            code = RENDERER.main(
                ['render_emitter_interfaces.py', gear, '--out', str(output)],
                repo_root=root)
        return code, output, stdout.getvalue(), stderr.getvalue()

    def test_configure_signature(self):
        root = self.repo()
        code, output, _stdout, stderr = self.run_main(root)

        self.assertEqual(code, 0, stderr)
        self.assertIn(
            'class FixtureConfigurator(ParentConfigurator):\n'
            '    @classmethod\n'
            '    def configure(cls, cmd: adsk.core.Command) -> None: ...',
            output.read_text(encoding='utf-8'))

    def test_generate_signature_follows_base(self):
        root = self.repo(base="""
class Generator:
    def generate(self, command_inputs: adsk.core.CommandInputs):
        raise RuntimeError('must not execute')
""")
        code, output, _stdout, stderr = self.run_main(root)

        self.assertEqual(code, 0, stderr)
        text = output.read_text(encoding='utf-8')
        self.assertIn('class FixtureGenerator(Generator):', text)
        self.assertIn(
            'def generate(self, command_inputs: adsk.core.CommandInputs): ...', text)
        self.assertNotIn('def generate(self, inputs:', text)
        self.assertIn('Other required methods:\n\n- `__init__`\n- `cleanup`', text)

    def test_configure_callsite_drift(self):
        root = self.repo(command="""
class GearCommand:
    def command_created(self, args: adsk.core.CommandCreatedEventArgs):
        self.configurator.configure(self.command)
""")
        output = root / 'sheet.md'
        output.write_text('keep me\n', encoding='utf-8')

        code, _, _stdout, stderr = self.run_main(root, output)

        self.assertEqual(code, 2)
        self.assertIn('self.configurator.configure(args.command)', stderr)
        self.assertEqual(output.read_text(encoding='utf-8'), 'keep me\n')

    def test_no_adsk_import_execution(self):
        root = self.repo(
            base="""
raise RuntimeError('imported base')
class Generator:
    def generate(self, inputs: adsk.core.CommandInputs):
        pass
""",
            command="""
raise RuntimeError('imported command')
class GearCommand:
    def command_created(self, event: adsk.core.CommandCreatedEventArgs):
        self.configurator.configure(event.command)
""")

        code, output, _stdout, stderr = self.run_main(root)

        self.assertEqual(code, 0, stderr)
        self.assertIn('FixtureConfigurator', output.read_text(encoding='utf-8'))

    def test_idempotent_sheet(self):
        root = self.repo()
        first = root / 'first.md'
        second = root / 'second.md'

        first_code, _, _stdout, first_stderr = self.run_main(root, first)
        second_code, _, _stdout, second_stderr = self.run_main(root, second)

        self.assertEqual(first_code, 0, first_stderr)
        self.assertEqual(second_code, 0, second_stderr)
        self.assertEqual(first.read_bytes(), second.read_bytes())

    def test_missing_manifest_gets_base_signature_and_conventions(self):
        root = self.repo(manifest=None)

        code, output, _stdout, stderr = self.run_main(root)

        self.assertEqual(code, 0, stderr)
        text = output.read_text(encoding='utf-8')
        self.assertIn('class Generator:', text)
        self.assertIn('def generate(self, inputs: adsk.core.CommandInputs): ...', text)
        self.assertIn('## Typing conventions', text)
        self.assertNotIn('FixtureConfigurator', text)

    def test_invalid_manifest_leaves_output_unchanged(self):
        root = self.repo(manifest={'classes': {'Fixture': {'unknown': []}}})
        output = root / 'sheet.md'
        output.write_text('keep me\n', encoding='utf-8')

        code, _, _stdout, stderr = self.run_main(root, output)

        self.assertEqual(code, 2)
        self.assertIn('unknown field', stderr)
        self.assertEqual(output.read_text(encoding='utf-8'), 'keep me\n')

    def test_async_framework_methods_leave_output_unchanged(self):
        cases = (
            ("""
class Generator:
    async def generate(self, inputs: adsk.core.CommandInputs):
        pass
""", COMMAND, 'Generator.generate must be synchronous'),
            (BASE, """
class GearCommand:
    async def command_created(self, args: adsk.core.CommandCreatedEventArgs):
        self.configurator.configure(args.command)
""", 'GearCommand.command_created must be synchronous'),
        )
        for base, command, expected in cases:
            with self.subTest(expected=expected):
                root = self.repo(base=base, command=command)
                output = root / 'sheet.md'
                output.write_text('keep me\n', encoding='utf-8')

                code, _, _stdout, stderr = self.run_main(root, output)

                self.assertEqual(code, 2)
                self.assertIn(expected, stderr)
                self.assertEqual(output.read_text(encoding='utf-8'), 'keep me\n')

    def test_failed_atomic_replace_leaves_output_and_no_temporary_file(self):
        root = self.repo()
        output = root / 'sheet.md'
        output.write_text('keep me\n', encoding='utf-8')

        with mock.patch.object(RENDERER.os, 'replace', side_effect=OSError('replace failed')):
            code, _, _stdout, stderr = self.run_main(root, output)

        self.assertEqual(code, 2)
        self.assertIn('replace failed', stderr)
        self.assertEqual(output.read_text(encoding='utf-8'), 'keep me\n')
        self.assertEqual(list(root.glob('.sheet.md.*')), [])

    def test_invalid_gear_name_leaves_output_unchanged(self):
        root = self.repo()
        output = root / 'sheet.md'
        output.write_text('keep me\n', encoding='utf-8')

        code, _, _stdout, stderr = self.run_main(root, output, gear='fixturegear\n')

        self.assertEqual(code, 2)
        self.assertIn(RENDERER.USAGE, stderr)
        self.assertEqual(output.read_text(encoding='utf-8'), 'keep me\n')


if __name__ == '__main__':
    unittest.main()
