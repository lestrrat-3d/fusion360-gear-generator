"""Focused checks for the emitter-facing compiled step view."""
import importlib.util
import json
import tempfile
import unittest
from pathlib import Path


SCRIPT = Path(__file__).with_name('build_steps_view.py')
spec = importlib.util.spec_from_file_location('build_steps_view', SCRIPT)
view_tool = importlib.util.module_from_spec(spec)
spec.loader.exec_module(view_tool)


class StepsViewTests(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        self.root = Path(temporary.name)
        (self.root / '.tmp').mkdir()
        (self.root / 'spec/gear').mkdir(parents=True)
        self.source = self.root / 'spec/gear/steps.md'
        metadata = {
            'schema': 2,
            'citations': [{'path': 'spec/gear/instructions.md', 'first': 1, 'last': 2}],
            'calls': [{'span': 'sketch.project(anchor)', 'name': 'project', 'receiver': 'sketch',
                       'owner': 'adsk.fusion.Sketch', 'role': 'required',
                       'condition': 'anchor exists', 'reason': None}],
        }
        self.source.write_text(
            'The inherited pipeline stays inherited.\n\n'
            '<!-- step-metadata: 2 -->\n\n'
            '## Provenance\n\n| file | hash |\n|---|---|\n| a | b |\n\n'
            '## Compilation contract\n\n```json\n{"contract": "keep"}\n```\n\n'
            '## Exact values\n\n```json\n{"inputs": ["omit"]}\n```\n\n'
            '## 1 `[GO]` Draw\n\nKeep geometry and `sketch.project(anchor)`.\n\n'
            '<!-- step-meta\n' + json.dumps(metadata) + '\n-->\n\n'
            '**From:** `spec/gear/instructions.md` L1–2.\n',
            encoding='utf-8')

    def test_preserves_instructions_contract_and_call_conditions(self):
        view, raw_manifest = view_tool.build(self.root, 'gear')
        text = view.decode()
        self.assertIn('{"contract": "keep"}', text)
        self.assertIn('The inherited pipeline stays inherited.', text)
        self.assertIn('Keep geometry and `sketch.project(anchor)`.', text)
        self.assertIn('`required`: `sketch.project(anchor)` on `adsk.fusion.Sketch` '
                      'when anchor exists', text)
        self.assertIn('## Deterministic setup', text)
        self.assertNotIn('"inputs": ["omit"]', text)
        self.assertNotIn('## Provenance', text)
        self.assertNotIn('**From:**', text)
        self.assertNotIn('<!-- step-meta', text)
        self.assertEqual(json.loads(raw_manifest)['steps'], 1)

    def test_verify_detects_source_and_view_changes(self):
        self.assertEqual(0, view_tool.main(['build', 'gear', '--root', str(self.root)]))
        self.assertEqual(0, view_tool.main(['verify', 'gear', '--root', str(self.root)]))
        view = self.root / '.tmp/gear.steps-view.md'
        before = view.stat().st_mtime_ns
        self.assertEqual(0, view_tool.main(['build', 'gear', '--root', str(self.root)]))
        self.assertEqual(before, view.stat().st_mtime_ns)
        view.write_bytes(view.read_bytes() + b'changed')
        self.assertEqual(2, view_tool.main(['verify', 'gear', '--root', str(self.root)]))
        self.assertEqual(0, view_tool.main(['build', 'gear', '--root', str(self.root)]))
        self.source.write_bytes(self.source.read_bytes() + b'\nChanged source.\n')
        self.assertEqual(2, view_tool.main(['verify', 'gear', '--root', str(self.root)]))

    def test_unknown_pre_step_section_is_rejected(self):
        text = self.source.read_text(encoding='utf-8')
        self.source.write_text(text.replace('## Provenance', '## Extra rules\n\nKeep this.\n\n'
                                            '## Provenance'), encoding='utf-8')
        with self.assertRaisesRegex(view_tool.ViewError, 'unrecognized pre-step section'):
            view_tool.build(self.root, 'gear')


if __name__ == '__main__':
    unittest.main()
