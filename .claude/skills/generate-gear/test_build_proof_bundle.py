"""Focused tests for checked construction proof extraction."""
import importlib.util
import json
import tempfile
import unittest
from pathlib import Path

SCRIPT = Path(__file__).with_name('build_proof_bundle.py')
spec = importlib.util.spec_from_file_location('build_proof_bundle', SCRIPT)
bundle_tool = importlib.util.module_from_spec(spec)
spec.loader.exec_module(bundle_tool)


class ProofBundleTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        (self.root / '.tmp').mkdir()
        (self.root / 'proof/spurgear').mkdir(parents=True)
        (self.root / 'spec/spurgear').mkdir(parents=True)
        (self.root / 'spec/spurgear/steps.md').write_text(
            '## 3 `[GO]` Draw\n\nProof function `stepDraw` in '
            '`proof/spurgear/draw_test.go`.\n', encoding='utf-8')
        (self.root / 'proof/spurgear/draw_test.go').write_text(
            'package spurgear_test\n\n'
            '// helper comment stays with the source.\n'
            'func helper() int { return shared() }\n\n'
            'func stepDraw() int { requiredAssertion(); return helper() }\n\n'
            'func requiredAssertion() bool { return true }\n\n'
            'func irrelevantAssertion() bool { return false }\n', encoding='utf-8')
        (self.root / 'proof/spurgear/other_test.go').write_text(
            'package spurgear_test\n\nfunc shared() int { return 3 }\n', encoding='utf-8')
        (self.root / 'proof/spurgear/zz_registrations_test.go').write_text(
            'package spurgear_test\n\nfunc TestDraw() { stepDraw() }\n', encoding='utf-8')

    def test_exact_source_and_cross_file_closure(self):
        bundle, raw_manifest = bundle_tool.build(self.root, 'spurgear')
        view = bundle.decode('utf-8')
        manifest = json.loads(raw_manifest)
        self.assertIn('func stepDraw() int { requiredAssertion(); return helper() }', view)
        self.assertIn('func requiredAssertion() bool { return true }', view)
        self.assertIn('// helper comment stays with the source.', view)
        self.assertIn('func shared() int { return 3 }', view)
        self.assertNotIn('func irrelevantAssertion()', view)
        self.assertNotIn('func TestDraw()', view)
        self.assertEqual([{'step': '3', 'function': 'stepDraw',
                           'file': 'proof/spurgear/draw_test.go'}], manifest['coverage'])
        for source in manifest['files']:
            data = (self.root / source['path']).read_bytes()
            self.assertEqual(len(data), sum(item['end'] - item['start']
                                            for item in source['ranges']))
            self.assertEqual(bundle_tool.digest(data), source['sha256'])
            for item in source['ranges']:
                self.assertEqual(bundle_tool.digest(data[item['start']:item['end']]),
                                 item['sha256'])

    def test_missing_step_function_or_registration_fails(self):
        (self.root / 'proof/spurgear/draw_test.go').write_text(
            'package spurgear_test\nfunc helper() int { return 3 }\n', encoding='utf-8')
        with self.assertRaisesRegex(bundle_tool.BundleError, 'missing proof function'):
            bundle_tool.build(self.root, 'spurgear')
        (self.root / 'proof/spurgear/draw_test.go').write_text(
            'package spurgear_test\nfunc stepDraw() int { return 3 }\n', encoding='utf-8')
        (self.root / 'proof/spurgear/zz_registrations_test.go').write_text(
            'package spurgear_test\n', encoding='utf-8')
        with self.assertRaisesRegex(bundle_tool.BundleError, 'checked registration'):
            bundle_tool.build(self.root, 'spurgear')

    def test_missing_step_reference_fails(self):
        (self.root / 'spec/spurgear/steps.md').write_text(
            '## 3 `[GO]` Draw\n\nNo proof reference.\n', encoding='utf-8')
        with self.assertRaisesRegex(bundle_tool.BundleError, 'exactly one Proof function'):
            bundle_tool.build(self.root, 'spurgear')

    def test_unresolved_cross_file_symbol_fails(self):
        source = self.root / 'proof/spurgear/other_test.go'
        source.write_text('package spurgear_test\nfunc shared() int { return 3 }\n'
                          'func another() int { return absent() }\n',
                          encoding='utf-8')
        with self.assertRaisesRegex(bundle_tool.BundleError, 'unresolved proof symbol.*absent'):
            bundle_tool.build(self.root, 'spurgear')

    def test_changed_source_and_bundle_fail_verification(self):
        self.assertEqual(0, bundle_tool.main(['build', 'spurgear', '--root', str(self.root)]))
        self.assertEqual(0, bundle_tool.main(['verify', 'spurgear', '--root', str(self.root)]))
        source = self.root / 'proof/spurgear/other_test.go'
        source.write_bytes(source.read_bytes() + b'\n// changed\n')
        self.assertEqual(2, bundle_tool.main(['verify', 'spurgear', '--root', str(self.root)]))


if __name__ == '__main__':
    unittest.main()
