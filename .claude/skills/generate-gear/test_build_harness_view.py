"""Focused checks for the compiler's shared proof harness API view."""
import importlib.util
import tempfile
import unittest
from pathlib import Path


SCRIPT = Path(__file__).with_name('build_harness_view.py')
spec = importlib.util.spec_from_file_location('build_harness_view', SCRIPT)
view_tool = importlib.util.module_from_spec(spec)
spec.loader.exec_module(view_tool)


class HarnessViewTests(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        self.root = Path(temporary.name)
        (self.root / '.tmp').mkdir()
        for package in view_tool.PACKAGES:
            (self.root / 'proof' / package).mkdir(parents=True)
        (self.root / 'proof/go.mod').write_text('module example.com/proof\n\ngo 1.22\n', encoding='utf-8')
        (self.root / 'proof/proofkit/proofkit.go').write_text(
            'package proofkit\n'
            '// Case carries proof inputs.\n'
            'type Case struct { Name string }\n'
            '// Run proves every case with a fresh sketch.\n'
            'func Run(cases []Case) {}\n'
            'func hiddenCall() {}\n', encoding='utf-8')
        (self.root / 'proof/proofkit3d/proofkit3d.go').write_text(
            'package proofkit3d\n'
            '// RunSolid proves every case with a bounded-solid gate.\n'
            'func RunSolid() {}\n', encoding='utf-8')
        (self.root / 'proof/proofkit/proofkit_test.go').write_text(
            'package proofkit\nfunc testOnly() {}\n', encoding='utf-8')

    def test_real_go_doc_preserves_public_semantics_without_test_bodies(self):
        view, _ = view_tool.build(self.root)
        text = view.decode()
        self.assertIn('Case carries proof inputs', text)
        self.assertIn('Run proves every case with a fresh sketch', text)
        self.assertIn('RunSolid proves every case with a bounded-solid gate', text)
        self.assertNotIn('hiddenCall', text)
        self.assertNotIn('testOnly', text)

    def test_reuse_and_verify_detect_source_or_view_changes(self):
        self.assertEqual(0, view_tool.main(['build', '--root', str(self.root)]))
        self.assertEqual(0, view_tool.main(['verify', '--root', str(self.root)]))
        view = self.root / '.tmp/harness-api-view.md'
        before = view.stat().st_mtime_ns
        self.assertEqual(0, view_tool.main(['build', '--root', str(self.root)]))
        self.assertEqual(before, view.stat().st_mtime_ns)
        view.write_bytes(view.read_bytes() + b'changed')
        self.assertEqual(2, view_tool.main(['verify', '--root', str(self.root)]))
        self.assertEqual(0, view_tool.main(['build', '--root', str(self.root)]))
        source = self.root / 'proof/proofkit/proofkit_test.go'
        source.write_bytes(source.read_bytes() + b'// new assertion\n')
        self.assertEqual(2, view_tool.main(['verify', '--root', str(self.root)]))


if __name__ == '__main__':
    unittest.main()
