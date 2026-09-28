"""Focused tests for the checked gear dependency API view."""
import importlib.util
import json
import tempfile
import unittest
from pathlib import Path


SCRIPT = Path(__file__).with_name('build_dependency_view.py')
spec = importlib.util.spec_from_file_location('build_dependency_view', SCRIPT)
view_tool = importlib.util.module_from_spec(spec)
spec.loader.exec_module(view_tool)


class DependencyViewTests(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        self.root = Path(temporary.name)
        (self.root / '.tmp').mkdir()
        (self.root / 'spec/helicalgear').mkdir(parents=True)
        (self.root / 'lib/geargen').mkdir(parents=True)
        (self.root / 'spec/helicalgear/steps.md').write_text(
            '## 1 `[PROSE]` Subclass SpurConfigurator and use PARAM_MODULE.\n', encoding='utf-8')
        (self.root / 'lib/geargen/spurgear.py').write_text(
            "PARAM_MODULE = 'Module'\n"
            'class SpurConfigurator:\n'
            '    @classmethod\n'
            '    def configure(cls, cmd):\n'
            '        expensive_body()\n'
            'class Unrelated:\n'
            '    def skip(self): pass\n', encoding='utf-8')
        (self.root / 'lib/geargen/helicalgear.py').write_text(
            'class SpurConfigurator:\n    def configure(self, cmd): pass\n', encoding='utf-8')

    def test_real_signatures_without_bodies_or_target_module(self):
        view, raw_manifest = view_tool.build(self.root, 'helicalgear')
        text = view.decode()
        manifest = json.loads(raw_manifest)
        self.assertIn('PARAM_MODULE = \'Module\'', text)
        self.assertIn('@classmethod\n    def configure(cls, cmd): ...', text)
        self.assertNotIn('expensive_body()', text)
        self.assertNotIn('class Unrelated', text)
        self.assertEqual([item['path'] for item in manifest['files']],
                         ['lib/geargen/spurgear.py'])

    def test_verify_rejects_changed_source_or_view(self):
        self.assertEqual(0, view_tool.main(['build', 'helicalgear', '--root', str(self.root)]))
        self.assertEqual(0, view_tool.main(['verify', 'helicalgear', '--root', str(self.root)]))
        source = self.root / 'lib/geargen/spurgear.py'
        source.write_bytes(source.read_bytes() + b'\n# changed\n')
        self.assertEqual(2, view_tool.main(['verify', 'helicalgear', '--root', str(self.root)]))
        self.assertEqual(0, view_tool.main(['build', 'helicalgear', '--root', str(self.root)]))
        view = self.root / '.tmp/helicalgear.dependency-view.md'
        view.write_bytes(view.read_bytes() + b'changed')
        self.assertEqual(2, view_tool.main(['verify', 'helicalgear', '--root', str(self.root)]))


if __name__ == '__main__':
    unittest.main()
