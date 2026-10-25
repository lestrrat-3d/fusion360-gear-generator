#!/usr/bin/env python3
"""Every add-in module compiles and imports with the Fusion modules stubbed.

The add-ins run only inside Fusion, so this cannot show that a Fusion call is right. It does catch a syntax error,
a missing name, a bad relative import or a broken generated file in any gear, which nothing else in CI reads. The
entry file Fusion loads (`Gear Generator.py`), `config.py` and the deployed `diag_*.py` scripts are compiled too, and
the entry file is imported. A MagicMock stub answers any attribute, so a separate check reads every `_fx.<name>` a gear
module uses and requires the runtime to define it.
"""
import ast
import compileall
import importlib
import importlib.util
import sys
import tempfile
import types
import unittest
from pathlib import Path
from unittest import mock

ROOT = Path(__file__).resolve().parents[1]
GEARS = ('spurgear', 'helicalgear', 'herringbonegear', 'bevelgear', 'cycloidal')


def missing_runtime_names(path, runtime):
    """The `_fx.<name>` attributes a module reads that the runtime module does not define, with their lines."""
    tree = ast.parse(path.read_text())
    return sorted({(node.attr, node.lineno) for node in ast.walk(tree)
                   if isinstance(node, ast.Attribute) and isinstance(node.value, ast.Name) and node.value.id == '_fx'
                   and not hasattr(runtime, node.attr)})


class AddInImportTest(unittest.TestCase):
    def test_every_python_file_compiles(self):
        for directory in ('lib', 'commands'):
            with self.subTest(directory=directory):
                self.assertTrue(compileall.compile_dir(
                    str(ROOT / directory), quiet=2, force=True, legacy=False,
                    workers=1, maxlevels=20), f'{directory} has a file that does not compile')

    def test_the_entry_file_config_and_diagnostics_compile(self):
        files = sorted(ROOT.glob('*.py'))
        names = [path.name for path in files]
        self.assertIn('Gear Generator.py', names)
        self.assertIn('config.py', names)
        self.assertTrue(any(name.startswith('diag_') for name in names))
        for path in files:
            with self.subTest(file=path.name):
                compile(path.read_text(encoding='utf-8'), str(path), 'exec')

    def test_every_gear_module_imports_with_stubbed_fusion(self):
        saved = {name: sys.modules.get(name) for name in list(sys.modules)
                 if name == 'adsk' or name.startswith(('adsk.', 'addin'))}
        try:
            adsk = types.ModuleType('adsk')
            adsk.__path__ = []
            for sub in ('core', 'fusion'):
                stub = mock.MagicMock(name=f'adsk.{sub}')
                sys.modules[f'adsk.{sub}'] = stub
                setattr(adsk, sub, stub)
            sys.modules['adsk'] = adsk
            spec = importlib.util.spec_from_file_location(
                'addin', ROOT / '__init__.py', submodule_search_locations=[str(ROOT)])
            package = importlib.util.module_from_spec(spec)
            sys.modules['addin'] = package
            for gear in GEARS:
                with self.subTest(gear=gear):
                    module = importlib.import_module(f'addin.lib.geargen.{gear}')
                    self.assertTrue(hasattr(module, 'GEAR_BUILD_ID'), f'{gear} lacks its build id')
            importlib.import_module('addin.commands')
            # The file Fusion loads, as a module of the add-in package so its relative imports resolve.
            entry_spec = importlib.util.spec_from_file_location('addin.gear_generator_entry', ROOT / 'Gear Generator.py')
            entry = importlib.util.module_from_spec(entry_spec)
            sys.modules[entry_spec.name] = entry
            entry_spec.loader.exec_module(entry)
            self.assertTrue(callable(entry.run) and callable(entry.stop))

            runtime = importlib.import_module('addin.lib.geargen.decadfusion')
            for path in sorted((ROOT / 'lib' / 'geargen').glob('*.py')):
                with self.subTest(module=path.name):
                    self.assertEqual([], missing_runtime_names(path, runtime),
                                     f'{path.name} uses names lib/geargen/decadfusion.py does not define')
            with tempfile.TemporaryDirectory() as directory:
                sample = Path(directory) / 'gear.py'
                sample.write_text('_fx.decad_union\n_fx.decad_not_there\n')
                self.assertEqual([('decad_not_there', 2)], missing_runtime_names(sample, runtime))
        finally:
            for name in [n for n in sys.modules if n == 'adsk' or n.startswith(('adsk.', 'addin'))]:
                del sys.modules[name]
            sys.modules.update({n: m for n, m in saved.items() if m is not None})


if __name__ == '__main__':
    unittest.main()
