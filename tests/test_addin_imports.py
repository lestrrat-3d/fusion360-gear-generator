#!/usr/bin/env python3
"""Every add-in module compiles and imports with the Fusion modules stubbed.

The add-ins run only inside Fusion, so this cannot show that a Fusion call is right. It does catch a syntax error,
a missing name, a bad relative import or a broken generated file in any gear, which nothing else in CI reads.
"""
import compileall
import importlib
import importlib.util
import sys
import types
import unittest
from pathlib import Path
from unittest import mock

ROOT = Path(__file__).resolve().parents[1]
GEARS = ('spurgear', 'helicalgear', 'herringbonegear', 'bevelgear', 'cycloidal')


class AddInImportTest(unittest.TestCase):
    def test_every_python_file_compiles(self):
        for directory in ('lib', 'commands'):
            with self.subTest(directory=directory):
                self.assertTrue(compileall.compile_dir(
                    str(ROOT / directory), quiet=2, force=True, legacy=False,
                    workers=1, maxlevels=20), f'{directory} has a file that does not compile')

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
        finally:
            for name in [n for n in sys.modules if n == 'adsk' or n.startswith(('adsk.', 'addin'))]:
                del sys.modules[name]
            sys.modules.update({n: m for n, m in saved.items() if m is not None})


if __name__ == '__main__':
    unittest.main()
