#!/usr/bin/env python3
"""Regression tests for the pyright resolution/bootstrap module.

No test here touches the network or runs a real pip: `_pip_install` (or the `subprocess.run`
inside it) is always stubbed.
"""
import importlib.util
import os
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path
from unittest import mock


BOOT = Path(__file__).with_name('pyright_boot.py')
SPEC = importlib.util.spec_from_file_location('pyright_boot', BOOT)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def make_marker(target):
    """Create the file the cached-install branch looks for."""
    pkg = Path(target) / 'pyright'
    pkg.mkdir(parents=True, exist_ok=True)
    (pkg / '__init__.py').write_text('')


class CachePkgDirTests(unittest.TestCase):
    def test_honours_xdg_cache_home(self):
        with tempfile.TemporaryDirectory() as directory:
            with mock.patch.dict(os.environ, {'XDG_CACHE_HOME': directory}):
                self.assertEqual(
                    MODULE.cache_pkg_dir(),
                    os.path.join(directory, 'fusion360-gear-generator', 'pyright-pkg'))

    def test_falls_back_to_home_cache(self):
        env = dict(os.environ)
        env.pop('XDG_CACHE_HOME', None)
        with mock.patch.dict(os.environ, env, clear=True):
            self.assertEqual(
                MODULE.cache_pkg_dir(),
                os.path.join(os.path.expanduser('~'), '.cache',
                             'fusion360-gear-generator', 'pyright-pkg'))


class PythonPathTests(unittest.TestCase):
    def test_prepends_to_existing_pythonpath(self):
        with mock.patch.dict(os.environ, {'PYTHONPATH': '/already/here'}):
            self.assertEqual(MODULE._pythonpath_with('/new'),
                             '/new' + os.pathsep + '/already/here')

    def test_returns_bare_target_when_unset(self):
        env = dict(os.environ)
        env.pop('PYTHONPATH', None)
        with mock.patch.dict(os.environ, env, clear=True):
            self.assertEqual(MODULE._pythonpath_with('/new'), '/new')


class ResolutionOrderTests(unittest.TestCase):
    def setUp(self):
        patch = mock.patch.object(MODULE, '_pip_install')
        self.pip = patch.start()
        self.addCleanup(patch.stop)

    def test_importable_module_wins(self):
        with mock.patch.object(MODULE.importlib.util, 'find_spec', return_value=object()):
            argv, extra = MODULE.resolve_pyright()
        self.assertEqual(argv, [sys.executable, '-m', 'pyright'])
        self.assertEqual(extra, {})
        self.pip.assert_not_called()

    def test_path_binary_used_when_module_absent(self):
        with mock.patch.object(MODULE.importlib.util, 'find_spec', return_value=None):
            with mock.patch.object(MODULE.shutil, 'which', return_value='/usr/bin/pyright'):
                argv, extra = MODULE.resolve_pyright()
        self.assertEqual(argv, ['/usr/bin/pyright'])
        self.assertEqual(extra, {})
        self.pip.assert_not_called()

    def test_cached_target_install_used_before_installing(self):
        with tempfile.TemporaryDirectory() as directory:
            with mock.patch.dict(os.environ, {'XDG_CACHE_HOME': directory}):
                target = MODULE.cache_pkg_dir()
                make_marker(target)
                with mock.patch.object(MODULE.importlib.util, 'find_spec', return_value=None):
                    with mock.patch.object(MODULE.shutil, 'which', return_value=None):
                        argv, extra = MODULE.resolve_pyright()
                self.assertEqual(argv, [sys.executable, '-m', 'pyright'])
                self.assertTrue(extra['PYTHONPATH'].startswith(target))
        self.pip.assert_not_called()

    def test_no_install_refuses_to_provision(self):
        with tempfile.TemporaryDirectory() as directory:
            with mock.patch.dict(os.environ, {'XDG_CACHE_HOME': directory}):
                with mock.patch.object(MODULE.importlib.util, 'find_spec', return_value=None):
                    with mock.patch.object(MODULE.shutil, 'which', return_value=None):
                        with self.assertRaises(MODULE.PyrightUnavailable) as caught:
                            MODULE.resolve_pyright(no_install=True)
        self.assertIn('--no-install', str(caught.exception))
        self.assertIn('pip install', str(caught.exception))
        self.pip.assert_not_called()

    def test_missing_pyright_is_installed_into_the_cache(self):
        self.pip.side_effect = make_marker
        with tempfile.TemporaryDirectory() as directory:
            with mock.patch.dict(os.environ, {'XDG_CACHE_HOME': directory}):
                target = MODULE.cache_pkg_dir()
                with mock.patch.object(MODULE.importlib.util, 'find_spec', return_value=None):
                    with mock.patch.object(MODULE.shutil, 'which', return_value=None):
                        argv, extra = MODULE.resolve_pyright(quiet=True)
                self.pip.assert_called_once_with(target)
                self.assertEqual(argv, [sys.executable, '-m', 'pyright'])
                self.assertTrue(extra['PYTHONPATH'].startswith(target))
                self.assertTrue(os.path.isfile(os.path.join(target, 'pyright', '__init__.py')))


class InstallFailureTests(unittest.TestCase):
    def test_failed_install_is_cleaned_up_and_reported(self):
        with tempfile.TemporaryDirectory() as directory:
            target = os.path.join(directory, 'pyright-pkg')
            os.makedirs(target)
            error = subprocess.CalledProcessError(1, 'pip', stderr='boom')
            with mock.patch.object(MODULE.subprocess, 'run', side_effect=error):
                with self.assertRaises(MODULE.PyrightUnavailable) as caught:
                    MODULE._pip_install(target)
            self.assertFalse(os.path.exists(target))
        self.assertIn('boom', str(caught.exception))


if __name__ == '__main__':
    unittest.main()
