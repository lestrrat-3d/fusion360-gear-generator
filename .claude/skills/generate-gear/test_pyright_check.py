"""Tests for the reusable Pyright analysis boundary and CLI setup failures."""
import importlib.util
import json
import os
import stat
import tempfile
import unittest
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
from unittest import mock


CHECKER = Path(__file__).with_name('pyright_check.py')
SPEC = importlib.util.spec_from_file_location('pyright_check_analysis_tests', CHECKER)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


FAKE_PYRIGHT = '''\
#!/usr/bin/env python3
import json
import os
import sys
import time

config = sys.argv[sys.argv.index("-p") + 1]
with open(config, encoding="utf-8") as handle:
    settings = json.load(handle)
target = os.path.abspath(os.path.join(os.path.dirname(config), settings["include"][0]))
with open(target, encoding="utf-8") as handle:
    source = handle.read()
time.sleep(0.05)
line = 1 if "first" in source else 2
print(json.dumps({"generalDiagnostics": [{
    "file": target,
    "severity": "error",
    "message": "fixture finding",
    "range": {"start": {"line": line, "character": 0},
              "end": {"line": line, "character": 1}},
    "rule": "reportUndefinedVariable"
}]}))
sys.exit(1)
'''


def executable(path, text):
    path.write_text(text, encoding='utf-8')
    path.chmod(path.stat().st_mode | stat.S_IEXEC)


class AnalysisFixtureTests(unittest.TestCase):
    def setUp(self):
        self.tmpdir = tempfile.TemporaryDirectory()
        self.root = Path(MODULE.repo_root())
        self.fixtures = Path(self.tmpdir.name) / 'candidate directory with spaces'
        self.fixtures.mkdir()
        self.defs = Path(self.tmpdir.name) / 'defs'
        (self.defs / 'adsk').mkdir(parents=True)
        (self.defs / 'adsk' / 'core.py').write_text('', encoding='utf-8')
        self.pyright = Path(self.tmpdir.name) / 'fake pyright'
        executable(self.pyright, FAKE_PYRIGHT)

    def tearDown(self):
        self.tmpdir.cleanup()

    def analyze(self, paths):
        return MODULE.analyze_paths(
            paths, root=str(self.root), stubs=str(self.defs),
            pyright_argv=[str(self.pyright)])

    def test_external_candidate_preserves_path_and_relative_config(self):
        candidate = self.fixtures / 'candidate.py'
        candidate.write_text('first = 1\n', encoding='utf-8')

        result = self.analyze([candidate])

        self.assertIsNone(result.setup_error)
        self.assertEqual(list(result.diagnostics), [str(candidate.resolve())])
        self.assertEqual(result.diagnostics[str(candidate.resolve())][0]['file'], str(candidate.resolve()))
        self.assertEqual(result.diagnostics[str(candidate.resolve())][0]['range']['start']['line'], 1)
        self.assertNotEqual(result.metadata.invocations[0].target_path, str(candidate.resolve()))
        self.assertTrue(os.path.relpath(
            result.metadata.invocations[0].target_path,
            os.path.dirname(result.metadata.invocations[0].config_path)).startswith('../'))
        self.assertEqual(list(Path(self.root / 'lib' / 'geargen').glob('.pyright-candidate-*')), [])
        self.assertEqual(list(Path(self.root / '.tmp').glob('.pyright-check-*')), [])

    def test_two_concurrent_analyses_use_independent_scratch_files(self):
        first = self.fixtures / 'first.py'
        second = self.fixtures / 'second.py'
        first.write_text('first = 1\n', encoding='utf-8')
        second.write_text('second = 1\n', encoding='utf-8')

        with ThreadPoolExecutor(max_workers=2) as pool:
            results = list(pool.map(self.analyze, ([first], [second])))

        self.assertEqual([r.diagnostics[str(path.resolve())][0]['range']['start']['line']
                          for r, path in zip(results, (first, second))], [1, 2])
        configs = [r.metadata.invocations[0].config_path for r in results]
        targets = [r.metadata.invocations[0].target_path for r in results]
        self.assertEqual(len(set(configs)), 2)
        self.assertEqual(len(set(targets)), 2)
        self.assertTrue(all(not os.path.exists(path) for path in configs + targets))

    def test_exit_one_with_valid_json_is_successful_analysis(self):
        candidate = self.fixtures / 'candidate.py'
        candidate.write_text('first = 1\n', encoding='utf-8')

        result = self.analyze([candidate])

        self.assertTrue(result.ok)
        self.assertEqual(result.metadata.invocations[0].exit_code, 1)

    def test_invalid_stubs_and_malformed_output_are_setup_errors(self):
        candidate = self.fixtures / 'candidate.py'
        candidate.write_text('first = 1\n', encoding='utf-8')
        invalid = Path(self.tmpdir.name) / 'invalid stubs'
        invalid.mkdir()
        invalid_result = MODULE.analyze_paths(
            [candidate], root=str(self.root), stubs=str(invalid), pyright_argv=[str(self.pyright)])
        self.assertIn('no adsk/core.py', invalid_result.setup_error)

        malformed = Path(self.tmpdir.name) / 'malformed pyright'
        executable(malformed, '#!/usr/bin/env python3\nprint("not json")\n')
        malformed_result = MODULE.analyze_paths(
            [candidate], root=str(self.root), stubs=str(self.defs), pyright_argv=[str(malformed)])
        self.assertIn('malformed JSON', malformed_result.setup_error)
        self.assertEqual(list(Path(self.root / '.tmp').glob('.pyright-check-*')), [])
        self.assertEqual(list(Path(self.root / 'lib' / 'geargen').glob('.pyright-candidate-*')), [])

    def test_executable_failure_is_a_setup_error(self):
        candidate = self.fixtures / 'candidate.py'
        candidate.write_text('first = 1\n', encoding='utf-8')

        with mock.patch.object(MODULE.subprocess, 'run', side_effect=OSError('cannot execute')):
            failed = self.analyze([candidate])
        self.assertIn('failed to execute', failed.setup_error)


if __name__ == '__main__':
    unittest.main()
