#!/usr/bin/env python3
"""Regression tests for stage.py, the drafted-artifact placement script."""
import contextlib
import importlib.util
import io
import os
import stat
import subprocess
import tempfile
import unittest
from pathlib import Path
from unittest import mock

MODULE_PATH = Path(__file__).with_name('stage.py')
SPEC = importlib.util.spec_from_file_location('stage', MODULE_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def make_root(tmp, gear='spurgear', draft=None, placed=None, steps=None, module=None):
    """Build a fake repo root: .tmp/, proof/<gear>/, spec/<gear>/, lib/geargen/, proof/run.sh."""
    root = Path(tmp)
    (root / '.tmp').mkdir(parents=True, exist_ok=True)
    (root / 'proof' / gear).mkdir(parents=True, exist_ok=True)
    (root / 'spec' / gear).mkdir(parents=True, exist_ok=True)
    (root / 'lib' / 'geargen').mkdir(parents=True, exist_ok=True)
    proof_dir = root / 'proof'
    proof_dir.mkdir(parents=True, exist_ok=True)
    run_sh = proof_dir / 'run.sh'
    run_sh.write_text('#!/usr/bin/env bash\necho "run.sh not configured"\nexit 0\n')
    run_sh.chmod(run_sh.stat().st_mode | stat.S_IEXEC)

    if draft is not None:
        draft_dir = root / '.tmp' / ('%s-proof' % gear)
        draft_dir.mkdir(parents=True, exist_ok=True)
        for name, content in draft.items():
            path = draft_dir / name
            path.parent.mkdir(parents=True, exist_ok=True)
            if isinstance(content, bytes):
                path.write_bytes(content)
            else:
                path.write_text(content)

    if placed is not None:
        placed_dir = root / 'proof' / gear
        for name, content in placed.items():
            path = placed_dir / name
            if isinstance(content, bytes):
                path.write_bytes(content)
            else:
                path.write_text(content)

    if steps is not None:
        (root / '.tmp' / ('%s.steps.md' % gear)).write_text(steps)

    if module is not None:
        (root / '.tmp' / ('%s.generated.py' % gear)).write_text(module)

    return root


def run_stage(root, *argv):
    out, err = io.StringIO(), io.StringIO()
    with contextlib.redirect_stdout(out), contextlib.redirect_stderr(err):
        code = MODULE.main(['stage.py', '--root', str(root), *argv])
    return code, out.getvalue(), err.getvalue()


def git_init(root):
    subprocess.run(['git', 'init', '--quiet'], cwd=str(root), check=True)
    subprocess.run(['git', 'config', 'user.email', 'fixture@example.invalid'],
                    cwd=str(root), check=True)
    subprocess.run(['git', 'config', 'user.name', 'fixture'], cwd=str(root), check=True)


def git_commit_all(root, message='fixture'):
    subprocess.run(['git', 'add', '-A'], cwd=str(root), check=True)
    subprocess.run(['git', 'commit', '--quiet', '-m', message], cwd=str(root), check=True)


class PlacementTests(unittest.TestCase):
    def test_proof_copies_every_drafted_go_file_into_empty_destination(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = make_root(tmp, draft={'a_test.go': 'package spurgear_test\n',
                                          'b_test.go': 'package spurgear_test\n'})
            code, out, _ = run_stage(root, 'spurgear', 'proof')
            self.assertEqual(code, 0)
            self.assertIn('wrote', out)
            dest = root / 'proof' / 'spurgear'
            self.assertEqual((dest / 'a_test.go').read_text(), 'package spurgear_test\n')
            self.assertEqual((dest / 'b_test.go').read_text(), 'package spurgear_test\n')

    def test_proof_reports_unchanged_and_does_not_rewrite_identical_bytes(self):
        with tempfile.TemporaryDirectory() as tmp:
            content = 'package spurgear_test\n'
            root = make_root(tmp, draft={'a_test.go': content}, placed={'a_test.go': content})
            dest = root / 'proof' / 'spurgear' / 'a_test.go'
            before_mtime = dest.stat().st_mtime_ns
            code, out, _ = run_stage(root, 'spurgear', 'proof')
            self.assertEqual(code, 0)
            self.assertIn('unchanged', out)
            self.assertEqual(dest.stat().st_mtime_ns, before_mtime)

    def test_proof_overwrites_a_destination_file_that_differs(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = make_root(tmp, draft={'a_test.go': 'new\n'}, placed={'a_test.go': 'old\n'})
            code, _, _ = run_stage(root, 'spurgear', 'proof')
            self.assertEqual(code, 0)
            self.assertEqual((root / 'proof' / 'spurgear' / 'a_test.go').read_text(), 'new\n')

    def test_steps_places_draft_at_spec_steps_md(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = make_root(tmp, steps='# steps\n')
            code, out, _ = run_stage(root, 'spurgear', 'steps')
            self.assertEqual(code, 0)
            self.assertEqual((root / 'spec' / 'spurgear' / 'steps.md').read_text(), '# steps\n')
            self.assertIn('wrote', out)

    def test_module_places_draft_at_lib_geargen(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = make_root(tmp, module='# generated\n')
            code, _, _ = run_stage(root, 'spurgear', 'module')
            self.assertEqual(code, 0)
            self.assertEqual((root / 'lib' / 'geargen' / 'spurgear.py').read_text(),
                             '# generated\n')

    def test_source_survives_and_rerun_is_idempotent(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = make_root(tmp, draft={'a_test.go': 'x\n'})
            code1, _, _ = run_stage(root, 'spurgear', 'proof')
            self.assertEqual(code1, 0)
            self.assertTrue((root / '.tmp' / 'spurgear-proof' / 'a_test.go').exists())
            code2, out2, _ = run_stage(root, 'spurgear', 'proof')
            self.assertEqual(code2, 0)
            self.assertNotIn('wrote', out2)
            self.assertIn('unchanged', out2)


class StaleFileTests(unittest.TestCase):
    def test_prunes_a_go_file_the_draft_no_longer_produces(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = make_root(tmp, draft={'a_test.go': 'x\n'},
                              placed={'helpers_test.go': 'stale\n'})
            code, out, _ = run_stage(root, 'spurgear', 'proof')
            self.assertEqual(code, 0)
            self.assertIn('pruned', out)
            self.assertIn('helpers_test.go', out)
            self.assertFalse((root / 'proof' / 'spurgear' / 'helpers_test.go').exists())

    def test_non_go_file_in_destination_survives_and_is_mentioned(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = make_root(tmp, draft={'a_test.go': 'x\n'},
                              placed={'testdata.json': '{}'})
            code, out, _ = run_stage(root, 'spurgear', 'proof')
            self.assertEqual(code, 0)
            self.assertTrue((root / 'proof' / 'spurgear' / 'testdata.json').exists())
            self.assertEqual(out.count('testdata.json'), 1)

    def test_go_file_in_subdirectory_of_destination_is_not_pruned(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = make_root(tmp, draft={'a_test.go': 'x\n'})
            sub = root / 'proof' / 'spurgear' / 'sub'
            sub.mkdir()
            (sub / 'nested_test.go').write_text('nested\n')
            code, _, _ = run_stage(root, 'spurgear', 'proof')
            self.assertEqual(code, 0)
            self.assertTrue((sub / 'nested_test.go').exists())


class RefusalTests(unittest.TestCase):
    def _assert_refused_untouched(self, root, gear, *argv, before=None):
        code, _, err = run_stage(root, gear, *argv)
        self.assertEqual(code, 2)
        self.assertTrue(err.strip())
        if before is not None:
            for path, content in before.items():
                self.assertEqual(Path(path).read_bytes(), content)

    def test_missing_source_directory(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = make_root(tmp)
            self._assert_refused_untouched(root, 'spurgear', 'proof')

    def test_draft_directory_with_no_go_file(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = make_root(tmp, draft={'notes.txt': 'hi\n'})
            (root / '.tmp' / 'spurgear-proof' / 'notes.txt').unlink()
            self._assert_refused_untouched(root, 'spurgear', 'proof')

    def test_draft_directory_with_zero_byte_go_file(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = make_root(tmp, draft={'a_test.go': ''})
            self._assert_refused_untouched(root, 'spurgear', 'proof')

    def test_draft_directory_with_non_go_file(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = make_root(tmp, draft={'a_test.go': 'x\n', 'notes.md': 'hi\n'})
            self._assert_refused_untouched(root, 'spurgear', 'proof')

    def test_draft_directory_with_subdirectory(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = make_root(tmp, draft={'a_test.go': 'x\n'})
            (root / '.tmp' / 'spurgear-proof' / 'sub').mkdir()
            self._assert_refused_untouched(root, 'spurgear', 'proof')

    def test_missing_steps_source_and_zero_byte_steps_source(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = make_root(tmp)
            self._assert_refused_untouched(root, 'spurgear', 'steps')
        with tempfile.TemporaryDirectory() as tmp:
            root = make_root(tmp, steps='')
            self._assert_refused_untouched(root, 'spurgear', 'steps')

    def test_missing_module_source_and_zero_byte_module_source(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = make_root(tmp)
            self._assert_refused_untouched(root, 'spurgear', 'module')
        with tempfile.TemporaryDirectory() as tmp:
            root = make_root(tmp, module='')
            self._assert_refused_untouched(root, 'spurgear', 'module')

    def test_bad_gear_names(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = make_root(tmp)
            for bad in ('../evil', 'Spur', ''):
                self._assert_refused_untouched(root, bad, 'steps')

    def test_steps_and_module_refuse_when_parent_directory_missing(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = make_root(tmp, steps='# steps\n')
            import shutil
            shutil.rmtree(root / 'spec' / 'spurgear')
            self._assert_refused_untouched(root, 'spurgear', 'steps')
        with tempfile.TemporaryDirectory() as tmp:
            root = make_root(tmp, module='# gen\n')
            import shutil
            shutil.rmtree(root / 'lib' / 'geargen')
            self._assert_refused_untouched(root, 'spurgear', 'module')

    def test_proof_creates_destination_directory_when_absent(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = make_root(tmp, draft={'a_test.go': 'x\n'})
            import shutil
            shutil.rmtree(root / 'proof' / 'spurgear')
            code, _, _ = run_stage(root, 'spurgear', 'proof')
            self.assertEqual(code, 0)
            self.assertTrue((root / 'proof' / 'spurgear' / 'a_test.go').exists())


class DirtyDestinationTests(unittest.TestCase):
    def test_hand_edited_destination_refuses_without_force(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = make_root(tmp, draft={'a_test.go': 'draft-2\n'},
                              placed={'a_test.go': 'draft-1\n'})
            git_init(root)
            git_commit_all(root)
            (root / 'proof' / 'spurgear' / 'a_test.go').write_text('hand-edited\n')

            code, _, err = run_stage(root, 'spurgear', 'proof')
            self.assertEqual(code, 2)
            self.assertIn('a_test.go', err)
            self.assertEqual((root / 'proof' / 'spurgear' / 'a_test.go').read_text(),
                             'hand-edited\n')

    def test_force_overwrites_hand_edited_destination(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = make_root(tmp, draft={'a_test.go': 'draft-2\n'},
                              placed={'a_test.go': 'draft-1\n'})
            git_init(root)
            git_commit_all(root)
            (root / 'proof' / 'spurgear' / 'a_test.go').write_text('hand-edited\n')

            code, _, _ = run_stage(root, 'spurgear', 'proof', '--force')
            self.assertEqual(code, 0)
            self.assertEqual((root / 'proof' / 'spurgear' / 'a_test.go').read_text(),
                             'draft-2\n')

    def test_receipt_matching_hand_edited_content_succeeds_without_force(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = make_root(tmp, draft={'a_test.go': 'draft-1\n'})
            code0, _, _ = run_stage(root, 'spurgear', 'proof')
            self.assertEqual(code0, 0)
            git_init(root)
            git_commit_all(root)

            (root / '.tmp' / 'spurgear-proof' / 'a_test.go').write_text('draft-2\n')
            code, _, _ = run_stage(root, 'spurgear', 'proof')
            self.assertEqual(code, 0)
            self.assertEqual((root / 'proof' / 'spurgear' / 'a_test.go').read_text(),
                             'draft-2\n')

    def test_clean_committed_destination_with_no_receipt_is_overwritten(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = make_root(tmp, draft={'a_test.go': 'draft-2\n'},
                              placed={'a_test.go': 'draft-1\n'})
            git_init(root)
            git_commit_all(root)

            code, _, _ = run_stage(root, 'spurgear', 'proof')
            self.assertEqual(code, 0)
            self.assertEqual((root / 'proof' / 'spurgear' / 'a_test.go').read_text(),
                             'draft-2\n')

    def test_non_git_root_stages_successfully_and_notes_skipped_check(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = make_root(tmp, draft={'a_test.go': 'draft-2\n'},
                              placed={'a_test.go': 'draft-1\n'})
            code, _, err = run_stage(root, 'spurgear', 'proof')
            self.assertEqual(code, 0)
            self.assertIn('not a git work tree', err)
            self.assertEqual((root / 'proof' / 'spurgear' / 'a_test.go').read_text(),
                             'draft-2\n')


class AtomicityTests(unittest.TestCase):
    def test_replace_failure_rolls_back_every_touched_file(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = make_root(tmp,
                              draft={'a_test.go': 'new-a\n', 'b_test.go': 'new-b\n'},
                              placed={'a_test.go': 'old-a\n', 'b_test.go': 'old-b\n'})
            real_replace = os.replace
            calls = {'n': 0}

            def flaky_replace(src, dst):
                calls['n'] += 1
                if calls['n'] == 2:
                    raise OSError('synthetic replace failure')
                return real_replace(src, dst)

            with mock.patch.object(MODULE.os, 'replace', side_effect=flaky_replace):
                code, _, err = run_stage(root, 'spurgear', 'proof')
            self.assertEqual(code, 2)
            self.assertTrue(err.strip())
            self.assertEqual((root / 'proof' / 'spurgear' / 'a_test.go').read_text(), 'old-a\n')
            self.assertEqual((root / 'proof' / 'spurgear' / 'b_test.go').read_text(), 'old-b\n')

    def test_unlink_failure_during_prune_rolls_back_writes_too(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = make_root(tmp,
                              draft={'a_test.go': 'new-a\n'},
                              placed={'a_test.go': 'old-a\n', 'stale_test.go': 'stale\n'})

            def flaky_unlink(path):
                raise OSError('synthetic unlink failure')

            with mock.patch.object(MODULE.os, 'unlink', side_effect=flaky_unlink):
                code, _, err = run_stage(root, 'spurgear', 'proof')
            self.assertEqual(code, 2)
            self.assertTrue(err.strip())
            self.assertEqual((root / 'proof' / 'spurgear' / 'a_test.go').read_text(), 'old-a\n')
            self.assertTrue((root / 'proof' / 'spurgear' / 'stale_test.go').exists())


class RunIndexDryRunTests(unittest.TestCase):
    def test_run_with_passing_stub_reports_ok_and_passes_output(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = make_root(tmp, draft={'a_test.go': 'x\n'})
            run_sh = root / 'proof' / 'run.sh'
            run_sh.write_text('#!/usr/bin/env bash\necho hello-from-stub\nexit 0\n')
            run_sh.chmod(run_sh.stat().st_mode | stat.S_IEXEC)

            code, out, _ = run_stage(root, 'spurgear', 'proof', '--run')
            self.assertEqual(code, 0)
            self.assertIn('proof run OK', out)
            self.assertIn('hello-from-stub', out)

    def test_run_with_failing_stub_returns_1_and_keeps_placement(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = make_root(tmp, draft={'a_test.go': 'x\n'})
            run_sh = root / 'proof' / 'run.sh'
            run_sh.write_text('#!/usr/bin/env bash\necho boom-from-stub\nexit 1\n')
            run_sh.chmod(run_sh.stat().st_mode | stat.S_IEXEC)

            code, out, err = run_stage(root, 'spurgear', 'proof', '--run')
            self.assertEqual(code, 1)
            self.assertIn('FAILED', err)
            self.assertIn('boom-from-stub', out)
            self.assertTrue((root / 'proof' / 'spurgear' / 'a_test.go').exists())

    def test_run_flag_rejected_for_steps_and_module(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = make_root(tmp, steps='# steps\n', module='# gen\n')
            with self.assertRaises(SystemExit) as cm:
                MODULE.main(['stage.py', '--root', str(root), 'spurgear', 'steps', '--run'])
            self.assertEqual(cm.exception.code, 2)
            with self.assertRaises(SystemExit) as cm:
                MODULE.main(['stage.py', '--root', str(root), 'spurgear', 'module', '--run'])
            self.assertEqual(cm.exception.code, 2)

    def test_proof_indexes_writes_and_prunes_unless_no_index(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = make_root(tmp, draft={'a_test.go': 'new\n'},
                              placed={'stale_test.go': 'stale\n'})
            git_init(root)
            git_commit_all(root)

            code, _, _ = run_stage(root, 'spurgear', 'proof')
            self.assertEqual(code, 0)
            staged = subprocess.run(
                ['git', 'diff', '--cached', '--name-only'], cwd=str(root),
                capture_output=True, text=True, check=True).stdout
            self.assertIn('proof/spurgear/a_test.go', staged)
            self.assertIn('proof/spurgear/stale_test.go', staged)

    def test_no_index_leaves_git_index_untouched(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = make_root(tmp, draft={'a_test.go': 'new\n'})
            git_init(root)
            git_commit_all(root)

            code, _, _ = run_stage(root, 'spurgear', 'proof', '--no-index')
            self.assertEqual(code, 0)
            staged = subprocess.run(
                ['git', 'diff', '--cached', '--name-only'], cwd=str(root),
                capture_output=True, text=True, check=True).stdout
            self.assertEqual(staged.strip(), '')

    def test_dry_run_prints_would_lines_and_touches_nothing(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = make_root(tmp, draft={'a_test.go': 'new\n'},
                              placed={'stale_test.go': 'stale\n'})
            code, out, _ = run_stage(root, 'spurgear', 'proof', '--dry-run')
            self.assertEqual(code, 0)
            self.assertIn('would write', out)
            self.assertIn('would prune', out)
            self.assertFalse((root / 'proof' / 'spurgear' / 'a_test.go').exists())
            self.assertTrue((root / 'proof' / 'spurgear' / 'stale_test.go').exists())
            self.assertFalse((root / '.tmp' / 'stage').exists())

    def test_dry_run_against_missing_source_still_exits_2(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = make_root(tmp)
            code_dry, _, err_dry = run_stage(root, 'spurgear', 'proof', '--dry-run')
            code_real, _, err_real = run_stage(root, 'spurgear', 'proof')
            self.assertEqual(code_dry, 2)
            self.assertEqual(code_real, 2)
            self.assertEqual(err_dry.strip(), err_real.strip())


if __name__ == '__main__':
    unittest.main()
