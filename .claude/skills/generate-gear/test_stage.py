#!/usr/bin/env python3
"""Regression tests for stage.py, the drafted-artifact placement script."""
import contextlib
import importlib.util
import io
import os
import re
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


def make_root(tmp, gear='spurgear', draft=None, placed=None, steps=None, module=None,
              owned=None):
    """Build a fake repo root: .tmp/, proof/<gear>/, spec/<gear>/, lib/geargen/, proof/run.sh.

    `owned` writes `proof/<gear>/stage-manifest.json` claiming those names, which
    is what a directory stage.py has placed into before looks like. Leaving it
    `None` writes no manifest, which is what a directory staged before stage.py
    tracked ownership looks like, and there nothing may be pruned.
    """
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

    if owned is not None:
        plan = MODULE.gear_paths(str(root), gear, 'proof')
        (root / 'proof' / gear / MODULE.MANIFEST_NAME).write_bytes(
            MODULE.manifest_bytes(plan, owned))

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
                              placed={'helpers_test.go': 'stale\n'},
                              owned=['a_test.go', 'helpers_test.go'])
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
                              placed={'a_test.go': 'old-a\n', 'stale_test.go': 'stale\n'},
                              owned=['a_test.go', 'stale_test.go'])

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
            for target in ('steps', 'module'):
                err = io.StringIO()
                with self.assertRaises(SystemExit) as cm, contextlib.redirect_stderr(err):
                    MODULE.main(['stage.py', '--root', str(root), 'spurgear', target, '--run'])
                self.assertEqual(cm.exception.code, 2)
                self.assertIn('unrecognized arguments: --run', err.getvalue())

    def test_proof_indexes_writes_and_prunes_unless_no_index(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = make_root(tmp, draft={'a_test.go': 'new\n'},
                              placed={'stale_test.go': 'stale\n'},
                              owned=['stale_test.go'])
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
                              placed={'stale_test.go': 'stale\n'},
                              owned=['stale_test.go'])
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


class CompileTargetTests(unittest.TestCase):
    """The `compile` target places the step list and the proof, or neither."""

    def test_compile_places_steps_and_proof_in_one_run(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = make_root(tmp, draft={'a_test.go': 'package spurgear_test\n',
                                          'b_test.go': 'package spurgear_test\n'},
                              steps='# steps\n')
            code, out, _ = run_stage(root, 'spurgear', 'compile')
            self.assertEqual(code, 0)
            self.assertEqual((root / 'spec' / 'spurgear' / 'steps.md').read_text(), '# steps\n')
            dest = root / 'proof' / 'spurgear'
            self.assertEqual((dest / 'a_test.go').read_text(), 'package spurgear_test\n')
            self.assertEqual((dest / 'b_test.go').read_text(), 'package spurgear_test\n')
            self.assertIn('stage: steps OK', out)
            self.assertIn('stage: proof OK', out)
            self.assertIn('stage: compile OK (steps + proof)', out)

    def test_compile_rerun_is_idempotent(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = make_root(tmp, draft={'a_test.go': 'x\n'}, steps='# steps\n')
            code1, _, _ = run_stage(root, 'spurgear', 'compile')
            self.assertEqual(code1, 0)
            code2, out2, _ = run_stage(root, 'spurgear', 'compile')
            self.assertEqual(code2, 0)
            self.assertNotIn('wrote', out2)
            self.assertIn('stage: unchanged %s' % (root / 'spec' / 'spurgear' / 'steps.md'),
                          out2)
            self.assertIn('stage: unchanged %s'
                          % (root / 'proof' / 'spurgear' / 'a_test.go'), out2)
            self.assertIn('stage: compile OK (steps + proof)', out2)

    def test_compile_refuses_when_steps_draft_missing_and_writes_no_proof_file(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = make_root(tmp, draft={'a_test.go': 'x\n'})
            code, _, err = run_stage(root, 'spurgear', 'compile')
            self.assertEqual(code, 2)
            self.assertIn('spurgear.steps.md', err)
            self.assertFalse((root / 'proof' / 'spurgear' / 'a_test.go').exists())

    def test_compile_refuses_when_proof_draft_missing_and_leaves_steps_untouched(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = make_root(tmp, steps='# steps\n')
            code, _, err = run_stage(root, 'spurgear', 'compile')
            self.assertEqual(code, 2)
            self.assertIn('spurgear-proof', err)
            self.assertFalse((root / 'spec' / 'spurgear' / 'steps.md').exists())

    def test_compile_dirty_steps_destination_refuses_before_any_write(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = make_root(tmp, draft={'a_test.go': 'draft-2\n'}, steps='# steps-2\n')
            (root / 'spec' / 'spurgear' / 'steps.md').write_text('# steps-1\n')
            git_init(root)
            git_commit_all(root)
            (root / 'spec' / 'spurgear' / 'steps.md').write_text('hand-edited\n')

            code, _, err = run_stage(root, 'spurgear', 'compile')
            self.assertEqual(code, 2)
            self.assertIn('steps.md', err)
            self.assertEqual((root / 'spec' / 'spurgear' / 'steps.md').read_text(),
                             'hand-edited\n')
            self.assertFalse((root / 'proof' / 'spurgear' / 'a_test.go').exists())

    def test_compile_force_overwrites_both(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = make_root(tmp, draft={'a_test.go': 'draft-2\n'},
                              placed={'a_test.go': 'draft-1\n'}, steps='# steps-2\n')
            (root / 'spec' / 'spurgear' / 'steps.md').write_text('# steps-1\n')
            git_init(root)
            git_commit_all(root)
            (root / 'spec' / 'spurgear' / 'steps.md').write_text('hand-edited\n')
            (root / 'proof' / 'spurgear' / 'a_test.go').write_text('hand-edited\n')

            code, _, _ = run_stage(root, 'spurgear', 'compile', '--force')
            self.assertEqual(code, 0)
            self.assertEqual((root / 'spec' / 'spurgear' / 'steps.md').read_text(),
                             '# steps-2\n')
            self.assertEqual((root / 'proof' / 'spurgear' / 'a_test.go').read_text(),
                             'draft-2\n')

    def test_compile_dry_run_reports_both_and_touches_nothing(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = make_root(tmp, draft={'a_test.go': 'x\n'}, steps='# steps\n')
            code, out, _ = run_stage(root, 'spurgear', 'compile', '--dry-run')
            self.assertEqual(code, 0)
            self.assertEqual(out.count('would write'), 2)
            self.assertIn('steps.md', out)
            self.assertIn('a_test.go', out)
            self.assertFalse((root / 'spec' / 'spurgear' / 'steps.md').exists())
            self.assertFalse((root / 'proof' / 'spurgear' / 'a_test.go').exists())
            self.assertFalse((root / '.tmp' / 'stage').exists())

    def test_compile_writes_both_receipts(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = make_root(tmp, draft={'a_test.go': 'x\n'}, steps='# steps\n')
            code, _, _ = run_stage(root, 'spurgear', 'compile')
            self.assertEqual(code, 0)
            self.assertTrue((root / '.tmp' / 'stage' / 'spurgear.steps.json').exists())
            self.assertTrue((root / '.tmp' / 'stage' / 'spurgear.proof.json').exists())

            code_steps, out_steps, _ = run_stage(root, 'spurgear', 'steps')
            self.assertEqual(code_steps, 0)
            self.assertIn('unchanged', out_steps)
            self.assertNotIn('wrote', out_steps)

    def test_compile_indexes_proof_unless_no_index(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = make_root(tmp, draft={'a_test.go': 'new\n'},
                              placed={'stale_test.go': 'stale\n'}, steps='# steps\n',
                              owned=['stale_test.go'])
            git_init(root)
            git_commit_all(root)

            code, _, _ = run_stage(root, 'spurgear', 'compile')
            self.assertEqual(code, 0)
            staged = subprocess.run(
                ['git', 'diff', '--cached', '--name-only'], cwd=str(root),
                capture_output=True, text=True, check=True).stdout
            self.assertIn('proof/spurgear/a_test.go', staged)
            self.assertIn('proof/spurgear/stale_test.go', staged)

        with tempfile.TemporaryDirectory() as tmp:
            root = make_root(tmp, draft={'a_test.go': 'new\n'}, steps='# steps\n')
            git_init(root)
            git_commit_all(root)

            code, _, _ = run_stage(root, 'spurgear', 'compile', '--no-index')
            self.assertEqual(code, 0)
            staged = subprocess.run(
                ['git', 'diff', '--cached', '--name-only'], cwd=str(root),
                capture_output=True, text=True, check=True).stdout
            self.assertEqual(staged.strip(), '')

    def test_compile_proof_failure_rolls_back_placed_steps(self):
        real_replace = os.replace

        def fail_on_proof(src, dst):
            if os.sep + 'proof' + os.sep in str(dst):
                raise OSError('synthetic proof failure')
            return real_replace(src, dst)

        with tempfile.TemporaryDirectory() as tmp:
            root = make_root(tmp, draft={'a_test.go': 'new\n'}, steps='# steps-2\n')
            (root / 'spec' / 'spurgear' / 'steps.md').write_text('# steps-1\n')
            with mock.patch.object(MODULE.os, 'replace', side_effect=fail_on_proof):
                code, _, err = run_stage(root, 'spurgear', 'compile')
            self.assertEqual(code, 2)
            self.assertTrue(err.strip())
            self.assertEqual((root / 'spec' / 'spurgear' / 'steps.md').read_text(),
                             '# steps-1\n')
            self.assertFalse((root / 'proof' / 'spurgear' / 'a_test.go').exists())

        with tempfile.TemporaryDirectory() as tmp:
            root = make_root(tmp, draft={'a_test.go': 'new\n'}, steps='# steps-2\n')
            with mock.patch.object(MODULE.os, 'replace', side_effect=fail_on_proof):
                code, _, err = run_stage(root, 'spurgear', 'compile')
            self.assertEqual(code, 2)
            self.assertFalse((root / 'spec' / 'spurgear' / 'steps.md').exists())

    def test_run_flag_rejected_for_compile(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = make_root(tmp, draft={'a_test.go': 'x\n'}, steps='# steps\n')
            err = io.StringIO()
            with self.assertRaises(SystemExit) as cm, contextlib.redirect_stderr(err):
                MODULE.main(['stage.py', '--root', str(root), 'spurgear', 'compile', '--run'])
            self.assertEqual(cm.exception.code, 2)
            self.assertIn('unrecognized arguments: --run', err.getvalue())


class OwnershipTests(unittest.TestCase):
    """Pruning is restricted to files the manifest says this script generated.

    `proof/<gear>/` may hold `.go` files the pipeline does not generate, and
    before the manifest existed every one of them was deleted on the next
    placement of that gear. `proof/bevelgear/render_test.go` and
    `proof/cycloidal/render_test.go` are the two real cases; nothing here
    depends on either name.
    """

    def _read_manifest(self, root, gear='spurgear'):
        path = root / 'proof' / gear / MODULE.MANIFEST_NAME
        return MODULE.parse_manifest(path.read_bytes())

    def test_auxiliary_file_survives_when_no_manifest_claims_it(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = make_root(tmp, draft={'a_test.go': 'x\n'},
                              placed={'a_test.go': 'x\n', 'render_test.go': 'hand\n'},
                              owned=['a_test.go'])
            code, out, _ = run_stage(root, 'spurgear', 'proof')
            self.assertEqual(code, 0)
            self.assertTrue((root / 'proof' / 'spurgear' / 'render_test.go').exists())
            self.assertIn('render_test.go left alone', out)
            self.assertNotIn('stage: pruned', out)
            self.assertIn('0 pruned', out)

    def test_auxiliary_file_survives_a_directory_with_no_manifest(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = make_root(tmp, draft={'a_test.go': 'x\n'},
                              placed={'a_test.go': 'x\n', 'render_test.go': 'hand\n'})
            code, out, _ = run_stage(root, 'spurgear', 'proof')
            self.assertEqual(code, 0)
            self.assertTrue((root / 'proof' / 'spurgear' / 'render_test.go').exists())
            self.assertIn('does not claim it', out)

    def test_obsolete_generated_file_is_removed_and_auxiliary_kept_together(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = make_root(tmp, draft={'a_test.go': 'x\n'},
                              placed={'a_test.go': 'x\n', 'old_test.go': 'stale\n',
                                      'render_test.go': 'hand\n'},
                              owned=['a_test.go', 'old_test.go'])
            code, out, _ = run_stage(root, 'spurgear', 'proof')
            self.assertEqual(code, 0)
            self.assertFalse((root / 'proof' / 'spurgear' / 'old_test.go').exists())
            self.assertTrue((root / 'proof' / 'spurgear' / 'render_test.go').exists())
            self.assertIn('old_test.go', out)
            self.assertIn('render_test.go left alone', out)

    def test_manifest_records_exactly_the_draft_and_survives_a_second_run(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = make_root(tmp, draft={'a_test.go': 'x\n', 'b_test.go': 'y\n'},
                              placed={'render_test.go': 'hand\n'})
            code, _, _ = run_stage(root, 'spurgear', 'proof')
            self.assertEqual(code, 0)
            self.assertEqual(self._read_manifest(root), {'a_test.go', 'b_test.go'})

            # The manifest written by run 1 is what lets run 2 prune b_test.go,
            # which is the whole point of recording it.
            (root / '.tmp' / 'spurgear-proof' / 'b_test.go').unlink()
            code, _, _ = run_stage(root, 'spurgear', 'proof')
            self.assertEqual(code, 0)
            self.assertFalse((root / 'proof' / 'spurgear' / 'b_test.go').exists())
            self.assertTrue((root / 'proof' / 'spurgear' / 'render_test.go').exists())
            self.assertEqual(self._read_manifest(root), {'a_test.go'})

    def test_damaged_manifest_reads_as_absent_and_prunes_nothing(self):
        for damaged in (b'not json at all', b'[]', b'{"files": "a_test.go"}',
                         b'{"files": [1, 2]}', b'\xff\xfe\x00'):
            with self.subTest(damaged=damaged):
                with tempfile.TemporaryDirectory() as tmp:
                    root = make_root(tmp, draft={'a_test.go': 'x\n'},
                                      placed={'old_test.go': 'stale\n'})
                    (root / 'proof' / 'spurgear' / MODULE.MANIFEST_NAME).write_bytes(damaged)
                    code, out, _ = run_stage(root, 'spurgear', 'proof')
                    self.assertEqual(code, 0)
                    self.assertTrue((root / 'proof' / 'spurgear' / 'old_test.go').exists())
                    self.assertIn('does not claim it', out)

    def test_manifest_is_not_reported_as_a_human_added_extra(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = make_root(tmp, draft={'a_test.go': 'x\n'}, owned=['a_test.go'])
            code, out, _ = run_stage(root, 'spurgear', 'proof')
            self.assertEqual(code, 0)
            self.assertNotIn('%s left alone' % MODULE.MANIFEST_NAME, out)

    def test_modified_destination_still_refuses_without_force(self):
        """The manifest is not the overwrite guard, and does not become one."""
        with tempfile.TemporaryDirectory() as tmp:
            root = make_root(tmp, draft={'a_test.go': 'draft-2\n'},
                              placed={'a_test.go': 'draft-1\n'}, owned=['a_test.go'])
            git_init(root)
            git_commit_all(root)
            (root / 'proof' / 'spurgear' / 'a_test.go').write_text('hand-edited\n')

            code, _, err = run_stage(root, 'spurgear', 'proof')
            self.assertEqual(code, 2)
            self.assertIn('neither this script', err)
            self.assertEqual((root / 'proof' / 'spurgear' / 'a_test.go').read_text(),
                             'hand-edited\n')

            code, _, _ = run_stage(root, 'spurgear', 'proof', '--force')
            self.assertEqual(code, 0)
            self.assertEqual((root / 'proof' / 'spurgear' / 'a_test.go').read_text(),
                             'draft-2\n')

    def test_uncommitted_auxiliary_file_still_refuses_the_whole_placement(self):
        """The clean-tree guard still reads every `.go` file, owned or not.

        Ownership decides what may be deleted; it does not narrow this guard.
        An auxiliary file with uncommitted edits therefore still refuses the
        placement, which is the pre-existing behaviour and is left as it was:
        the guard's job is to make a human look before the script writes into
        a directory holding unsaved work, and `--force` is the way past it.
        Narrowing it to owned files alone would be a separate change with its
        own argument.
        """
        with tempfile.TemporaryDirectory() as tmp:
            root = make_root(tmp, draft={'a_test.go': 'x\n'},
                              placed={'a_test.go': 'x\n', 'render_test.go': 'committed\n'},
                              owned=['a_test.go'])
            git_init(root)
            git_commit_all(root)
            (root / 'proof' / 'spurgear' / 'render_test.go').write_text('edited\n')

            code, _, err = run_stage(root, 'spurgear', 'proof')
            self.assertEqual(code, 2)
            self.assertIn('render_test.go', err)
            self.assertEqual((root / 'proof' / 'spurgear' / 'render_test.go').read_text(),
                             'edited\n')

            # Forcing past it writes the draft and still does not touch the
            # auxiliary file, because ownership decides deletion on its own.
            code, _, _ = run_stage(root, 'spurgear', 'proof', '--force')
            self.assertEqual(code, 0)
            self.assertEqual((root / 'proof' / 'spurgear' / 'render_test.go').read_text(),
                             'edited\n')

    def test_dry_run_preview_matches_what_a_real_run_changes(self):
        """Every `would` line, and no other file, is what the real run touches."""
        def snapshot(directory):
            return {path.name: path.read_bytes()
                    for path in sorted(directory.iterdir()) if path.is_file()}

        draft = {'a_test.go': 'new-a\n', 'c_test.go': 'new-c\n'}
        placed = {'a_test.go': 'old-a\n', 'b_test.go': 'stale-b\n',
                  'render_test.go': 'hand\n', 'testdata.json': '{}'}
        owned = ['a_test.go', 'b_test.go']

        with tempfile.TemporaryDirectory() as tmp:
            root = make_root(tmp, draft=dict(draft), placed=dict(placed), owned=owned)
            directory = root / 'proof' / 'spurgear'
            before = snapshot(directory)
            code, out, _ = run_stage(root, 'spurgear', 'proof', '--dry-run')
            self.assertEqual(code, 0)
            self.assertEqual(snapshot(directory), before)
            # Read the preview back out of the report rather than out of the
            # plan: the report is what a reader acts on.
            would_write = {line.rsplit('/', 1)[1].split(' ')[0]
                            for line in out.splitlines() if 'would write' in line}
            would_prune = {line.rsplit('/', 1)[1].split(' ')[0]
                            for line in out.splitlines() if 'would prune' in line}

        with tempfile.TemporaryDirectory() as tmp:
            root = make_root(tmp, draft=dict(draft), placed=dict(placed), owned=owned)
            directory = root / 'proof' / 'spurgear'
            before = snapshot(directory)
            code, _, _ = run_stage(root, 'spurgear', 'proof')
            self.assertEqual(code, 0)
            after = snapshot(directory)

        changed = {name for name in set(before) | set(after)
                   if before.get(name) != after.get(name)}
        removed = set(before) - set(after)
        written = {name for name in changed if name in after}

        self.assertEqual(would_write, {'a_test.go', 'c_test.go'})
        self.assertEqual(would_prune, {'b_test.go'})
        self.assertEqual(removed, would_prune)
        # The manifest is this script's own bookkeeping, not a previewed action.
        self.assertEqual(written - {MODULE.MANIFEST_NAME}, would_write)
        self.assertIn(MODULE.MANIFEST_NAME, after)


class SeededManifestTests(unittest.TestCase):
    """The manifests committed for the existing gears must not over-claim.

    A seed is a claim about which files the pipeline generates. A seed that
    wrongly named an auxiliary file would hand the deletion back its blessing,
    with the mechanism working exactly as designed, so the seeds are checked
    against the step lists rather than against the directory listings.
    """

    # stage.py sits at .claude/skills/generate-gear/, so the repo root is four
    # levels up from the file rather than three.
    REPO = MODULE_PATH.parent.parent.parent.parent
    PROOF_PATH = re.compile(
        r'`?proof/(?P<gear>[a-z][a-z0-9_]*)/(?P<name>[A-Za-z0-9_]+\.go)`?')

    def _gears(self):
        spec = self.REPO / 'spec'
        if not spec.is_dir():
            self.skipTest('not running inside the gear repository')
        for entry in sorted(spec.iterdir()):
            if (entry / 'steps.md').is_file() and (self.REPO / 'proof' / entry.name).is_dir():
                yield entry.name

    def _declared(self, gear):
        text = (self.REPO / 'spec' / gear / 'steps.md').read_text(encoding='utf-8')
        head = text.split('## Provenance', 1)[0]
        return {m.group('name') for m in self.PROOF_PATH.finditer(head)
                if m.group('gear') == gear}

    def test_every_compiled_gear_has_a_manifest(self):
        for gear in self._gears():
            with self.subTest(gear=gear):
                self.assertTrue((self.REPO / 'proof' / gear / MODULE.MANIFEST_NAME).is_file())

    def test_manifest_names_exactly_the_step_list_proof_files(self):
        for gear in self._gears():
            with self.subTest(gear=gear):
                path = self.REPO / 'proof' / gear / MODULE.MANIFEST_NAME
                claimed = MODULE.parse_manifest(path.read_bytes())
                self.assertIsNotNone(claimed, '%s is unreadable' % path)
                self.assertEqual(claimed, self._declared(gear))

    def test_no_manifest_claims_a_file_absent_from_its_directory(self):
        for gear in self._gears():
            with self.subTest(gear=gear):
                path = self.REPO / 'proof' / gear / MODULE.MANIFEST_NAME
                claimed = MODULE.parse_manifest(path.read_bytes()) or set()
                present = {entry.name for entry in (self.REPO / 'proof' / gear).iterdir()
                           if entry.name.endswith('.go')}
                self.assertEqual(claimed - present, set())


if __name__ == '__main__':
    unittest.main()
