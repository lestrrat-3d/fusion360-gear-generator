#!/usr/bin/env python3
"""Regression tests for the per-stage environment check.

Two properties matter more than the individual verdicts. The first is that a warning never
costs a run: pyright, the stub cache and the root checkout are all things a stage can proceed
without, and grading any of them a failure would stop a round that would have succeeded. The
second is that the check writes nothing but `.tmp/` — it stands in front of tools that clone
and install, and a preflight that quietly did their work would hide the cost it exists to
report.

Everything runs against a fixture repository in a tempdir, so no test depends on the machine
having the engines, the stubs or the plugin database.
"""
import importlib.util
import io
import json
import os
import subprocess
import tempfile
import unittest
from contextlib import redirect_stderr, redirect_stdout
from pathlib import Path
from unittest import mock


CHECKER = Path(__file__).with_name('preflight.py')
SPEC = importlib.util.spec_from_file_location('preflight', CHECKER)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)

GEAR = 'testgear'


def git(root, *args):
    subprocess.run(['git', '-C', str(root)] + list(args),
                   check=True, capture_output=True, text=True)


def make_repo(root, gear=GEAR):
    """A repository shaped enough for every check to reach a verdict on it."""
    for rel in ('spec/%s' % gear, 'lib/geargen', 'lib/fusion360utils',
                '.claude/skills/generate-gear', 'proof/%s' % gear, 'proof/proofkit',
                'proof/proofkit3d', 'proof/involute'):
        (root / rel).mkdir(parents=True, exist_ok=True)
    (root / 'spec' / gear / 'instructions.md').write_text('# spec\n')
    for name in ('base.py', 'misc.py', 'utilities.py', 'spurproxy.py'):
        (root / 'lib' / 'geargen' / name).write_text('')
    (root / 'lib' / 'fusion360utils' / '__init__.py').write_text('')
    (root / 'proof' / 'run.sh').write_text('#!/usr/bin/env bash\n')
    git(root, 'init', '-q')
    (root / 'README.md').write_text('fixture\n')
    git(root, 'add', '-A')
    git(root, '-c', 'user.name=t', '-c', 'user.email=t@example.com',
        'commit', '-q', '-m', 'fixture')
    return root


def completed(returncode, stdout='', stderr=''):
    """What the check_compile.py seam hands back, without running it."""
    return subprocess.CompletedProcess(args=['check_compile.py'], returncode=returncode,
                                       stdout=stdout, stderr=stderr)


def stub_defs(root):
    """A directory that satisfies fusion_stubs.defs_at()."""
    defs = root / 'defs'
    (defs / 'adsk').mkdir(parents=True)
    (defs / 'adsk' / 'core.py').write_text('')
    return defs


class Fixture(unittest.TestCase):
    """A fixture repo plus a context pointed at it."""

    def setUp(self):
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        # The repo sits one level down so that its parent — where the engine siblings would
        # live — belongs to the test and not to the machine's temp directory.
        self.root = make_repo(Path(directory.name) / 'repo')
        self.ctx = MODULE.Context(str(self.root), GEAR)

    def run_cli(self, *argv):
        """(exit code, stdout) for one invocation, with stderr swallowed."""
        out, err = io.StringIO(), io.StringIO()
        with redirect_stdout(out), redirect_stderr(err):
            code = MODULE.cli(list(argv))
        return code, out.getvalue()

    def resolved(self, gear=GEAR):
        """An environment in which the two machine-wide dependencies are satisfied."""
        query = self.root / 'query_fusion_api.py'
        query.write_text('')
        return {'FUSION_QUERY_API': str(query),
                'FUSION_API_STUBS': str(stub_defs(self.root))}


class UsageTests(Fixture):
    """Exit 2 is for an invocation the script refuses, never for a failed check."""

    def test_bad_gear_name_is_a_usage_error(self):
        code, _ = self.run_cli('Spur Gear', '--root', str(self.root))

        self.assertEqual(code, 2)

    def test_unknown_stage_is_a_usage_error(self):
        with self.assertRaises(SystemExit) as raised:
            with redirect_stderr(io.StringIO()):
                MODULE.cli([GEAR, '--stage', 'polish', '--root', str(self.root)])

        self.assertEqual(raised.exception.code, 2)

    def test_root_without_the_repository_layout_is_a_usage_error(self):
        with tempfile.TemporaryDirectory() as elsewhere:
            code, _ = self.run_cli(GEAR, '--root', elsewhere)

        self.assertEqual(code, 2)

    def test_root_naming_the_repository_is_accepted(self):
        self.assertEqual(MODULE.resolve_root(str(self.root)), str(self.root))


class EmitStageTests(Fixture):
    """The emit stage stands or falls on the compiled artifacts."""

    def test_missing_step_list_fails_the_stage(self):
        with mock.patch.dict(os.environ, self.resolved()):
            code, out = self.run_cli(GEAR, '--stage', 'emit', '--root', str(self.root))

        self.assertEqual(code, 1)
        self.assertIn('[FAIL] steps:', out)
        self.assertIn('NOT READY', out)

    def test_step_list_and_proof_make_the_stage_ready(self):
        (self.root / 'spec' / GEAR / 'steps.md').write_text('# steps\n')
        (self.root / 'proof' / GEAR / 'x.go').write_text('package proof\n')

        with mock.patch.dict(os.environ, self.resolved()), \
                mock.patch.object(MODULE, '_run_check_compile', return_value=completed(0)):
            code, out = self.run_cli(GEAR, '--stage', 'emit', '--root', str(self.root))

        self.assertEqual(code, 0, out)
        self.assertIn('READY', out)

    def test_empty_proof_directory_fails(self):
        (self.root / 'spec' / GEAR / 'steps.md').write_text('# steps\n')

        status, detail = MODULE.check_proof_dir(self.ctx)

        self.assertEqual(status, MODULE.FAIL)
        self.assertIn('no .go file', detail)

    def test_absent_contract_manifest_is_skipped_not_failed(self):
        status, detail = MODULE.check_contract_manifest(self.ctx)

        self.assertEqual(status, MODULE.SKIP)
        self.assertIn('prose-checked', detail)


class StepsCurrentTests(Fixture):
    """The emit stage's freshness row: it stands where `/emit-gear` step 2 stood, so it must
    reach the same verdicts check_compile.py reaches, and it must belong to no other stage."""

    def steps(self):
        (self.root / 'spec' / GEAR / 'steps.md').write_text('# steps\n')

    def test_only_the_emit_stage_runs_it(self):
        self.assertIn('steps-current', [k for k, _, _ in MODULE.plan('emit')])
        self.assertNotIn('steps-current', [k for k, _, _ in MODULE.plan('compile')])
        self.assertNotIn('steps-current', [k for k, _, _ in MODULE.plan('generate')])
        self.assertIn('steps-current', [k for k, _, _ in MODULE.plan('all')])

    def test_a_missing_step_list_is_skipped_here_and_failed_by_the_steps_row(self):
        status, detail = MODULE.check_steps_current(self.ctx)

        self.assertEqual(status, MODULE.SKIP)
        self.assertIn('steps row', detail)

        with mock.patch.dict(os.environ, self.resolved()):
            results = MODULE.run_checks(self.ctx, 'emit')
        by_key = {r['key']: r for r in results}
        self.assertEqual(by_key['steps-current']['status'], MODULE.SKIP)
        self.assertEqual(by_key['steps']['status'], MODULE.FAIL)

    def test_exit_zero_is_ok(self):
        self.steps()

        with mock.patch.object(MODULE, '_run_check_compile',
                               return_value=completed(0, 'compile check: OK (12 steps)\n')):
            status, detail = MODULE.check_steps_current(self.ctx)

        self.assertEqual(status, MODULE.OK)
        self.assertIn('current', detail)

    def test_blocking_findings_fail_and_name_the_drift(self):
        self.steps()
        report = ('coverage: spec/x.md — 3/4 lines claimed by a step\n'
                  'compile check: BLOCKING (1)\n'
                  '  spec/testgear/instructions.md has changed since the step list was compiled\n')

        with mock.patch.object(MODULE, '_run_check_compile', return_value=completed(1, report)):
            status, detail = MODULE.check_steps_current(self.ctx)

        self.assertEqual(status, MODULE.FAIL)
        self.assertIn('compile check: BLOCKING (1)', detail)
        self.assertIn('run /compile-gear %s' % GEAR, detail)
        self.assertNotIn('coverage:', detail)

    def test_exit_two_fails_as_a_setup_error(self):
        self.steps()

        with mock.patch.object(MODULE, '_run_check_compile',
                               return_value=completed(2, '', 'no spec/testgear/steps.md\n')):
            status, detail = MODULE.check_steps_current(self.ctx)

        self.assertEqual(status, MODULE.FAIL)
        self.assertIn('exit 2', detail)
        self.assertIn('no spec/testgear/steps.md', detail)

    def test_a_timeout_fails_the_row_rather_than_raising(self):
        self.steps()
        timed_out = subprocess.TimeoutExpired(cmd='check_compile.py',
                                              timeout=MODULE.CHECK_COMPILE_TIMEOUT)

        with mock.patch.object(MODULE, '_run_check_compile', side_effect=timed_out):
            status, detail = MODULE.check_steps_current(self.ctx)

        self.assertEqual(status, MODULE.FAIL)
        self.assertIn('did not finish', detail)

    def test_the_command_runs_the_skill_s_script_from_the_repo_root(self):
        self.steps()

        with mock.patch.object(MODULE.subprocess, 'run', return_value=completed(0)) as run:
            MODULE.check_steps_current(self.ctx)

        argv = run.call_args[0][0]
        self.assertEqual(argv[0], MODULE.sys.executable)
        self.assertEqual(argv[1], os.path.join(MODULE.HERE, 'check_compile.py'))
        self.assertEqual(argv[2], GEAR)
        self.assertEqual(run.call_args[1]['cwd'], str(self.root))
        self.assertEqual(run.call_args[1]['timeout'], MODULE.CHECK_COMPILE_TIMEOUT)


class TmpDirTests(Fixture):
    """The one thing the check is allowed to write."""

    def test_missing_tmp_dir_is_created_and_reported(self):
        status, detail = MODULE.check_tmp_dir(self.ctx)

        self.assertEqual(status, MODULE.OK)
        self.assertIn('created', detail)
        self.assertTrue((self.root / '.tmp').is_dir())

    def test_existing_tmp_dir_is_not_reported_as_created(self):
        (self.root / '.tmp').mkdir()

        status, detail = MODULE.check_tmp_dir(self.ctx)

        self.assertEqual(status, MODULE.OK)
        self.assertNotIn('created', detail)


class WorktreeTests(Fixture):
    """The root checkout is a warning, because read-only work legitimately runs there."""

    def test_root_checkout_warns(self):
        status, detail = MODULE.check_worktree(self.ctx)

        self.assertEqual(status, MODULE.WARN)
        self.assertIn('worktree', detail)

    def test_linked_worktree_does_not_warn(self):
        linked = self.root / '.worktrees' / 'b'
        git(self.root, 'worktree', 'add', '-q', str(linked), '-b', 'b')

        status, _ = MODULE.check_worktree(MODULE.Context(str(linked), GEAR))

        self.assertEqual(status, MODULE.OK)


class EngineTests(Fixture):
    """Engine resolution copies proof/run.sh: the override wins, and go.mod decides."""

    def test_override_pointing_at_a_module_resolves(self):
        engine = self.root / 'sketch'
        engine.mkdir()
        (engine / 'go.mod').write_text('module x\n')

        with mock.patch.dict(os.environ, {'SKETCH_DIR': str(engine)}):
            status, detail = MODULE.check_sketch_engine(self.ctx)

        self.assertEqual(status, MODULE.OK)
        self.assertIn(str(engine), detail)

    def test_override_without_a_go_mod_fails_and_names_the_path(self):
        empty = self.root / 'not-an-engine'
        empty.mkdir()

        with mock.patch.dict(os.environ, {'DECAD_DIR': str(empty)}):
            status, detail = MODULE.check_decad_engine(self.ctx)

        self.assertEqual(status, MODULE.FAIL)
        self.assertIn(str(empty), detail)
        self.assertIn('DECAD_DIR', detail)

    def test_without_an_override_the_main_checkout_sibling_is_used(self):
        sibling = self.root.parent / 'sketch'
        sibling.mkdir()
        (sibling / 'go.mod').write_text('module x\n')

        with mock.patch.dict(os.environ, {}, clear=False):
            os.environ.pop('SKETCH_DIR', None)
            status, detail = MODULE.check_sketch_engine(self.ctx)

        self.assertEqual(status, MODULE.OK, detail)
        self.assertIn(str(sibling), detail)


class ToolTests(Fixture):
    """A missing toolchain is read off PATH, never installed."""

    def empty_path(self):
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        return {'PATH': directory.name}

    def test_git_missing_fails(self):
        with mock.patch.dict(os.environ, self.empty_path()):
            status, detail = MODULE.check_git(self.ctx)

        self.assertEqual(status, MODULE.FAIL)
        self.assertIn('PATH', detail)

    def test_go_missing_fails_the_compile_stage(self):
        with mock.patch.dict(os.environ, self.empty_path()):
            status, _ = MODULE.check_go(self.ctx)

        self.assertEqual(status, MODULE.FAIL)

    def test_go_missing_only_warns_for_generate_without_a_sketch_bench(self):
        with mock.patch.dict(os.environ, self.empty_path()):
            results = MODULE.run_checks(self.ctx, 'generate')

        by_key = {r['key']: r for r in results}
        self.assertEqual(by_key['go']['status'], MODULE.WARN)
        self.assertEqual(by_key['sketch-bench']['status'], MODULE.WARN)

    def test_go_missing_fails_generate_once_a_sketch_bench_exists(self):
        (self.root / 'spec' / GEAR / 'steps.md').write_text('steps\n')
        (self.root / 'proof' / GEAR).mkdir(parents=True, exist_ok=True)

        with mock.patch.dict(os.environ, self.empty_path()):
            results = MODULE.run_checks(self.ctx, 'generate')

        by_key = {r['key']: r for r in results}
        self.assertEqual(by_key['go']['status'], MODULE.FAIL)
        self.assertEqual(by_key['sketch-bench']['status'], MODULE.OK)

    def test_pyright_absence_is_a_warning(self):
        with mock.patch.object(MODULE.importlib.util, 'find_spec', return_value=None):
            status, detail = MODULE.check_pyright(self.ctx)

        self.assertEqual(status, MODULE.WARN)
        self.assertIn('pip install', detail)


class ApiDatabaseTests(Fixture):
    """The database is resolved by the module that owns the policy, not re-derived here."""

    def test_existing_query_script_resolves(self):
        query = self.root / 'query_fusion_api.py'
        query.write_text('')

        with mock.patch.dict(os.environ, {'FUSION_QUERY_API': str(query)}):
            status, detail = MODULE.check_api_db(self.ctx)

        self.assertEqual(status, MODULE.OK)
        self.assertIn(str(query), detail)

    def test_missing_query_script_fails_with_the_module_s_own_message(self):
        with mock.patch.dict(os.environ, {'FUSION_QUERY_API': str(self.root / 'nowhere.py')}):
            status, detail = MODULE.check_api_db(self.ctx)

        self.assertEqual(status, MODULE.FAIL)
        self.assertIn('does not exist', detail)

    def test_the_generate_stage_downgrades_it_to_a_warning(self):
        with mock.patch.dict(os.environ, {'FUSION_QUERY_API': str(self.root / 'nowhere.py')}):
            results = MODULE.run_checks(self.ctx, 'generate')

        by_key = {r['key']: r for r in results}
        self.assertEqual(by_key['api-db']['status'], MODULE.WARN)

    def test_the_union_keeps_the_strictest_grade(self):
        with mock.patch.dict(os.environ, {'FUSION_QUERY_API': str(self.root / 'nowhere.py')}):
            results = MODULE.run_checks(self.ctx, 'all')

        by_key = {r['key']: r for r in results}
        self.assertEqual(by_key['api-db']['status'], MODULE.FAIL)
        self.assertEqual(len(by_key), len(results))  # each key runs once


class StubTests(Fixture):
    """$FUSION_API_STUBS is authoritative; a cold cache is not a broken environment."""

    def test_set_but_wrong_env_var_fails(self):
        with mock.patch.dict(os.environ, {'FUSION_API_STUBS': str(self.root / 'nowhere')}):
            status, detail = MODULE.check_stubs(self.ctx)

        self.assertEqual(status, MODULE.FAIL)
        self.assertIn('exit 2', detail)

    def test_set_and_correct_env_var_resolves(self):
        with mock.patch.dict(os.environ, {'FUSION_API_STUBS': str(stub_defs(self.root))}):
            status, _ = MODULE.check_stubs(self.ctx)

        self.assertEqual(status, MODULE.OK)

    def test_cold_cache_warns_and_clones_nothing(self):
        cache = self.root / 'cache'
        cache.mkdir()
        env = {'XDG_CACHE_HOME': str(cache)}
        with mock.patch.dict(os.environ, env):
            os.environ.pop('FUSION_API_STUBS', None)
            status, detail = MODULE.check_stubs(self.ctx)

        self.assertEqual(status, MODULE.WARN)
        self.assertIn('auto-clone', detail)
        self.assertEqual(os.listdir(str(cache)), [])


class ModelTierTests(Fixture):
    """The row records the tiers; it never decides whether a stage may run."""

    def test_the_row_skips_when_no_default_model_is_given(self):
        status, detail = MODULE.check_model_tiers(self.ctx)

        self.assertEqual(status, MODULE.SKIP)
        self.assertIn('--default-model', detail)

    def test_the_row_reports_both_roles(self):
        ctx = MODULE.Context(str(self.root), GEAR, 'opus')

        status, detail = MODULE.check_model_tiers(ctx)

        self.assertEqual(status, MODULE.OK)
        self.assertIn('design=opus', detail)
        self.assertIn('mechanical=sonnet', detail)

    def test_an_off_ladder_default_still_passes(self):
        ctx = MODULE.Context(str(self.root), GEAR, 'something-unreleased')

        status, detail = MODULE.check_model_tiers(ctx)

        self.assertEqual(status, MODULE.OK)
        self.assertIn('not on the ladder', detail)

    def test_the_flag_reaches_the_row(self):
        with mock.patch.dict(os.environ, self.resolved()):
            _, out = self.run_cli(GEAR, '--stage', 'emit', '--root', str(self.root),
                                  '--default-model', 'opus')

        self.assertIn('model-tiers: design=opus, mechanical=sonnet', out)

    def test_every_stage_records_the_tiers(self):
        for stage in MODULE.STAGE_ORDER:
            with self.subTest(stage=stage):
                self.assertIn('model-tiers', [key for key, _, _ in MODULE.STAGES[stage]])


class ReportTests(Fixture):
    """The two output shapes, and the promise that warnings alone still exit 0."""

    def test_json_keys_every_check_and_matches_the_exit_code(self):
        with mock.patch.dict(os.environ, self.resolved()):
            code, out = self.run_cli(GEAR, '--stage', 'emit', '--root', str(self.root),
                                     '--format', 'json')

        report = json.loads(out)
        self.assertEqual(report['gear'], GEAR)
        self.assertEqual(report['stage'], 'emit')
        self.assertEqual(report['ready'], code == 0)
        keys = [c['key'] for c in report['checks']]
        self.assertEqual(sorted(keys), sorted(k for k, _, _ in MODULE.STAGES['emit']))
        for check in report['checks']:
            self.assertIn(check['status'], (MODULE.OK, MODULE.WARN, MODULE.FAIL, MODULE.SKIP))
            self.assertTrue(check['detail'])

    def test_warnings_alone_leave_the_exit_code_at_zero(self):
        cache = self.root / 'cache'
        cache.mkdir()
        env = {'XDG_CACHE_HOME': str(cache),
               'FUSION_QUERY_API': str(self.root / 'nowhere.py')}
        with mock.patch.dict(os.environ, env):
            os.environ.pop('FUSION_API_STUBS', None)
            os.environ.pop('SKETCH_DIR', None)
            code, out = self.run_cli(GEAR, '--stage', 'generate', '--root', str(self.root))

        self.assertEqual(code, 0, out)
        self.assertIn('[warn]', out)
        self.assertNotIn('[FAIL]', out)
        self.assertIn('READY', out)

    def test_text_report_counts_the_failures(self):
        with mock.patch.dict(os.environ, self.resolved()):
            code, out = self.run_cli(GEAR, '--stage', 'emit', '--root', str(self.root))

        self.assertEqual(code, 1)
        self.assertIn('NOT READY (2 failures)', out)  # steps.md and the proof directory


if __name__ == '__main__':
    unittest.main()
