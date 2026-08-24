#!/usr/bin/env python3
"""Regression tests for pick_model.py.

Two invariants the suite defends:

  * the tier follows the role and the session default, never a pinned model
    name — the whole reason the script exists is that "spawn on haiku" is
    wrong on any session whose default is not one rung above haiku;
  * stdout carries the model name and nothing else, because a SKILL.md tells
    the orchestrator to read one token from it.
"""
import contextlib
import importlib.util
import io
import unittest
from pathlib import Path


def _load(name):
    path = Path(__file__).with_name('%s.py' % name)
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


pick_model = _load('pick_model')


def run(argv):
    """Invoke main() and return (exit code, stdout, stderr)."""
    out, err = io.StringIO(), io.StringIO()
    code = pick_model.main(argv, out=out, err=err)
    return code, out.getvalue(), err.getvalue()


class LadderTests(unittest.TestCase):
    def test_step_down_walks_the_ladder(self):
        self.assertEqual(pick_model.step_down('opus'), 'sonnet')
        self.assertEqual(pick_model.step_down('sonnet'), 'haiku')

    def test_step_down_at_the_bottom_is_none(self):
        self.assertIsNone(pick_model.step_down('haiku'))

    def test_step_down_off_the_ladder_is_none(self):
        self.assertIsNone(pick_model.step_down('fable'))
        self.assertIsNone(pick_model.step_down('something-unreleased'))

    def test_ladder_runs_highest_to_lowest(self):
        self.assertEqual(pick_model.LADDER[0], 'opus')
        self.assertEqual(pick_model.LADDER[-1], 'haiku')


class DesignRoleTests(unittest.TestCase):
    def test_design_returns_the_default_untouched(self):
        for default in ('opus', 'sonnet', 'haiku', 'fable'):
            with self.subTest(default=default):
                model, _ = pick_model.resolve('design', default)
                self.assertEqual(model, default)

    def test_orchestrator_is_an_alias_of_design(self):
        self.assertEqual(pick_model.resolve('orchestrator', 'opus')[0], 'opus')
        self.assertEqual(pick_model.resolve('orchestrator', 'haiku')[0], 'haiku')

    def test_escalated_does_not_disturb_design(self):
        model, _ = pick_model.resolve('design', 'opus', escalated=True)
        self.assertEqual(model, 'opus')


class MechanicalRoleTests(unittest.TestCase):
    def test_mechanical_steps_down_one_rung(self):
        self.assertEqual(pick_model.resolve('mechanical', 'opus')[0], 'sonnet')
        self.assertEqual(pick_model.resolve('mechanical', 'sonnet')[0], 'haiku')

    def test_mechanical_at_the_floor_keeps_the_default(self):
        model, reason = pick_model.resolve('mechanical', 'haiku')
        self.assertEqual(model, 'haiku')
        self.assertIn('bottom of the ladder', reason)

    def test_mechanical_off_the_ladder_keeps_the_default(self):
        model, reason = pick_model.resolve('mechanical', 'fable')
        self.assertEqual(model, 'fable')
        self.assertIn('not on the ladder', reason)

    def test_an_opus_session_never_lands_on_haiku(self):
        # The bug this script was written for: a pinned `haiku` drops an Opus
        # session two rungs instead of one.
        self.assertNotEqual(pick_model.resolve('mechanical', 'opus')[0], 'haiku')

    def test_escalated_returns_the_session_default(self):
        model, reason = pick_model.resolve('mechanical', 'opus', escalated=True)
        self.assertEqual(model, 'opus')
        self.assertIn('escalated', reason)


class StreamTests(unittest.TestCase):
    def test_stdout_is_only_the_model_name(self):
        code, out, _ = run(['--role', 'mechanical', '--default', 'opus'])
        self.assertEqual(code, 0)
        self.assertEqual(out, 'sonnet\n')

    def test_the_reason_goes_to_stderr(self):
        _, out, err = run(['--role', 'mechanical', '--default', 'opus'])
        self.assertNotIn('steps', out)
        self.assertIn('pick_model:', err)

    def test_escalated_flag_reaches_the_resolution(self):
        _, out, _ = run(['--role', 'mechanical', '--default', 'opus', '--escalated'])
        self.assertEqual(out, 'opus\n')


class UsageTests(unittest.TestCase):
    def _usage_failure(self, argv):
        err = io.StringIO()
        with contextlib.redirect_stderr(err):
            with self.assertRaises(SystemExit) as caught:
                pick_model.parse_args(argv)
        self.assertNotIn(caught.exception.code, (0, None))

    def test_unknown_role_is_rejected(self):
        self._usage_failure(['--role', 'bogus', '--default', 'opus'])

    def test_missing_default_is_rejected(self):
        self._usage_failure(['--role', 'mechanical'])

    def test_missing_role_is_rejected(self):
        self._usage_failure(['--default', 'opus'])

    def test_an_unreleased_default_is_accepted(self):
        # The ladder is allowed to lag the models a harness offers, so an
        # unknown name must fall back rather than fail the run.
        args = pick_model.parse_args(['--role', 'mechanical', '--default', 'newthing'])
        self.assertEqual(args.default, 'newthing')


if __name__ == '__main__':
    unittest.main()
