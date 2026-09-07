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
import json
import tempfile
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


class MappingTests(unittest.TestCase):
    def setUp(self):
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        self.root = Path(directory.name)
        self.mapping = {
            'schema': 1,
            'mechanical': {'fixture-large': 'fixture-small'},
        }

    def write_mapping(self, value):
        path = self.root / 'mapping.json'
        if isinstance(value, str):
            path.write_text(value, encoding='utf-8')
        else:
            path.write_text(json.dumps(value), encoding='utf-8')
        return path

    def test_load_mapping_returns_the_complete_validated_object(self):
        path = self.write_mapping(self.mapping)

        self.assertEqual(pick_model.load_mapping(path), self.mapping)

    def test_mapped_mechanical_uses_the_exact_default(self):
        model, reason = pick_model.resolve(
            'mechanical', 'fixture-large', mapping=self.mapping)

        self.assertEqual(model, 'fixture-small')
        self.assertIn('mapping', reason)

    def test_design_ignores_mapping_target(self):
        model, _ = pick_model.resolve('design', 'fixture-large', mapping=self.mapping)

        self.assertEqual(model, 'fixture-large')

    def test_escalation_precedes_mapping(self):
        model, reason = pick_model.resolve(
            'mechanical', 'fixture-large', escalated=True, mapping=self.mapping)

        self.assertEqual(model, 'fixture-large')
        self.assertIn('escalated', reason)

    def test_unmapped_default_uses_the_legacy_fallback(self):
        self.assertEqual(
            pick_model.resolve('mechanical', 'opus', mapping=self.mapping),
            pick_model.resolve('mechanical', 'opus'),
        )

    def test_rejects_duplicate_keys(self):
        for document in (
                '{"schema": 1, "schema": 1, "mechanical": {}}',
                '{"schema": 1, "mechanical": {"a": "b", "a": "c"}}'):
            with self.subTest(document=document):
                with self.assertRaises(pick_model.MappingError):
                    pick_model.load_mapping(self.write_mapping(document))

    def test_rejects_nonstandard_json_constants(self):
        with self.assertRaises(pick_model.MappingError):
            pick_model.load_mapping(self.write_mapping(
                '{"schema": 1, "mechanical": {"source": NaN}}'))

    def test_rejects_overlong_json_integer(self):
        document = '{"schema": %s, "mechanical": {}}' % ('1' * 5000)

        with self.assertRaises(pick_model.MappingError):
            pick_model.load_mapping(self.write_mapping(document))

    def test_rejects_invalid_utf8(self):
        path = self.root / 'mapping.json'
        path.write_bytes(b'\xff')

        with self.assertRaises(pick_model.MappingError):
            pick_model.load_mapping(path)

    def test_rejects_wrong_shapes_and_model_identifiers(self):
        bad_mappings = (
            [],
            {'schema': True, 'mechanical': {}},
            {'schema': 2, 'mechanical': {}},
            {'schema': 1},
            {'schema': 1, 'mechanical': {}, 'extra': None},
            {'schema': 1, 'mechanical': []},
            {'schema': 1, 'mechanical': {'': 'target'}},
            {'schema': 1, 'mechanical': {'source model': 'target'}},
            {'schema': 1, 'mechanical': {'source': ''}},
            {'schema': 1, 'mechanical': {'source': 'target\nmodel'}},
            {'schema': 1, 'mechanical': {'source': 1}},
        )
        for mapping in bad_mappings:
            with self.subTest(mapping=mapping):
                with self.assertRaises(pick_model.MappingError):
                    pick_model.load_mapping(self.write_mapping(mapping))

    def test_bad_mapping_exits_two_without_stdout_for_any_role(self):
        path = self.write_mapping('{bad')
        for role in ('design', 'orchestrator', 'mechanical'):
            with self.subTest(role=role):
                code, out, err = run([
                    '--role', role, '--default', 'fixture-large', '--mapping', str(path),
                ])
                self.assertEqual(code, 2)
                self.assertEqual(out, '')
                self.assertIn('pick_model.py:', err)

    def test_escaped_lone_surrogate_exits_two_without_stdout(self):
        path = self.write_mapping(
            '{"schema": 1, "mechanical": {"fixture-large": "\\ud800"}}')

        code, out, err = run([
            '--role', 'mechanical', '--default', 'fixture-large',
            '--mapping', str(path),
        ])

        self.assertEqual(code, 2)
        self.assertEqual(out, '')
        self.assertIn('pick_model.py:', err)

    def test_ordinary_unicode_identifiers_are_allowed(self):
        expected = {
            'schema': 1,
            'mechanical': {'大': '小'},
        }
        mapping = pick_model.load_mapping(self.write_mapping(expected))

        model, _ = pick_model.resolve('mechanical', '大', mapping=mapping)

        self.assertEqual(model, '小')

    def test_unreadable_mapping_exits_two_without_stdout(self):
        code, out, _ = run([
            '--role', 'mechanical', '--default', 'fixture-large',
            '--mapping', str(self.root / 'missing.json'),
        ])

        self.assertEqual(code, 2)
        self.assertEqual(out, '')


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

    def test_mapping_flag_reaches_the_resolution(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'mapping.json'
            path.write_text(
                '{"schema": 1, "mechanical": {"fixture-large": "fixture-small"}}',
                encoding='utf-8')
            _, out, _ = run([
                '--role', 'mechanical', '--default', 'fixture-large',
                '--mapping', str(path),
            ])
        self.assertEqual(out, 'fixture-small\n')


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
