#!/usr/bin/env python3
"""CLI checks for the shared Fusion API support decision."""
import contextlib
import importlib.util
import io
import json
import os
import unittest
from pathlib import Path
from unittest import mock


CLI_PATH = Path(__file__).with_name('query_api_status.py')
CLI_SPEC = importlib.util.spec_from_file_location('query_api_status', CLI_PATH)
CLI = importlib.util.module_from_spec(CLI_SPEC)
CLI_SPEC.loader.exec_module(CLI)


def result(disposition, status):
    return {
        'schema': 1,
        'owner': 'adsk.fusion.WidgetTools',
        'name': 'addWidget',
        'status': status,
        'scope': 'receiver',
        'disposition': disposition,
        'declared_on': None,
        'returns': None,
        'evidence': [],
        'stale_watchlist': False,
    }


class QueryApiStatusTest(unittest.TestCase):
    def run_cli(self, value):
        output = io.StringIO()
        with mock.patch.object(CLI.fusion_api, 'describe_call', return_value=value), \
                mock.patch.object(CLI.fusion_api, 'query_session', return_value=contextlib.nullcontext()), \
                contextlib.redirect_stdout(output):
            code = CLI.main([
                '--owner', 'adsk.fusion.WidgetTools', '--member', 'addWidget'])
        return code, output.getvalue()

    def test_allow_and_advisory_exit_zero_with_sorted_json(self):
        for disposition, status in (('allow', 'documented'), ('advisory', 'unverified')):
            with self.subTest(disposition=disposition):
                code, output = self.run_cli(result(disposition, status))
                self.assertEqual(code, 0)
                self.assertEqual(json.loads(output)['status'], status)
                self.assertEqual(output, json.dumps(result(disposition, status), sort_keys=True) + '\n')

    def test_block_exits_one(self):
        code, _ = self.run_cli(result('block', 'not_found'))
        self.assertEqual(code, 1)

    def test_setup_error_exits_two(self):
        code, output = self.run_cli(result('setup_error', 'unavailable'))
        self.assertEqual(code, 2)
        self.assertEqual(json.loads(output)['status'], 'unavailable')

    def test_session_entry_failure_exits_two_with_original_reason(self):
        output = io.StringIO()
        with mock.patch.object(
                CLI.fusion_api, 'query_session',
                side_effect=CLI.fusion_api.Unavailable('invalid transport')), \
                contextlib.redirect_stdout(output):
            code = CLI.main([
                '--owner', 'adsk.fusion.WidgetTools', '--member', 'addWidget'])

        self.assertEqual(code, 2)
        parsed = json.loads(output.getvalue())
        self.assertEqual((parsed['status'], parsed['disposition']),
                         ('unavailable', 'setup_error'))
        self.assertEqual(parsed['evidence'], ['invalid transport'])

    def test_refuted_pair_wins_before_invalid_transport_is_used(self):
        output = io.StringIO()
        with mock.patch.dict(
                os.environ, {CLI.fusion_api.QUERY_TRANSPORT: 'invalid'}), \
                contextlib.redirect_stdout(output):
            code = CLI.main(['--owner', 'adsk.core.Base', '--member', 'cast'])

        self.assertEqual(code, 1)
        self.assertEqual(json.loads(output.getvalue())['status'], 'refuted')

    def test_invalid_owner_exits_two(self):
        with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit) as raised:
            CLI.main(['--owner', 'Sketch', '--member', 'project'])
        self.assertEqual(raised.exception.code, 2)


if __name__ == '__main__':
    unittest.main()
