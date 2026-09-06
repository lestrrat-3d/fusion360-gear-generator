"""Transport checks for the run-scoped Fusion API query session."""
import io
import json
import os
import tempfile
import unittest
from pathlib import Path
from unittest import mock

import fusion_api


class SessionHandshakeTest(unittest.TestCase):
    def test_ready_commands_must_be_a_list_of_strings(self):
        process = mock.Mock()
        process.stdout = io.StringIO(json.dumps({
            'type': 'ready', 'protocol': 1, 'commands': ['show', {}]}) + '\n')
        process.stdin = io.StringIO()
        process.stderr = io.StringIO()
        process.poll.return_value = 0

        with mock.patch.object(fusion_api.subprocess, 'Popen', return_value=process):
            client = fusion_api._QuerySession('/fixture/query_fusion_api.py', 'session')
            with self.assertRaises(fusion_api.Unavailable):
                client._start()
            client.close()


class SessionFixtureTest(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.script = Path(self.directory.name) / 'query.py'
        self.script.write_text(
            "import json, sys\n"
            "if '--help' in sys.argv:\n"
            "    print('  serve')\n"
            "    raise SystemExit(0)\n"
            "if sys.argv[-1] != 'serve':\n"
            "    print('adsk.fusion.%s [method]' % sys.argv[-1])\n"
            "    raise SystemExit(0)\n"
            "print(json.dumps({'type': 'ready', 'protocol': 1, 'commands': "
            "['show', 'members', 'search']}), flush=True)\n"
            "for line in sys.stdin:\n"
            "    request = json.loads(line)\n"
            "    name = request['argv'][-1]\n"
            "    print(json.dumps({'id': request['id'], 'type': 'result', "
            "'returncode': 0, 'stdout': 'adsk.fusion.%s [method]\\n' % name, "
            "'stderr': ''}), flush=True)\n",
            encoding='utf-8')
        self.original_script = fusion_api._script
        fusion_api._script = str(self.script)
        self.addCleanup(self._restore_script)

    def _restore_script(self):
        fusion_api._script = self.original_script

    def _mode(self, mode):
        self.script.write_text(
            ("import json, os, sys, time\n"
            "mode = __MODE__\n"
            "pid_file = os.environ.get('FUSION_QUERY_PID_FILE')\n"
            "if pid_file:\n"
            "    open(pid_file, 'w').write(str(os.getpid()))\n"
            "if '--help' in sys.argv:\n"
            "    if mode != 'old':\n"
            "        print('  serve')\n"
            "    raise SystemExit(0)\n"
            "if mode == 'old':\n"
            "    print('adsk.fusion.%s [method]' % sys.argv[-1])\n"
            "    raise SystemExit(0)\n"
            "if mode == 'db_error':\n"
            "    print(json.dumps({'type': 'error', 'id': None, 'message': 'database unavailable'}), flush=True)\n"
            "    raise SystemExit(1)\n"
            "print(json.dumps({'type': 'ready', 'protocol': 1, 'commands': "
            "['show', 'members', 'search']}), flush=True)\n"
            "if mode == 'eof':\n"
            "    raise SystemExit(0)\n"
            "for line in sys.stdin:\n"
            "    request = json.loads(line)\n"
            "    if mode == 'timeout':\n"
            "        time.sleep(60)\n"
            "        continue\n"
            "    name = request['argv'][-1]\n"
            "    response_id = request['id'] + 1 if mode == 'mismatch' else request['id']\n"
            "    stdout = 'adsk.fusion.%s [method]\\n' % name\n"
            "    if mode == 'malformed':\n"
            "        stdout = 7\n"
            "    print(json.dumps({'id': response_id, 'type': 'result', 'returncode': 0, "
            "'stdout': stdout, 'stderr': ''}), flush=True)\n"
            ).replace('__MODE__', repr(mode)),
            encoding='utf-8')

    def test_lookup_many_uses_one_session_and_keeps_results_associated(self):
        with mock.patch.object(fusion_api.subprocess, 'Popen', wraps=fusion_api.subprocess.Popen) as popen:
            with fusion_api.query_session():
                found = fusion_api.lookup_many(['alpha', 'beta', 'gamma'], workers=3)

        self.assertEqual(set(found), {'alpha', 'beta', 'gamma'})
        self.assertEqual(found['alpha'], [('adsk.fusion.alpha', 'method')])
        self.assertEqual(popen.call_count, 2)  # one capability probe and one session child

    def test_legacy_transport_is_explicit(self):
        with mock.patch.object(fusion_api.subprocess, 'run') as run:
            run.return_value = mock.Mock(stdout='adsk.fusion.alpha [method]\n', returncode=0)
            with fusion_api.query_session(transport='legacy'):
                self.assertEqual(fusion_api.lookup('alpha'), [('adsk.fusion.alpha', 'method')])
        self.assertEqual(run.call_count, 1)
        self.assertNotIn('serve', run.call_args.args[0])

    def test_legacy_transport_launches_once_per_query(self):
        with mock.patch.object(fusion_api.subprocess, 'Popen', wraps=fusion_api.subprocess.Popen) as popen:
            with fusion_api.query_session(transport='legacy'):
                found = fusion_api.lookup_many(['alpha', 'beta', 'gamma'], workers=3)

        self.assertEqual(set(found), {'alpha', 'beta', 'gamma'})
        self.assertEqual(popen.call_count, 3)

    def test_older_tool_automatically_uses_legacy_transport(self):
        self._mode('old')
        with fusion_api.query_session():
            found = fusion_api.lookup('alpha')

        self.assertEqual(found, [('adsk.fusion.alpha', 'method')])

    def test_supported_tool_database_error_does_not_fall_back(self):
        self._mode('db_error')
        with mock.patch.object(fusion_api, '_run_legacy') as legacy:
            with self.assertRaises(fusion_api.Unavailable):
                with fusion_api.query_session():
                    fusion_api.lookup('alpha')

        legacy.assert_not_called()

    def test_malformed_result_is_unavailable(self):
        self._mode('malformed')
        with self.assertRaises(fusion_api.Unavailable):
            with fusion_api.query_session():
                fusion_api.lookup('alpha')

    def test_mismatched_response_id_is_unavailable(self):
        self._mode('mismatch')
        with self.assertRaises(fusion_api.Unavailable):
            with fusion_api.query_session():
                fusion_api.lookup('alpha')

    def test_eof_is_unavailable(self):
        self._mode('eof')
        with self.assertRaises(fusion_api.Unavailable):
            with fusion_api.query_session():
                fusion_api.lookup('alpha')

    def test_timeout_reaps_the_session_child(self):
        self._mode('timeout')
        pid_file = Path(self.directory.name) / 'query.pid'
        old_timeout = fusion_api.SESSION_REQUEST_TIMEOUT
        fusion_api.SESSION_REQUEST_TIMEOUT = 0.05
        self.addCleanup(setattr, fusion_api, 'SESSION_REQUEST_TIMEOUT', old_timeout)
        with mock.patch.dict(os.environ, {'FUSION_QUERY_PID_FILE': str(pid_file)}):
            with self.assertRaises(fusion_api.Unavailable):
                with fusion_api.query_session():
                    fusion_api.lookup('alpha')

        pid = int(pid_file.read_text())
        with self.assertRaises(ProcessLookupError):
            os.kill(pid, 0)


if __name__ == '__main__':
    unittest.main()
