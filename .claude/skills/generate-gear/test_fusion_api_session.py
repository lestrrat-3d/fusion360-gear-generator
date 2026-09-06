"""Transport checks for the run-scoped Fusion API query session."""
import io
import json
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


if __name__ == '__main__':
    unittest.main()
