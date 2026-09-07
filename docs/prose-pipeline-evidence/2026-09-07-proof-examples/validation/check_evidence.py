from pathlib import Path
import hashlib
import json

base = Path('docs/prose-pipeline-evidence/2026-09-07-proof-examples')
records = sorted(base.glob('spurgear/*/trial.json'))
assert len(records) == 2
intervals = []
caches = []
count = 0
for path in records:
    record = json.loads(path.read_text())
    assert record['trial'] == 'warm-up' and record['scored'] is False
    assert record['status'] in ('failed', 'setup_error')
    assert record['completed_draft_rounds'] == 0 and record['gate_reports_observed'] == 0
    assert record['compile_output_digests'] == record['emit_input_digests'] == []
    assert record['final_artifact_digests'] == []
    assert all(record[k] is None for k in ('emit_run_id', 'final_compile_report', 'final_emit_report',
                                         'input_tokens', 'output_tokens', 'cost', 'compile_first_pass'))
    directory = path.parent
    manifest = json.loads((directory / 'raw-evidence-digests.json').read_text())
    actual = {str(p.relative_to(directory)) for p in directory.rglob('*')
              if p.is_file() and p.name != 'raw-evidence-digests.json'}
    assert {entry['path'] for entry in manifest} == actual
    for entry in manifest:
        raw = (directory / entry['path']).read_bytes()
        assert hashlib.sha256(raw).hexdigest() == entry['sha256']
        assert len(raw) == entry['bytes']
        count += 1
    for p in directory.glob('*.raw.json'):
        wrapper = json.loads(p.read_text())
        raw = ''.join(wrapper['lines']).encode('utf-8')
        assert len(raw) == wrapper['original_bytes']
        assert hashlib.sha256(raw).hexdigest() == wrapper['original_sha256']
    run = json.loads((directory / 'compile/run.json').read_text())
    assert run['run_id'] == record['compile_run_id']
    assert run['starting_git_commit'] == record['commit']
    events = [json.loads(p.read_text()) for p in (directory / 'compile/events').glob('*.json')]
    starts = [e['timestamp_epoch_s'] for e in events if e['phase'] == 'overall' and e['action'] == 'start']
    ends = [e['timestamp_epoch_s'] for e in events if e['phase'] == 'overall' and e['action'] == 'finish']
    assert len(starts) == len(ends) == 1
    assert abs(ends[0] - starts[0] - record['serial_wall_time_s']) < 1e-5
    intervals.append((starts[0], ends[0]))
    env = json.loads((directory / 'environment.json').read_text())
    assert env['GIT_CEILING_DIRECTORIES'] == env['TMPDIR']
    assert env['GOTOOLCHAIN'] == 'go1.26.8' and env['GOMAXPROCS'] == '8'
    caches.append((env['GOCACHE'], env['TMPDIR']))
    for p in directory.glob('*verification.json'):
        assert json.loads(p.read_text())['mismatches'] == []
intervals.sort()
assert intervals[0][1] < intervals[1][0]
assert len({p[0] for p in caches}) == len({p[1] for p in caches}) == 2
print(f'PASS: {len(records)} actual partial trial records, {count} raw hashes, exact wrappers, serial intervals.')
