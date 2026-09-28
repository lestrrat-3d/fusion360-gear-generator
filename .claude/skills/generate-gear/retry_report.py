"""Render a smaller drafting retry view from a complete gate runner report."""
import json
import sys
from pathlib import Path


MARKERS = {'compile-gear': 'COMPILE_GATES_JSON: ', 'emit-gear': 'GATES_JSON: '}
RUNNERS = {'compile-gear': 'run_compile_gates:', 'emit-gear': 'run_gates:'}
ROWS = {'compile-gear': 'stages', 'emit-gear': 'gates'}


class RetryReportError(ValueError):
    pass


def compact(report, skill, gear):
    """Return a checked retry view, or None for a legacy hand-written report."""
    marker = MARKERS[skill]
    matches = [line[len(marker):] for line in report.splitlines() if line.startswith(marker)]
    if not matches:
        if report.startswith(tuple(RUNNERS.values())):
            raise RetryReportError('runner report has no structured verdict')
        return None
    if len(matches) != 1 or not report.rstrip().endswith(marker + matches[0]):
        raise RetryReportError('runner report needs one final structured verdict')
    try:
        verdict = json.loads(matches[0])
    except json.JSONDecodeError as exc:
        raise RetryReportError('invalid structured verdict: {}'.format(exc)) from exc
    if not isinstance(verdict, dict) or verdict.get('schema') != 1 or verdict.get('gear') != gear:
        raise RetryReportError('structured verdict has wrong schema or gear')
    if verdict.get('verdict') != 'fail' or verdict.get('exit_code') != 1:
        raise RetryReportError('retry needs a failed draft verdict, not a pass or setup error')
    rows = verdict.get(ROWS[skill])
    if not isinstance(rows, list) or not rows:
        raise RetryReportError('structured verdict has no stage rows')

    lines = ['retry view: {} {}'.format(skill, gear), 'verdict: FAIL']
    if skill == 'compile-gear':
        if verdict.get('iteration_mode'):
            lines.append('iteration base: {}'.format(verdict.get('iteration_base')))
        if verdict.get('effective_proof_scope'):
            lines.append('proof scope: {}'.format(verdict['effective_proof_scope']))
        if 'proof_is_complete' in verdict:
            lines.append('proof is complete: {}'.format(verdict['proof_is_complete']))
        if verdict.get('proof_omission_reason'):
            lines.append('proof omission: {}'.format(verdict['proof_omission_reason']))
    lines.extend(['', 'stage verdicts:'])
    for row in rows:
        if not isinstance(row, dict) or row.get('status') not in ('pass', 'fail', 'skip', 'error', 'note'):
            raise RetryReportError('structured verdict has an invalid stage row')
        status, key = row['status'], row.get('key')
        if not isinstance(key, str) or not key:
            raise RetryReportError('structured verdict has a stage without a key')
        summary = row.get('skip_reason') if status == 'skip' else row.get('headline')
        tag = 'EMIT' if row.get('disposition') == 'emit_required' else status.upper()
        lines.append('  {} {}: {}'.format(tag, key, summary or ''))

    lines.extend(['', 'diagnostics:'])
    diagnostics = 0
    for row in rows:
        status = row['status']
        include = status in ('fail', 'error', 'note') or row.get('disposition') == 'emit_required'
        if skill == 'compile-gear' and status == 'pass' and row['key'] != 'proof':
            include = True  # compile and playbook output can carry non-blocking advisories
        if skill == 'emit-gear' and row.get('advisory') and status == 'pass':
            include = True
        if not include:
            continue
        stdout = (row.get('stdout') or '').splitlines()
        if row.get('headline') in stdout:
            stdout.remove(row['headline'])
        stderr = (row.get('stderr') or '').splitlines()
        if not stdout and not stderr:
            continue
        diagnostics += 1
        lines.append('  {}:'.format(row['key']))
        lines.extend('    ' + line for line in stdout)
        if stderr:
            lines.append('    stderr:')
            lines.extend('    ' + line for line in stderr)
    if not diagnostics:
        lines.append('  (none)')

    faults = [(row['key'], row['fault']) for row in rows if row.get('fault')]
    if skill == 'emit-gear' and verdict.get('classification'):
        faults = []  # the structured classification below includes its reasons
    if faults:
        lines.extend(['', 'first-pass fault classification:'])
        lines.extend('  {}: {}'.format(key, fault) for key, fault in faults)
    if skill == 'compile-gear' and verdict.get('handoff'):
        lines.extend(['', 'handoff: {}'.format(json.dumps(verdict['handoff'], sort_keys=True))])
    if skill == 'emit-gear' and verdict.get('classification'):
        lines.extend(['', 'first-pass fault classification:'])
        lines.extend('  {}: {}: {}'.format(row.get('gate'), row.get('fault'), row.get('why'))
                     for row in verdict['classification'])
    return '\n'.join(lines) + '\n'


def main(argv=None):
    args = sys.argv[1:] if argv is None else argv
    if len(args) != 3 or args[0] not in MARKERS:
        print('usage: retry_report.py <compile-gear|emit-gear> <gear> <full-report>', file=sys.stderr)
        return 2
    skill, gear, report_file = args
    try:
        report = Path(report_file).read_text(encoding='utf-8')
        view = compact(report, skill, gear)
        if view is None:
            raise RetryReportError('full runner report is required')
    except (OSError, UnicodeError, RetryReportError) as exc:
        print('retry_report: {}'.format(exc), file=sys.stderr)
        return 2
    sys.stdout.write(view)
    return 0


if __name__ == '__main__':
    sys.exit(main())
