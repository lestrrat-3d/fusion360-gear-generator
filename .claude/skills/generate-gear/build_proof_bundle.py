#!/usr/bin/env python3
"""Export the checked Go declarations needed by an emitter's construction steps.

The manifest pins every canonical input and every included and excluded byte
range. Verification rebuilds the view, so an old or edited bundle fails closed.
The canonical proof files stay in place for the complete proof gate.
"""
import argparse
import hashlib
import json
import re
import subprocess
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
STEP = re.compile(r'^##\s+(\S+)\s+`\[GO\]`', re.MULTILINE)
PROOF = re.compile(r'^Proof function `([A-Za-z_][A-Za-z_0-9]*)` in `(proof/[^`]+\.go)`\.', re.MULTILINE)
GEAR = re.compile(r'[a-z][a-z0-9_]*\Z')
VERSION = 1


class BundleError(Exception):
    pass


def digest(data):
    return hashlib.sha256(data).hexdigest()


def checked_steps(text, gear):
    sections = list(STEP.finditer(text))
    if not sections:
        raise BundleError('no [GO] steps found')
    result = []
    for index, match in enumerate(sections):
        end = sections[index + 1].start() if index + 1 < len(sections) else len(text)
        refs = PROOF.findall(text[match.end():end])
        if len(refs) != 1:
            raise BundleError('step {} needs exactly one Proof function reference'.format(match.group(1)))
        function, path = refs[0]
        if not path.startswith('proof/{}/'.format(gear)):
            raise BundleError('step {} references proof outside {}'.format(match.group(1), gear))
        result.append({'step': match.group(1), 'function': function, 'file': path})
    if len({row['step'] for row in result}) != len(result):
        raise BundleError('duplicate [GO] step ID')
    return result


def parse_sources(root, paths):
    executable = root / '.tmp/proof-bundle-ast'
    source = HERE / 'proof_bundle_ast.go'
    result = subprocess.run(['go', 'build', '-o', str(executable), str(source)],
                            cwd=root, capture_output=True, text=True)
    if result.returncode:
        raise BundleError('Go parser build failed: {}'.format(result.stderr.strip()))
    result = subprocess.run([str(executable), *paths], cwd=root, capture_output=True, text=True)
    if result.returncode:
        raise BundleError('Go proof parse failed: {}'.format(result.stderr.strip()))
    return json.loads(result.stdout)


def select_declarations(sources, steps):
    declarations = [decl for source in sources for decl in source['declarations']]
    by_name = {}
    by_file = {}
    for index, decl in enumerate(declarations):
        by_file.setdefault(decl['file'], []).append(index)
        for name in decl['names']:
            if name in by_name:
                raise BundleError('duplicate proof declaration: {}'.format(name))
            by_name[name] = index

    roots = set()
    for row in steps:
        index = by_name.get(row['function'])
        if index is None or declarations[index]['file'] != row['file']:
            raise BundleError('missing proof function {} in {}'.format(row['function'], row['file']))
        roots.add(index)
        registrations = [i for i in by_file.get('proof/{}/zz_registrations_test.go'.format(
            row['file'].split('/')[1]), []) if row['function'] in declarations[i]['refs']]
        if len(registrations) != 1:
            raise BundleError('proof function {} needs one checked registration'.format(row['function']))
        # The registration proves the step is gated, but its case tables and
        # assertion callbacks are validation inputs, not construction inputs.

    for source in sources:
        missing = sorted(set(source['unresolved']) - set(by_name))
        if missing:
            raise BundleError('unresolved proof symbol(s) in {}: {}'.format(
                source['file'], ', '.join(missing)))

    selected = set()
    pending = list(roots)
    while pending:
        index = pending.pop()
        if index in selected:
            continue
        selected.add(index)
        decl = declarations[index]
        for name in decl['refs']:
            dependency = by_name.get(name)
            if dependency is not None and dependency not in selected:
                pending.append(dependency)
        receiver = decl.get('receiver')
        if receiver:
            dependency = by_name.get(receiver)
            if dependency is not None and dependency not in selected:
                pending.append(dependency)
        for other, candidate in enumerate(declarations):
            if candidate.get('receiver') and candidate['receiver'] in decl['names']:
                pending.append(other)
    closure = []
    for index in sorted(selected):
        decl = declarations[index]
        dependencies = sorted({name for name in decl['refs'] if name in by_name
                               and by_name[name] != index})
        closure.append({'file': decl['file'], 'start': decl['start'], 'end': decl['end'],
                        'names': decl['names'], 'depends_on': dependencies,
                        'step_root': index in roots})
    return declarations, selected, closure


def ranges(size, selected):
    cursor = 0
    output = []
    for start, end in sorted(selected):
        if start < cursor or end > size or end <= start:
            raise BundleError('invalid or overlapping Go declaration range')
        if cursor < start:
            output.append({'start': cursor, 'end': start, 'included': False})
        output.append({'start': start, 'end': end, 'included': True})
        cursor = end
    if cursor < size:
        output.append({'start': cursor, 'end': size, 'included': False})
    return output


def build(root, gear):
    if not GEAR.fullmatch(gear):
        raise BundleError('invalid gear name')
    step_path = 'spec/{}/steps.md'.format(gear)
    step_bytes = (root / step_path).read_bytes()
    steps = checked_steps(step_bytes.decode('utf-8'), gear)
    proof_paths = sorted(path.relative_to(root).as_posix()
                         for path in (root / 'proof' / gear).glob('*.go'))
    if not proof_paths:
        raise BundleError('no Go proof files for {}'.format(gear))
    sources = parse_sources(root, proof_paths)
    declarations, selected, closure = select_declarations(sources, steps)
    files = []
    fragments = ['# Checked construction proof view\n\n',
                 'Canonical source: spec/{}/steps.md and proof/{}/.\n'.format(gear, gear),
                 'Use the full proof gate for validation.\n\n']
    for source in sources:
        path = source['file']
        data = (root / path).read_bytes()
        chosen = [(decl['start'], decl['end']) for index, decl in enumerate(declarations)
                  if index in selected and decl['file'] == path]
        source_ranges = ranges(len(data), chosen)
        for item in source_ranges:
            item['sha256'] = digest(data[item['start']:item['end']])
            if not item['included']:
                continue
            part = data[item['start']:item['end']]
            start_line = data.count(b'\n', 0, item['start']) + 1
            end_line = data.count(b'\n', 0, item['end']) + 1
            fragments.extend(['## {}:{}-{}\n\n'.format(path, start_line, end_line),
                              '```go\n', part.decode('utf-8'), '\n```\n\n'])
        files.append({'path': path, 'sha256': digest(data), 'size': len(data),
                      'ranges': source_ranges})
    bundle = ''.join(fragments).encode('utf-8')
    manifest = {'version': VERSION, 'gear': gear,
                'steps': {'path': step_path, 'sha256': digest(step_bytes)},
                'coverage': steps, 'declarations': closure, 'files': files,
                'bundle_sha256': digest(bundle)}
    return bundle, (json.dumps(manifest, indent=2, sort_keys=True) + '\n').encode('utf-8')


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', choices=('build', 'verify'))
    parser.add_argument('gear')
    parser.add_argument('--root', type=Path, default=Path.cwd())
    args = parser.parse_args(argv)
    root = args.root.resolve()
    bundle_path = root / '.tmp/{}.proof-bundle.md'.format(args.gear)
    manifest_path = root / '.tmp/{}.proof-bundle.json'.format(args.gear)
    try:
        bundle, manifest = build(root, args.gear)
        if args.action == 'build':
            bundle_path.parent.mkdir(exist_ok=True)
            bundle_path.write_bytes(bundle)
            manifest_path.write_bytes(manifest)
        elif bundle_path.read_bytes() != bundle or manifest_path.read_bytes() != manifest:
            raise BundleError('proof bundle or input digests changed; rebuild before emission')
    except (BundleError, OSError, UnicodeError) as exc:
        print('proof bundle: {}'.format(exc), file=sys.stderr)
        return 2
    print('{}: {} bytes, {} checked steps'.format(bundle_path, len(bundle),
                                                    len(json.loads(manifest)['coverage'])))
    return 0


if __name__ == '__main__':
    sys.exit(main())
