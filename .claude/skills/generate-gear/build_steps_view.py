#!/usr/bin/env python3
"""Build and verify an emitter-facing view of the canonical compiled steps."""
import argparse
import hashlib
import json
import re
import sys
from pathlib import Path

import step_metadata


GEAR = re.compile(r'[a-z][a-z0-9_]*\Z')
SECTION = re.compile(r'^## (.+)$', re.M)
FROM_BLOCK = re.compile(r'^\*\*From:\*\*.*?(?=\n\s*\n|\Z)', re.M | re.S)
VERSION = 1


class ViewError(ValueError):
    pass


def digest(data):
    return hashlib.sha256(data).hexdigest()


def sections(text):
    headings = list(SECTION.finditer(text))
    return [(heading.group(1), text[heading.start():
            headings[index + 1].start() if index + 1 < len(headings) else len(text)])
            for index, heading in enumerate(headings)]


def compact_step(title, body, version):
    if version == 2:
        try:
            payload = step_metadata.parse_step(body, version)
            _, start, end, _ = step_metadata._metadata_comment(body)
        except step_metadata.MetadataError as exc:
            raise ViewError('{}: {}'.format(title, exc)) from exc
        calls = ['- `{}`: `{}`{}{}'.format(
            call['role'], call['span'],
            ' on `{}`'.format(call['owner']) if call['owner'] else '',
            ' when {}'.format(call['condition']) if call['condition'] else
            ' ({})'.format(call['reason']) if call['reason'] else '')
            for call in payload['calls']]
        body = body[:start] + body[end:]
        if calls:
            body = body.rstrip() + '\n\nChecked call roles:\n' + '\n'.join(calls) + '\n'
    return FROM_BLOCK.sub('', body).rstrip() + '\n'


def build(root, gear):
    if not GEAR.fullmatch(gear):
        raise ViewError('invalid gear name')
    source_path = root / 'spec' / gear / 'steps.md'
    source = source_path.read_bytes()
    text = source.decode('utf-8')
    first_step = step_metadata.STEP_HEADING.search(text)
    if first_step is None:
        raise ViewError('no compiled steps found')
    preamble = text[:first_step.start()]
    named_sections = sections(preamble)
    names = [name for name, _ in named_sections]
    if len(names) != len(set(names)):
        raise ViewError('repeated pre-step section heading')
    unexpected = sorted(set(names) - {'Provenance', 'Compilation contract', 'Exact values'})
    if unexpected:
        raise ViewError('unrecognized pre-step section: {}'.format(unexpected[0]))
    preamble_sections = dict(named_sections)
    if 'Provenance' not in preamble_sections:
        raise ViewError('compiled steps have no provenance section')
    has_exact_values = 'Exact values' in preamble_sections
    if has_exact_values and 'Compilation contract' not in preamble_sections:
        raise ViewError('exact values need a compilation contract for the emitter view')
    version = 2 if '<!-- step-metadata: 2 -->' in preamble else 1

    parts = ['# Checked emitter step view\n\n',
             'Canonical source: spec/{}/steps.md. Complete gates read the canonical file.\n'.format(gear),
             'Use each step in order; checked call roles below each step preserve the required conditions.\n\n']
    first_section = SECTION.search(preamble)
    introduction = re.sub(r'^<!-- step-metadata: 2 -->\n?', '',
                          preamble[:first_section.start()], flags=re.M).strip()
    if introduction.startswith('The proof files are ') and '\n' not in introduction:
        introduction = ''
    if introduction:
        parts.append(introduction + '\n\n')
    if 'Compilation contract' in preamble_sections:
        parts.append(preamble_sections['Compilation contract'].rstrip() + '\n\n')
    if has_exact_values:
        parts.append('## Deterministic setup\n\n')
        parts.append('The canonical Exact values section is rendered into Python after drafting. '
                     'Use the contract constants and complete the geometry and selection handling. '
                     'Leave the setup method bodies for the renderer as the emitter prompt directs.\n\n')

    headings = list(step_metadata.STEP_HEADING.finditer(text))
    for index, heading in enumerate(headings):
        end = headings[index + 1].start() if index + 1 < len(headings) else len(text)
        title = text[heading.start():heading.end()]
        body = text[heading.end():end]
        parts.append(title + '\n' + compact_step(title, body, version) + '\n')
    view = ''.join(parts).encode('utf-8')
    manifest = {'version': VERSION, 'gear': gear,
                'source': {'path': source_path.relative_to(root).as_posix(), 'sha256': digest(source)},
                'steps': len(headings), 'view_sha256': digest(view)}
    return view, (json.dumps(manifest, indent=2, sort_keys=True) + '\n').encode('utf-8')


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', choices=('build', 'verify'))
    parser.add_argument('gear')
    parser.add_argument('--root', type=Path, default=Path.cwd())
    args = parser.parse_args(argv)
    root = args.root.resolve()
    view_path = root / '.tmp/{}.steps-view.md'.format(args.gear)
    manifest_path = root / '.tmp/{}.steps-view.json'.format(args.gear)
    try:
        view, manifest = build(root, args.gear)
        if args.action == 'build':
            view_path.parent.mkdir(exist_ok=True)
            if not view_path.exists() or view_path.read_bytes() != view:
                view_path.write_bytes(view)
            if not manifest_path.exists() or manifest_path.read_bytes() != manifest:
                manifest_path.write_bytes(manifest)
        elif view_path.read_bytes() != view or manifest_path.read_bytes() != manifest:
            raise ViewError('step view or canonical source changed; rebuild before emission')
    except (OSError, UnicodeError, ViewError) as exc:
        print('step view: {}'.format(exc), file=sys.stderr)
        return 2
    print('{}: {} bytes, {} steps'.format(view_path, len(view), json.loads(manifest)['steps']))
    return 0


if __name__ == '__main__':
    sys.exit(main())
