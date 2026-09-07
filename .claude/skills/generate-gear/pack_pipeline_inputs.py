#!/usr/bin/env python3
"""Pack one drafting stage's complete text inputs into a deterministic bundle.

Run from the repository root. The bundle contains no source-path headers: manifest.json maps
each logical source path to ordered UTF-8 chunks whose concatenation is the original file.
"""
import argparse
import hashlib
import json
import os
import shutil
import stat
import subprocess
import sys
import tempfile
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import provenance  # noqa: E402
import render_prompt  # noqa: E402


CHUNK_BYTES = 12_000
FORMATS = 'docs/prose-pipeline-handoffs/formats.md'
CONSTRUCTION = 'proof/examples/CONSTRUCTION.md'
CONSTRUCTION_EXAMPLE = 'proof/examples/proofkit3d_construction_example_test.go'
INVOLUTE = 'proof/involute/involute.go'
REGISTRATIONS = 'zz_registrations_test.go'
STAGE_MANIFEST = 'stage-manifest.json'

COMPILE_HARNESS_DIRECTORIES = ('proof/proofkit', 'proof/proofkit3d')
COMPILE_FIXED_INPUTS = (CONSTRUCTION, CONSTRUCTION_EXAMPLE, FORMATS, INVOLUTE)
EMIT_FIXED_INPUTS = (
    FORMATS,
    'lib/geargen/base.py',
    'lib/geargen/misc.py',
    'lib/geargen/utilities.py',
    'lib/geargen/spurproxy.py',
)
EMIT_FRAMEWORK_DIRECTORIES = ('lib/fusion360utils',)


class PackError(Exception):
    """A bundle input or output request is unusable; the CLI exits 2."""


class ScaffoldCheckError(Exception):
    """Registration omission was requested without a successful scaffold check."""


def _sha256(data):
    return hashlib.sha256(data).hexdigest()


def _logical_path(path):
    return Path(path).as_posix()


def _physical_path(root, logical):
    candidate = Path(logical)
    if candidate.is_absolute() or '..' in candidate.parts:
        raise PackError('input path is not repository-relative: {}'.format(logical))
    return root / candidate


def _read_text_bytes(root, logical):
    path = _physical_path(root, logical)
    try:
        mode = path.stat().st_mode
    except OSError as exc:
        raise PackError('cannot stat required input {}: {}'.format(logical, exc))
    if not stat.S_ISREG(mode):
        raise PackError('required input is not a regular file: {}'.format(logical))
    try:
        data = path.read_bytes()
    except OSError as exc:
        raise PackError('cannot read required input {}: {}'.format(logical, exc))
    try:
        data.decode('utf-8')
    except UnicodeDecodeError as exc:
        raise PackError('required input is not valid UTF-8: {}: {}'.format(logical, exc))
    return data


def _expand(root, logical_directory, suffix=None, excluded=()):
    directory = _physical_path(root, logical_directory)
    try:
        directory_mode = directory.stat().st_mode
    except OSError as exc:
        raise PackError('cannot inspect required input directory {}: {}'.format(
            logical_directory, exc))
    if not stat.S_ISDIR(directory_mode):
        raise PackError('required input directory is not a directory: {}'.format(
            logical_directory))
    excluded = set(excluded)
    paths = []
    try:
        candidates = directory.rglob('*')
        for candidate in candidates:
            relative = candidate.relative_to(root).as_posix()
            if relative in excluded:
                continue
            try:
                candidate_mode = candidate.lstat().st_mode
            except OSError as exc:
                raise PackError('cannot inspect required input {}: {}'.format(relative, exc))
            if stat.S_ISDIR(candidate_mode):
                continue
            matches = suffix is None or candidate.suffix == suffix
            if not matches:
                continue
            if not stat.S_ISREG(candidate_mode):
                raise PackError('required input is not a regular file: {}'.format(relative))
            paths.append(relative)
    except OSError as exc:
        raise PackError('cannot expand required input directory {}: {}'.format(
            logical_directory, exc))
    paths.sort()
    if not paths:
        detail = ' matching {}'.format(suffix) if suffix else ''
        raise PackError('required input directory {} has no{} files'.format(
            logical_directory, detail))
    return paths


def _rendered_prompt(root, stage, gear):
    skill = '{}-gear'.format(stage)
    logical = '.claude/skills/{}/prompt.md'.format(skill)
    template = _read_text_bytes(root, logical).decode('utf-8')
    try:
        return render_prompt.render(template, {'gear': gear}).encode('utf-8')
    except render_prompt.RenderError as exc:
        raise PackError('cannot render standard {} prompt: {}'.format(stage, exc))


def _call_provenance_inputs(root, gear):
    previous = Path.cwd()
    try:
        os.chdir(str(root))
        return set(provenance.provenance_inputs(gear))
    except OSError as exc:
        raise PackError('cannot discover provenance inputs: {}'.format(exc))
    finally:
        os.chdir(str(previous))


def _resolve_references(root, source_logical):
    """Reject unresolved Markdown references using provenance's exact candidate order."""
    text = _read_text_bytes(root, source_logical).decode('utf-8')
    source = Path(source_logical)
    for reference in provenance.DOCUMENT_REF.findall(text):
        normal = (
            Path(os.path.normpath(str(source.parent / reference))),
            Path(os.path.normpath(reference)),
        )
        resolved = False
        for candidate in normal:
            physical = root / candidate
            if physical.is_file():
                resolved = True
                break
        if not resolved and reference == 'PLAYBOOK.md':
            _read_text_bytes(root, provenance.PLAYBOOK)
            resolved = True
        if not resolved:
            raise PackError('{} references missing document {}'.format(
                source_logical, reference))


def discover_compile_inputs(root, gear):
    instructions = 'spec/{}/instructions.md'.format(gear)
    fusion = 'spec/{}/fusion.md'.format(gear)
    contract = 'spec/{}/contract.json'.format(gear)

    _read_text_bytes(root, instructions)
    _read_text_bytes(root, provenance.PLAYBOOK)
    for logical in COMPILE_FIXED_INPUTS:
        _read_text_bytes(root, logical)

    fusion_path = _physical_path(root, fusion)
    contract_path = _physical_path(root, contract)
    if fusion_path.exists():
        _read_text_bytes(root, fusion)
    if contract_path.exists():
        _read_text_bytes(root, contract)

    _resolve_references(root, instructions)
    if fusion_path.is_file():
        _resolve_references(root, fusion)

    logical_paths = {_logical_path(path) for path in _call_provenance_inputs(root, gear)}
    forbidden_exact = {
        'spec/{}/steps.md'.format(gear),
        'lib/geargen/{}.py'.format(gear),
    }
    forbidden_proof_prefix = 'proof/{}/'.format(gear)
    forbidden = sorted(
        logical for logical in logical_paths
        if logical in forbidden_exact or logical.startswith(forbidden_proof_prefix)
    )
    if forbidden:
        raise PackError(
            'compile provenance references forbidden old output: {}'.format(forbidden[0]))
    logical_paths.update(COMPILE_FIXED_INPUTS)
    for directory in COMPILE_HARNESS_DIRECTORIES:
        logical_paths.update(_expand(root, directory, suffix='.go'))

    sources = [(logical, _read_text_bytes(root, logical)) for logical in logical_paths]
    sources.append(('@rendered-prompt', _rendered_prompt(root, 'compile', gear)))
    return sorted(sources, key=lambda item: item[0])


def discover_emit_inputs(root, gear, omit_registrations=False):
    proof_directory = 'proof/{}'.format(gear)
    registration = '{}/{}'.format(proof_directory, REGISTRATIONS)
    excluded = {'{}/{}'.format(proof_directory, STAGE_MANIFEST)}
    if omit_registrations:
        excluded.add(registration)

    logical_paths = {
        'spec/{}/steps.md'.format(gear),
        '.tmp/{}.playbook-extract.md'.format(gear),
        '.tmp/{}.emitter-interfaces.md'.format(gear),
    }
    logical_paths.update(EMIT_FIXED_INPUTS)
    logical_paths.update(_expand(root, proof_directory, excluded=excluded))
    for directory in EMIT_FRAMEWORK_DIRECTORIES:
        logical_paths.update(_expand(root, directory, suffix='.py'))

    sources = [(logical, _read_text_bytes(root, logical)) for logical in logical_paths]
    sources.append(('@rendered-prompt', _rendered_prompt(root, 'emit', gear)))
    omitted = []
    if omit_registrations:
        data = _read_text_bytes(root, registration)
        omitted.append({
            'path': registration,
            'sha256': _sha256(data),
            'reason': 'verified generated registration omitted by --omit-registrations',
        })
    return sorted(sources, key=lambda item: item[0]), omitted


def _chunk_ranges(data):
    start = 0
    while start < len(data):
        end = min(start + CHUNK_BYTES, len(data))
        if end < len(data):
            newline = data.rfind(b'\n', start, end)
            if newline >= start:
                end = newline + 1
            else:
                while end > start and data[end] & 0xC0 == 0x80:
                    end -= 1
        if end <= start:
            raise PackError('cannot split UTF-8 input at byte {}'.format(start))
        yield start, end
        start = end


def write_bundle(out, sources, omitted):
    if os.path.lexists(str(out)):
        raise PackError('output path already exists: {}'.format(out))
    parent = out.parent
    if not parent.is_dir():
        raise PackError('output parent is not a directory: {}'.format(parent))

    temporary = None
    try:
        temporary = Path(tempfile.mkdtemp(prefix='.{}.'.format(out.name), dir=str(parent)))
        chunks_directory = temporary / 'chunks'
        chunks_directory.mkdir()
        entries = []
        number = 1
        for logical, data in sorted(sources, key=lambda item: item[0]):
            chunks = []
            for start, end in _chunk_ranges(data):
                name = '{:06d}.txt'.format(number)
                number += 1
                chunk_path = chunks_directory / name
                chunk = data[start:end]
                chunk_path.write_bytes(chunk)
                chunks.append({
                    'path': 'chunks/{}'.format(name),
                    'start_byte': start,
                    'end_byte': end,
                    'sha256': _sha256(chunk),
                })
            entries.append({
                'path': logical,
                'sha256': _sha256(data),
                'bytes': len(data),
                'chunks': chunks,
            })
        manifest = {'schema': 1, 'files': entries}
        if omitted:
            manifest['omitted'] = omitted
        manifest_text = json.dumps(
            manifest, sort_keys=True, ensure_ascii=False, indent=2) + '\n'
        (temporary / 'manifest.json').write_text(manifest_text, encoding='utf-8')
        os.replace(str(temporary), str(out))
        temporary = None
    except PackError:
        raise
    except OSError as exc:
        raise PackError('cannot write bundle {}: {}'.format(out, exc))
    finally:
        if temporary is not None:
            shutil.rmtree(str(temporary), ignore_errors=True)


def run_scaffold_check(root, gear):
    command = [
        sys.executable,
        str(root / '.claude/skills/generate-gear/scaffold_proof.py'),
        gear,
        '--check',
    ]
    try:
        result = subprocess.run(command, cwd=str(root), capture_output=True, text=True)
    except OSError as exc:
        raise ScaffoldCheckError('cannot run scaffold check: {}'.format(exc))
    if result.returncode != 0:
        detail = (result.stderr.strip() or result.stdout.strip() or 'no diagnostic output')
        raise ScaffoldCheckError(
            'scaffold_proof.py {} --check failed (exit {}): {}'.format(
                gear, result.returncode, detail))


def pack(root, gear, stage, out, omit_registrations=False, scaffold_check=None):
    if stage not in ('compile', 'emit'):
        raise PackError('stage must be compile or emit')
    if not render_prompt.GEAR_RE.match(gear):
        raise PackError('invalid gear name {!r}'.format(gear))
    if omit_registrations and stage != 'emit':
        raise PackError('--omit-registrations is valid only with --stage emit')

    omitted = []
    if stage == 'compile':
        sources = discover_compile_inputs(root, gear)
    else:
        if omit_registrations:
            (scaffold_check or run_scaffold_check)(root, gear)
        sources, omitted = discover_emit_inputs(root, gear, omit_registrations)
    write_bundle(out, sources, omitted)


def main(argv=None):
    parser = argparse.ArgumentParser(prog='pack_pipeline_inputs.py')
    parser.add_argument('gear')
    parser.add_argument('--stage', required=True, choices=('compile', 'emit'))
    parser.add_argument('--out', required=True)
    parser.add_argument('--omit-registrations', action='store_true')
    arguments = parser.parse_args(argv)
    try:
        pack(
            Path.cwd().resolve(),
            arguments.gear,
            arguments.stage,
            Path(os.path.abspath(arguments.out)),
            arguments.omit_registrations,
        )
    except ScaffoldCheckError as exc:
        print('pack_pipeline_inputs: {}'.format(exc), file=sys.stderr)
        return 1
    except PackError as exc:
        print('pack_pipeline_inputs: {}'.format(exc), file=sys.stderr)
        return 2
    return 0


if __name__ == '__main__':
    sys.exit(main())
