#!/usr/bin/env python3
"""Build and verify the compiler's shared proof harness API view."""
import argparse
import hashlib
import json
import os
import subprocess
import sys
from pathlib import Path


PACKAGES = ('proofkit', 'proofkit3d')
VERSION = 1


class ViewError(ValueError):
    pass


def digest(data):
    return hashlib.sha256(data).hexdigest()


def sources(root):
    paths = [root / 'proof/go.mod', root / 'proof/go.sum']
    for package in PACKAGES:
        directory = root / 'proof' / package
        package_files = sorted(directory.glob('*.go'))
        if not any(not path.name.endswith('_test.go') for path in package_files):
            raise ViewError('no harness implementation in {}'.format(directory))
        paths.extend(package_files)
    return [{'path': path.relative_to(root).as_posix(), 'sha256': digest(path.read_bytes())}
            for path in paths if path.is_file()]


def build(root):
    proof = root / 'proof'
    files = sources(root)
    env = os.environ.copy()
    env['GOCACHE'] = str(root / '.tmp/go-build')
    parts = ['# Checked proof harness API view\n\n',
             'Canonical source: proof/proofkit/ and proof/proofkit3d/. '
             'The complete proof gate uses those packages directly.\n\n']
    for package in PACKAGES:
        result = subprocess.run(['go', 'doc', '-all', './' + package], cwd=proof,
                                env=env, capture_output=True, text=True)
        if result.returncode or not result.stdout.strip():
            raise ViewError('go doc {} failed: {}'.format(
                package, result.stderr.strip() or 'empty API output'))
        parts.extend(['## {}\n\n'.format(package), '```text\n', result.stdout.rstrip(),
                      '\n```\n\n'])
    view = ''.join(parts).encode('utf-8')
    manifest = {'version': VERSION, 'files': files, 'view_sha256': digest(view)}
    return view, (json.dumps(manifest, indent=2, sort_keys=True) + '\n').encode('utf-8')


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', choices=('build', 'verify'))
    parser.add_argument('--root', type=Path, default=Path.cwd())
    args = parser.parse_args(argv)
    root = args.root.resolve()
    view_path = root / '.tmp/harness-api-view.md'
    manifest_path = root / '.tmp/harness-api-view.json'
    try:
        view, manifest = build(root)
        if args.action == 'build':
            view_path.parent.mkdir(exist_ok=True)
            if not view_path.exists() or view_path.read_bytes() != view:
                view_path.write_bytes(view)
            if not manifest_path.exists() or manifest_path.read_bytes() != manifest:
                manifest_path.write_bytes(manifest)
        elif view_path.read_bytes() != view or manifest_path.read_bytes() != manifest:
            raise ViewError('harness view or canonical source changed; rebuild before compilation')
    except (OSError, UnicodeError, ViewError) as exc:
        print('harness view: {}'.format(exc), file=sys.stderr)
        return 2
    print('{}: {} bytes, {} source files'.format(
        view_path, len(view), len(json.loads(manifest)['files'])))
    return 0


if __name__ == '__main__':
    sys.exit(main())
