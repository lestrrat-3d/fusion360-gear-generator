#!/usr/bin/env python3
"""Build and verify a compact API view of gear modules named by checked steps."""
import argparse
import ast
import hashlib
import json
import re
import sys
from pathlib import Path


GEAR = re.compile(r'[a-z][a-z0-9_]*\Z')
IDENTIFIER = re.compile(r'\b[A-Za-z_][A-Za-z_0-9]*\b')
FRAMEWORK = {'__init__', 'base', 'misc', 'utilities', 'solids', 'spurproxy'}
VERSION = 1


class ViewError(ValueError):
    pass


def digest(data):
    return hashlib.sha256(data).hexdigest()


def method_line(node):
    decorators = ['    @{}'.format(ast.unparse(item)) for item in node.decorator_list]
    prefix = 'async def' if isinstance(node, ast.AsyncFunctionDef) else 'def'
    returns = ' -> {}'.format(ast.unparse(node.returns)) if node.returns else ''
    return decorators + ['    {} {}({}){}: ...'.format(
        prefix, node.name, ast.unparse(node.args), returns)]


def declaration_lines(node):
    if isinstance(node, ast.ClassDef):
        bases = ', '.join(ast.unparse(base) for base in node.bases)
        heading = 'class {}({}):'.format(node.name, bases) if bases else 'class {}:'.format(node.name)
        lines = [heading]
        for member in node.body:
            if isinstance(member, (ast.FunctionDef, ast.AsyncFunctionDef)):
                lines.extend(method_line(member))
        if len(lines) == 1:
            lines.append('    ...')
        return lines
    if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)):
        return [line.removeprefix('    ') for line in method_line(node)]
    return []


def declared_names(node):
    if isinstance(node, (ast.ClassDef, ast.FunctionDef, ast.AsyncFunctionDef)):
        return [node.name]
    if isinstance(node, ast.Assign):
        return [target.id for target in node.targets if isinstance(target, ast.Name)]
    if isinstance(node, ast.AnnAssign) and isinstance(node.target, ast.Name):
        return [node.target.id]
    return []


def build(root, gear):
    if not GEAR.fullmatch(gear):
        raise ViewError('invalid gear name')
    step_path = root / 'spec' / gear / 'steps.md'
    step_bytes = step_path.read_bytes()
    wanted = set(IDENTIFIER.findall(step_bytes.decode('utf-8')))
    files = []
    sections = ['# Checked gear dependency API view\n',
                'Canonical source: spec/{}/steps.md and named lib/geargen modules.\n'.format(gear),
                'This view contains signatures and decorators, not method bodies.\n']
    module_dir = root / 'lib' / 'geargen'
    for path in sorted(module_dir.glob('*.py')):
        if path.stem in FRAMEWORK or path.stem == gear:
            continue
        raw = path.read_bytes()
        tree = ast.parse(raw, filename=str(path))
        selected = []
        for node in tree.body:
            names = declared_names(node)
            if not set(names) & wanted:
                continue
            if isinstance(node, (ast.ClassDef, ast.FunctionDef, ast.AsyncFunctionDef)):
                selected.extend(declaration_lines(node))
            elif isinstance(node, (ast.Assign, ast.AnnAssign)):
                value = node.value
                try:
                    literal = ast.literal_eval(value)
                except (ValueError, TypeError, SyntaxError):
                    continue
                if isinstance(literal, (str, int, float, bool)):
                    selected.extend('{} = {!r}'.format(name, literal)
                                    for name in names if name in wanted)
        if not selected:
            continue
        relative = path.relative_to(root).as_posix()
        files.append({'path': relative, 'sha256': digest(raw)})
        sections.extend(['\n## {}\n\n'.format(relative), '```python\n',
                         '\n'.join(selected), '\n```\n'])
    view = ''.join(sections).encode('utf-8')
    manifest = {'version': VERSION, 'gear': gear,
                'steps': {'path': step_path.relative_to(root).as_posix(), 'sha256': digest(step_bytes)},
                'files': files, 'view_sha256': digest(view)}
    return view, (json.dumps(manifest, indent=2, sort_keys=True) + '\n').encode('utf-8')


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', choices=('build', 'verify'))
    parser.add_argument('gear')
    parser.add_argument('--root', type=Path, default=Path.cwd())
    args = parser.parse_args(argv)
    root = args.root.resolve()
    view_path = root / '.tmp/{}.dependency-view.md'.format(args.gear)
    manifest_path = root / '.tmp/{}.dependency-view.json'.format(args.gear)
    try:
        view, manifest = build(root, args.gear)
        if args.action == 'build':
            view_path.parent.mkdir(exist_ok=True)
            view_path.write_bytes(view)
            manifest_path.write_bytes(manifest)
        elif view_path.read_bytes() != view or manifest_path.read_bytes() != manifest:
            raise ViewError('dependency view or input digests changed; rebuild before emission')
    except (OSError, UnicodeError, SyntaxError, ViewError) as exc:
        print('dependency view: {}'.format(exc), file=sys.stderr)
        return 2
    print('{}: {} bytes, {} dependency files'.format(
        view_path, len(view), len(json.loads(manifest)['files'])))
    return 0


if __name__ == '__main__':
    sys.exit(main())
