#!/usr/bin/env python3
"""Gate every method call in a generated gear against the real Fusion API index.

Why this exists: a generated file can call a Fusion method that does not exist and pass every
other gate. That is not hypothetical. The second task-list pilot passed parse, pyright,
check_input_read, check_contract AND check_task_calls while calling `addPointOnObject` five
times (the real name is `addCoincident`) and `setExtentDefinition`/`setExtentDirection` on all
three extrudes (the real method is `setOneSideExtent`). Both are immediate runtime
`AttributeError`s.

Nothing else catches them:
  - pyright never mentioned `addPointOnObject`. The intellisense stubs type
    `sketch.geometricConstraints` loosely enough that any method name type-checks.
  - pyright DID flag the extrude pair, but only as REVIEW, and the workflow tells authors to
    ignore REVIEW as stub pessimism — correct advice in general, which is exactly why a real
    bug hiding there goes unseen.
  - check_task_calls.py only asks whether a call the task list NAMES appears somewhere in the
    file. A substitution passes it: `addCoincident` does appear, in a different task.

This check automates [PB-API-LOOKUP] — "grep the index before you write the name" — instead of
trusting that the author did.

What it does: collect every `<something>.name(...)` call in the file, then require each name to
exist in the Fusion API index, to be defined in the file itself, to come from the framework
helper modules, or to be a plain Python method. Anything left over is a name that exists nowhere
and will fail the moment the add-in runs.

Usage:
    python3 check_api_calls.py <generated.py> [--framework <dir>] [--index <path>]

Exit 0 = OK, 1 = BLOCKING, 2 = the index could not be built.

The index is the one built by build_fusion_index.py, which clones the Fusion API reference on
demand. This script builds it the same way if it is missing, so a fresh machine needs no setup.
"""
import argparse
import ast
import json
import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
DEFAULT_INDEX = os.path.expanduser(
    '~/.cache/fusion360-gear-generator/fusion-api-index.jsonl')

# Plain Python: builtins, str/list/dict methods, and the math functions the gear math uses.
# None of these are Fusion names, so the index rightly does not carry them.
PYTHON_METHODS = {
    'append', 'extend', 'insert', 'pop', 'remove', 'sort', 'reverse', 'count', 'index',
    'format', 'join', 'split', 'strip', 'lstrip', 'rstrip', 'replace', 'startswith',
    'endswith', 'lower', 'upper', 'items', 'keys', 'values', 'update', 'copy', 'setdefault',
    'acos', 'asin', 'atan', 'atan2', 'cos', 'sin', 'tan', 'sqrt', 'hypot', 'radians',
    'degrees', 'floor', 'ceil', 'fabs', 'pow', 'log', 'exp', 'isclose',
}

# Real Fusion methods the intellisense stubs omit, so the derived index lacks them too.
# Keep this list SHORT and justify every entry — it is the gate's blind spot.
INDEX_GAPS = {
    # Names the shipped add-in calls successfully inside Fusion, which the
    # intellisense stubs omit, so the derived index lacks them too. Each is used by
    # lib/geargen/spurgear.py in an add-in that runs, which is the evidence. Keep
    # this list SHORT and justify every entry — it is the gate's blind spot.
    'project',                    # Sketch.project(entity) — spurgear.py:153
    'createInput2',               # SketchTexts.createInput2(text, height) — spurgear.py:187
    'addConstantRadiusEdgeSet',   # on FilletFeatureInput itself — spurgear.py:683
}


def load_index(path):
    if not os.path.exists(path):
        builder = os.path.join(HERE, 'build_fusion_index.py')
        print('api index missing; building it with %s' % builder, file=sys.stderr)
        rc = subprocess.call([sys.executable, builder])
        if rc != 0 or not os.path.exists(path):
            return None
    names = set()
    with open(path) as fh:
        for line in fh:
            try:
                names.add(json.loads(line)['name'])
            except Exception:
                continue
    return names


def defined_names(paths):
    """Every function/class/method name the given Python files define."""
    names = set()
    for path in paths:
        if not os.path.exists(path):
            continue
        try:
            tree = ast.parse(open(path).read())
        except SyntaxError:
            continue
        for node in ast.walk(tree):
            if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef, ast.ClassDef)):
                names.add(node.name)
    return names


def framework_files(root):
    out = []
    for base, _, files in os.walk(root):
        for name in files:
            if name.endswith('.py'):
                out.append(os.path.join(base, name))
    return out


def main():
    ap = argparse.ArgumentParser(add_help=True)
    ap.add_argument('target')
    ap.add_argument('--framework', default='lib',
                    help='directory holding the framework modules (default: lib)')
    ap.add_argument('--index', default=DEFAULT_INDEX)
    args = ap.parse_args()

    index = load_index(args.index)
    if index is None:
        print('check_api_calls: could not build the Fusion API index', file=sys.stderr)
        return 2

    src = open(args.target).read()
    tree = ast.parse(src)

    known = set(index) | PYTHON_METHODS | INDEX_GAPS
    known |= defined_names([args.target])
    known |= defined_names(framework_files(args.framework))
    # Names bound by imports in the target (framework helpers called bare).
    for node in ast.walk(tree):
        if isinstance(node, (ast.Import, ast.ImportFrom)):
            for alias in node.names:
                known.add(alias.asname or alias.name.split('.')[-1])

    unknown = {}
    for node in ast.walk(tree):
        if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute):
            name = node.func.attr
            if name.startswith('_') or name in known:
                continue
            unknown.setdefault(name, node.lineno)

    if unknown:
        print('api-call check: BLOCKING (%d)' % len(unknown))
        for name, lineno in sorted(unknown.items(), key=lambda kv: kv[1]):
            print("  %s:%d calls '%s(' — no such name in the Fusion API index, the framework, "
                  "or this file" % (args.target, lineno, name))
        print('  Grep the index for the name you meant: '
              'grep \'"class":"<OwningClass>"\' %s' % args.index)
        return 1

    print('api-call check: OK (every method call resolves)')
    return 0


if __name__ == '__main__':
    sys.exit(main())
