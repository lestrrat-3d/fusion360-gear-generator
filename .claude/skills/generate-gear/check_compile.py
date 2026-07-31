#!/usr/bin/env python3
"""Gate the compile stage: does the step list agree with its spec and its proof?

Compile turns `spec/<gear>/instructions.md` (plus `fusion.md`) into two artifacts, a step list at
`spec/<gear>/steps.md` and a runnable proof at `proof/<gear>/`. Neither is authoritative over the
other; this checks that they describe the same build and that both still match the prose they came
from.

Four checks gate, and one is reported:

  1. CITATIONS RESOLVE. Every `**From:**` line names real files and line ranges that exist.
  2. STEPS AND PROOF AGREE. Every `[GO]` step names the proof function that realises it, and every
     proof function is claimed by a step. Drift in either direction means one artifact moved
     without the other.
  3. API CALLS ARE REAL. Every Fusion call the step list names exists in the API index, on the
     class it is used from. Catches a spec that names a method Fusion does not have.
  4. INPUTS HAVE NOT DRIFTED. The provenance table's hashes still match the live spec files, so an
     edited spec cannot leave a stale step list looking healthy.

  COVERAGE is printed, never gated. The spec lines no step claims are worth skimming for
  omissions, but most of that list is headings and introductions, and the compiler is reporting on
  its own citations, so a lazily wide one silences it. Deciding which lines ought to count is a
  judgement call, and a gate would force it on every build.

Usage:
    python3 check_compile.py <gear>            # e.g. spurgear

Run from the repo root. Exit 0 = OK, 1 = BLOCKING, 2 = something is missing.
"""
import json
import os
import re
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
INDEX = os.path.expanduser('~/.cache/fusion360-gear-generator/fusion-api-index.jsonl')
MIN_CALL_LEN = 6


def read(path):
    with open(path) as fh:
        return fh.read()


def blob_hash(path):
    return subprocess.run(['git', 'hash-object', path],
                          capture_output=True, text=True).stdout.strip()


def steps_of(src):
    """Split the step list into (id, tag, body) triples, in file order."""
    out = []
    heads = list(re.finditer(r'^##\s+(\S+)\s+`\[(GO|PROSE)\]`\s+(.*)$', src, re.M))
    for i, m in enumerate(heads):
        end = heads[i + 1].start() if i + 1 < len(heads) else len(src)
        # The title counts as part of the step. A step may name its proof function
        # there rather than in the prose below, and both readings are reasonable.
        out.append((m.group(1), m.group(2), m.group(3) + '\n' + src[m.end():end]))
    return out


def citations(body):
    """Every (file, first, last) a step's From line names."""
    out = []
    for m in re.finditer(r'`([\w./-]+\.(?:md|go)):(\d+)(?:-(\d+))?`', body):
        first = int(m.group(2))
        out.append((m.group(1), first, int(m.group(3) or first)))
    return out


# Framework modules the generated code calls into. The per-gear implementations are
# deliberately excluded: letting them widen the allowlist would make an existing
# implementation authoritative, which is what this pipeline exists to avoid.
FRAMEWORK = [
    'lib/geargen/base.py', 'lib/geargen/misc.py', 'lib/geargen/utilities.py',
    'lib/geargen/solids.py', 'lib/geargen/spurproxy.py', 'lib/fusion360utils',
]

# Plain Python and math, which the index rightly does not carry.
PYTHON_METHODS = {
    'append', 'extend', 'insert', 'pop', 'remove', 'sort', 'reverse', 'count', 'index',
    'format', 'join', 'split', 'strip', 'replace', 'startswith', 'endswith', 'lower',
    'upper', 'items', 'keys', 'values', 'update', 'copy', 'setdefault', 'acos', 'asin',
    'atan', 'atan2', 'cos', 'sin', 'tan', 'sqrt', 'hypot', 'radians', 'degrees', 'floor',
    'ceil', 'fabs', 'isclose',
}

INDEX_GAPS = {
    # Names the shipped add-in calls successfully inside Fusion, which the
    # intellisense stubs omit, so the derived index lacks them too. Keep this list
    # SHORT and justify every entry — it is the gate's blind spot.
    'project',                    # Sketch.project(entity)
    'createInput2',               # SketchTexts.createInput2(text, height); the stubs carry only createInput3
    'addConstantRadiusEdgeSet',   # FilletFeatureInput.addConstantRadiusEdgeSet; the stubs put it only on FilletEdgeSetInputs
}


def defined_names(paths):
    """Every function, class and method name the given Python sources define."""
    import ast
    names = set()
    files = []
    for path in paths:
        if os.path.isdir(path):
            for base, _, entries in os.walk(path):
                files += [os.path.join(base, e) for e in entries if e.endswith('.py')]
        elif os.path.exists(path):
            files.append(path)
    for path in files:
        try:
            tree = ast.parse(read(path))
        except SyntaxError:
            continue
        for node in ast.walk(tree):
            if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef, ast.ClassDef)):
                names.add(node.name)
    return names


def contract_names(gear):
    """The classes and methods the gear's own contract manifest declares."""
    path = os.path.join('spec', gear, 'contract.json')
    if not os.path.exists(path):
        return set()
    names = set()
    try:
        data = json.loads(read(path))
    except ValueError:
        return names
    for cls, spec in (data.get('classes') or {}).items():
        names.add(cls)
        names.update(spec.get('methods') or [])
    return names


def named_calls(src):
    body = re.sub(r'```.*?```', '', src, flags=re.S)
    names = set()
    for span in re.findall(r'`([^`\n]+)`', body):
        for name in re.findall(r'\b([a-z][A-Za-z0-9_]{%d,})\s*\(' % (MIN_CALL_LEN - 1), span):
            names.add(name)
    for line in re.findall(r'<!--\s*check-compile:\s*ignore\s+([^>]*?)-->', src):
        names -= set(line.split())
    return names


def proof_functions(proof_dir):
    """Every step function the proof defines, by name."""
    found = set()
    if not os.path.isdir(proof_dir):
        return found
    for entry in sorted(os.listdir(proof_dir)):
        if not entry.endswith('.go'):
            continue
        for m in re.finditer(r'^func (step\w+)\(', read(os.path.join(proof_dir, entry)), re.M):
            found.add(m.group(1))
    return found


def main(argv):
    if len(argv) != 2:
        print('usage: check_compile.py <gear>', file=sys.stderr)
        return 2
    gear = argv[1]
    steps_path = os.path.join('spec', gear, 'steps.md')
    proof_dir = os.path.join('proof', gear)
    if not os.path.exists(steps_path):
        print('check_compile: no step list at %s' % steps_path, file=sys.stderr)
        return 2
    src = read(steps_path)
    steps = steps_of(src)
    if not steps:
        print('check_compile: %s declares no steps' % steps_path, file=sys.stderr)
        return 2

    problems = []

    # 1. citations resolve
    cited = {}
    for sid, _, body in steps:
        for path, first, last in citations(body):
            if not path.endswith('.md'):
                continue
            full = path if os.path.exists(path) else os.path.join('spec', gear, path)
            if not os.path.exists(full):
                problems.append("  %s cites %s, which does not exist" % (sid, path))
                continue
            total = len(read(full).splitlines())
            if last > total:
                problems.append("  %s cites %s:%d-%d, but that file has %d lines"
                                % (sid, path, first, last, total))
                continue
            cited.setdefault(full, set()).update(range(first, last + 1))

    # 2. steps and proof agree
    functions = proof_functions(proof_dir)
    claimed = set()
    for sid, tag, body in steps:
        named = set(re.findall(r'\b(step[A-Z]\w*)\b', body))
        if tag == 'GO' and not named:
            problems.append("  %s is tagged [GO] but names no proof function" % sid)
        for fn in named:
            if fn not in functions:
                problems.append("  %s names proof function %s, which %s/ does not define"
                                % (sid, fn, proof_dir))
            claimed.add(fn)
    for fn in sorted(functions - claimed):
        problems.append("  proof function %s is not claimed by any step" % fn)

    # 3. API calls are real
    if os.path.exists(INDEX):
        known = set()
        with open(INDEX) as fh:
            for line in fh:
                try:
                    known.add(json.loads(line)['name'])
                except Exception:
                    continue
        known |= PYTHON_METHODS | INDEX_GAPS
        known |= defined_names(FRAMEWORK) | contract_names(gear)
        for call in sorted(named_calls(src)):
            if call not in known:
                problems.append("  the step list names '%s(', which is not in the Fusion API index"
                                % call)
    else:
        print('check_compile: no API index at %s; skipping the call check' % INDEX,
              file=sys.stderr)

    # 4. inputs have not drifted
    stamped = dict(re.findall(r'\|\s*`([\w./-]+)`\s*\|\s*`([0-9a-f]{40})`\s*\|', src))
    if not stamped:
        problems.append("  the step list carries no provenance table, so staleness cannot be seen")
    for path, want in sorted(stamped.items()):
        if not os.path.exists(path):
            problems.append("  provenance names %s, which does not exist" % path)
            continue
        got = blob_hash(path)
        if got != want:
            problems.append("  %s has changed since the step list was compiled (%s now, %s then)"
                            % (path, got[:12], want[:12]))

    # coverage, reported only
    for path, lines in sorted(cited.items()):
        text = read(path).splitlines()
        live = [i for i, l in enumerate(text, 1) if l.strip()]
        missing = [i for i in live if i not in lines]
        print("coverage: %s — %d/%d lines claimed by a step, %d unclaimed"
              % (path, len(live) - len(missing), len(live), len(missing)))

    if problems:
        print('compile check: BLOCKING (%d)' % len(problems))
        for p in problems:
            print(p)
        return 1
    print('compile check: OK (%d steps, %d proof functions, %d spec files stamped)'
          % (len(steps), len(functions), len(stamped)))
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv))
