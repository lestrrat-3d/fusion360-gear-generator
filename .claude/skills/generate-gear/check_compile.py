#!/usr/bin/env python3
"""Gate the compile stage: does the step list agree with its spec and its proof?

Compile turns `spec/<gear>/instructions.md` (plus `fusion.md`) into two artifacts, a step list at
`spec/<gear>/steps.md` and a runnable proof at `proof/<gear>/`. Neither is authoritative over the
other; this checks that they describe the same build and that both still match the prose they came
from.

Four checks gate, and one is reported:

  1. CITATIONS RESOLVE. Every step has a nonempty `**From:**` line naming real files and line
     ranges that exist.
  2. STEPS AND PROOF AGREE. Every `[GO]` step names the proof function that realises it, and every
     proof function is claimed by a step. Drift in either direction means one artifact moved
     without the other.
  3. API CALLS ARE REAL. Every Fusion call the step list names exists in the API database the
     `fusion` plugin ships. Catches a spec that names a method Fusion does not have.
  4. INPUTS HAVE NOT DRIFTED. The provenance table contains and matches the existing instructions,
     optional fusion sidecar, playbook, and auxiliary documents referenced by the specs,
     so an edited source cannot leave a stale step list looking healthy.

  COVERAGE is printed, never gated. The spec lines no step claims are worth skimming for
  omissions, but most of that list is headings and introductions, and the compiler is reporting on
  its own citations, so a lazily wide one silences it. Deciding which lines ought to count is a
  judgement call, and a gate would force it on every build.

  UNVERIFIED CALLS are printed, never gated. These are the calls in fusion_api.UNVERIFIED_CALLS —
  ones the shipped add-in and this step list make that the API database does not back. They are
  not waived: every run names each one and says which class, if any, does declare it. They do not
  fail the run either, because only a Fusion session can settle whether the call works.

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
sys.path.insert(0, HERE)
import fusion_api  # noqa: E402  (sibling module; sys.path is fixed up just above)
from call_parser import call_shapes  # noqa: E402

PATH_REF = r'[\w./-]+\.(?:md|go|py|json|sh)'
PATH_TOKEN = re.compile(r'`(%s)`' % PATH_REF)
DOCUMENT_REF = re.compile(r'(?<![\w./-])[\w./-]+\.md\b')
INLINE_CITATION = re.compile(r'`(%s):(\d+)(?:\s*[-\u2013]\s*(\d+))?`' % PATH_REF)
LINE_RANGE = re.compile(r'\bL(\d+)(?:\s*[-\u2013]\s*(\d+))?\b')


def referenced_documents(path):
    """Return existing Markdown documents referenced from one gear's input spec."""
    found = set()
    for reference in DOCUMENT_REF.findall(read(path)):
        candidates = (
            os.path.normpath(os.path.join(os.path.dirname(path), reference)),
            os.path.normpath(reference),
        )
        for candidate in candidates:
            if not os.path.isfile(candidate):
                continue
            if candidate != path:
                found.add(candidate)
            break
    return found


def provenance_inputs(gear):
    """Existing source files whose hashes define a compiled step list."""
    instructions = os.path.join('spec', gear, 'instructions.md')
    fusion = os.path.join('spec', gear, 'fusion.md')
    playbook = os.path.join('.claude', 'skills', 'generate-gear', 'PLAYBOOK.md')
    specs = [path for path in (instructions, fusion) if os.path.isfile(path)]
    inputs = {
        path for path in (instructions, fusion, playbook) if os.path.isfile(path)
    }
    for path in specs:
        inputs.update(referenced_documents(path))
    return inputs


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


def from_block(body):
    """The raw text after a step's From marker, if any."""
    match = re.search(r'^\*\*From:\*\*(.*?)(?=\n\s*\n|\Z)', body, re.M | re.S)
    if not match:
        return None
    return match.group(1)


def citations_from_block(block):
    """Every (file, first, last) named by a From block.

    The committed step lists write citations as a backtick path followed by one or
    more `L<line>` or `L<first>–<last>` ranges. Older inline
    `` `path:line[-last]` `` citations remain parseable so fixtures and drafts
    fail on validation, not on syntax churn.
    """
    out = []
    for m in INLINE_CITATION.finditer(block):
        first = int(m.group(2))
        out.append((m.group(1), first, int(m.group(3) or first)))

    paths = list(PATH_TOKEN.finditer(block))
    for i, path_match in enumerate(paths):
        end = paths[i + 1].start() if i + 1 < len(paths) else len(block)
        for line_match in LINE_RANGE.finditer(block, path_match.end(), end):
            first = int(line_match.group(1))
            out.append((path_match.group(1), first, int(line_match.group(2) or first)))
    return out


def resolve_citation_path(path, gear):
    """Resolve a cited path from the repo root, with bare spec filenames allowed."""
    if os.path.exists(path):
        return path
    spec_path = os.path.join('spec', gear, path)
    if os.path.exists(spec_path):
        return spec_path
    return None


def citation_label(path, first, last):
    """A compact, actionable citation label for diagnostics."""
    if first == last:
        return '%s L%d' % (path, first)
    return '%s L%d–%d' % (path, first, last)


def validate_citations(body, gear):
    """Return (valid citations, problems) for one step body."""
    block = from_block(body)
    if block is None or not block.strip():
        return [], ["has no nonempty **From:** citation"]

    parsed = citations_from_block(block)
    if not parsed:
        return [], ["has no parseable **From:** file-and-line citation; expected `path` L1 or "
                    "`path` L1–2-style ranges"]

    valid = []
    problems = []
    for path, first, last in parsed:
        label = citation_label(path, first, last)
        if first < 1 or last < 1:
            problems.append("cites %s, but line numbers start at 1" % label)
            continue
        if first > last:
            problems.append("cites %s, but the first line is after the last line" % label)
            continue
        full = resolve_citation_path(path, gear)
        if full is None:
            problems.append("cites %s, which does not exist" % path)
            continue
        total = len(read(full).splitlines())
        if last > total:
            problems.append("cites %s, but that file has %d lines" % (label, total))
            continue
        valid.append((full, first, last))
    return valid, problems


def citations(body, gear):
    """Every valid (file, first, last) named by a step's From block."""
    valid, _ = validate_citations(body, gear)
    return valid


# Framework modules the generated code calls into. The per-gear implementations are
# deliberately excluded: letting them widen the allowlist would make an existing
# implementation authoritative, which is what this pipeline exists to avoid.
FRAMEWORK = [
    'lib/geargen/base.py', 'lib/geargen/misc.py', 'lib/geargen/utilities.py',
    'lib/geargen/solids.py', 'lib/geargen/spurproxy.py', 'lib/fusion360utils',
]

# Plain Python and math, which the API database rightly does not carry.
PYTHON_METHODS = {
    'append', 'extend', 'insert', 'pop', 'remove', 'sort', 'reverse', 'count', 'index',
    'format', 'join', 'split', 'strip', 'replace', 'startswith', 'endswith', 'lower',
    'upper', 'items', 'keys', 'values', 'update', 'copy', 'setdefault', 'acos', 'asin',
    'atan', 'atan2', 'cos', 'sin', 'tan', 'sqrt', 'hypot', 'radians', 'degrees', 'floor',
    'ceil', 'fabs', 'isclose',
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
        names.update(name for name, _ in call_shapes(span))
    for line in re.findall(r'<!--\s*check-compile:\s*ignore\s+([^>]*?)-->', src):
        names -= set(line.split())
    return names


def watched_calls(src, path):
    """Where the step list names a call from the unverified watchlist, receiver and all.

    An entry that names receivers only counts a call written on one of them, so the legitimate
    `chamferFeatures.createInput2()` is not dragged in beside `sketchTexts.createInput2(...)`.
    """
    seen = {}
    for name, _, receivers, _ in fusion_api.UNVERIFIED_CALLS:
        if receivers is None:
            pattern = r'\b%s\s*\(' % re.escape(name)
        else:
            pattern = r'\b(?:%s)\s*\.\s*%s\s*\(' % (
                '|'.join(re.escape(r) for r in receivers), re.escape(name))
        lines = sorted({src[:m.start()].count('\n') + 1 for m in re.finditer(pattern, src)})
        if lines:
            seen[name] = '%s:%s' % (path, ','.join(str(line) for line in lines))
    return seen


def strip_go_comments_and_literals(src):
    """Blank Go comments and literals so proof registration is read from code only."""
    out = list(src)
    i = 0
    while i < len(src):
        if src.startswith('//', i):
            j = src.find('\n', i)
            if j == -1:
                j = len(src)
            for k in range(i, j):
                out[k] = ' '
            i = j
            continue
        if src.startswith('/*', i):
            j = src.find('*/', i + 2)
            j = len(src) if j == -1 else j + 2
            for k in range(i, j):
                if out[k] != '\n':
                    out[k] = ' '
            i = j
            continue
        if src[i] in ('"', "'"):
            quote = src[i]
            j = i + 1
            while j < len(src):
                if src[j] == '\\':
                    j += 2
                    continue
                if src[j] == quote:
                    j += 1
                    break
                j += 1
            for k in range(i, min(j, len(src))):
                if out[k] != '\n':
                    out[k] = ' '
            i = j
            continue
        if src[i] == '`':
            j = src.find('`', i + 1)
            j = len(src) if j == -1 else j + 1
            for k in range(i, j):
                if out[k] != '\n':
                    out[k] = ' '
            i = j
            continue
        i += 1
    return ''.join(out)


def matching_delimiter(src, start):
    """Return the matching delimiter index for src[start], or None."""
    pairs = {'(': ')', '{': '}', '[': ']'}
    opens = set(pairs)
    closes = {v: k for k, v in pairs.items()}
    stack = [src[start]]
    for i in range(start + 1, len(src)):
        ch = src[i]
        if ch in opens:
            stack.append(ch)
        elif ch in closes:
            if not stack or stack[-1] != closes[ch]:
                return None
            stack.pop()
            if not stack:
                return i
    return None


def go_func_bodies(src, name_pattern):
    """Yield (name, body) for top-level Go functions whose names match name_pattern."""
    pattern = r'(?m)^func\s+(%s)\s*\(' % name_pattern
    for m in re.finditer(pattern, src):
        open_brace = src.find('{', m.end())
        if open_brace == -1:
            continue
        close_brace = matching_delimiter(src, open_brace)
        if close_brace is None:
            continue
        yield m.group(1), src[open_brace + 1:close_brace]


def split_go_args(src):
    """Split a Go argument list on top-level commas."""
    args = []
    start = 0
    stack = []
    pairs = {'(': ')', '{': '}', '[': ']'}
    closes = {v: k for k, v in pairs.items()}
    for i, ch in enumerate(src):
        if ch in pairs:
            stack.append(ch)
        elif ch in closes:
            if stack and stack[-1] == closes[ch]:
                stack.pop()
        elif ch == ',' and not stack:
            args.append(src[start:i].strip())
            start = i + 1
    tail = src[start:].strip()
    if tail:
        args.append(tail)
    return args


PROOF_RUN_CALLS = [
    (r'\bproofkit\s*\.\s*Run\s*\(', 2),
    (r'\bproofkit3d\s*\.\s*Run\s*\(', 2),
    (r'\bproofkit3d\s*\.\s*RunSolid\s*\(', 2),
    (r'\bproofkit3d\s*\.\s*RunWithGate\s*\(', 2),
]


def go_brace_pairs(src):
    """Return matching brace positions in already-scrubbed Go source."""
    pairs = {}
    stack = []
    for pos, char in enumerate(src):
        if char == '{':
            stack.append(pos)
        elif char == '}' and stack:
            opening = stack.pop()
            pairs[opening] = pos
    return pairs


def go_brace_depth(src, pos):
    """Return brace nesting depth before pos in already-scrubbed Go source."""
    return sum(char == '{' for char in src[:pos]) - sum(char == '}' for char in src[:pos])


def go_block_condition(src, opening):
    """Return the known reachability of an if block, or None when it is unknown."""
    line_start = src.rfind('\n', 0, opening) + 1
    header = src[line_start:opening].strip()
    match = re.match(r'if\s+(.+)$', header)
    if not match:
        return False
    condition = re.sub(r'\s+', ' ', match.group(1)).strip()
    if condition == 'false':
        return False
    if condition in ('true', 't != nil'):
        return True
    return None


def go_top_level_return_before(src, pos):
    """Return whether an unconditional return precedes pos in a test body."""
    for match in re.finditer(r'\breturn\b', src[:pos]):
        if go_brace_depth(src, match.start()) == 0:
            return True
    return False


def go_early_return_before(src, pos, brace_pairs):
    """Return whether a known-true top-level if-return precedes pos."""
    for opening, closing in brace_pairs.items():
        if closing >= pos or go_brace_depth(src, opening) != 0:
            continue
        if go_block_condition(src, opening) is not True:
            continue
        if re.fullmatch(r'\s*return\s*;?\s*', src[opening + 1:closing]):
            return True
    return False


def go_call_is_reachable(src, pos, brace_pairs):
    """Return whether a proof run at pos can execute on the test path."""
    if go_top_level_return_before(src, pos) or go_early_return_before(src, pos, brace_pairs):
        return False
    for opening, closing in brace_pairs.items():
        if not opening < pos < closing:
            continue
        condition = go_block_condition(src, opening)
        if condition is not True:
            return False
    return True


def registered_step_functions(src):
    """Step functions passed by reachable proofkit runs inside Go Test functions."""
    registered = set()
    for _, body in go_func_bodies(src, r'Test[A-Z]\w*'):
        for pattern, build_arg in PROOF_RUN_CALLS:
            for m in re.finditer(pattern, body):
                if not go_call_is_reachable(body, m.start(), go_brace_pairs(body)):
                    continue
                open_paren = m.end() - 1
                close_paren = matching_delimiter(body, open_paren)
                if close_paren is None:
                    continue
                args = split_go_args(body[open_paren + 1:close_paren])
                if len(args) > build_arg and re.fullmatch(r'step[A-Z]\w*', args[build_arg]):
                    registered.add(args[build_arg])
    return registered


def proof_functions(proof_dir):
    """Every step function the proof defines, by name."""
    found = set()
    if not os.path.isdir(proof_dir):
        return found
    for entry in sorted(os.listdir(proof_dir)):
        if not entry.endswith('.go'):
            continue
        src = strip_go_comments_and_literals(read(os.path.join(proof_dir, entry)))
        # step<Title>, so a helper called steps() is not mistaken for a step.
        for name, _ in go_func_bodies(src, r'step[A-Z]\w*'):
            found.add(name)
    return found


def proof_registrations(proof_dir):
    """Every step function directly registered in a proofkit run from a Go Test."""
    found = set()
    if not os.path.isdir(proof_dir):
        return found
    for entry in sorted(os.listdir(proof_dir)):
        if not entry.endswith('_test.go'):
            continue
        src = strip_go_comments_and_literals(read(os.path.join(proof_dir, entry)))
        found.update(registered_step_functions(src))
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
        valid, citation_problems = validate_citations(body, gear)
        for problem in citation_problems:
            problems.append("  %s %s" % (sid, problem))
        for path, first, last in valid:
            if not path.endswith('.md'):
                continue
            cited.setdefault(path, set()).update(range(first, last + 1))

    # 2. steps and proof agree
    functions = proof_functions(proof_dir)
    registered = proof_registrations(proof_dir)
    claimed = set()
    for sid, tag, body in steps:
        named = set(re.findall(r'\b(step[A-Z]\w*)\b', body))
        if tag == 'GO' and not named:
            problems.append("  %s is tagged [GO] but names no proof function" % sid)
        for fn in named:
            if fn not in functions:
                problems.append("  %s names proof function %s, which %s/ does not define"
                                % (sid, fn, proof_dir))
            elif fn not in registered:
                problems.append("  %s names proof function %s, but no Go Test registers it "
                                "in a proofkit run" % (sid, fn))
            claimed.add(fn)
    for fn in sorted(functions - claimed):
        problems.append("  proof function %s is not claimed by any step" % fn)
    for fn in sorted(functions - registered):
        problems.append("  proof function %s is defined but is not registered in any "
                        "proofkit run inside a Go Test" % fn)

    # 3. API calls are real
    local = PYTHON_METHODS | defined_names(FRAMEWORK) | contract_names(gear)
    watched = {name for name, _, _, _ in fusion_api.UNVERIFIED_CALLS}
    candidates = sorted(name for name in named_calls(src) if name not in local)
    try:
        hits = fusion_api.lookup_many(n for n in candidates if n not in watched)
        findings = fusion_api.unverified_findings(watched_calls(src, steps_path))
    except fusion_api.Unavailable as exc:
        print('check_compile: %s' % exc, file=sys.stderr)
        return 2
    for call in candidates:
        if call in watched or hits[call]:
            continue
        near = fusion_api.similar(call)
        problems.append("  the step list names '%s(', which the Fusion API database does not have%s"
                        % (call, '' if not near else
                           '; the nearest names it does have are %s' % ', '.join(near)))

    # 4. inputs have not drifted
    stamped = dict(re.findall(r'\|\s*`([\w./-]+)`\s*\|\s*`([0-9a-f]{40})`\s*\|', src))
    if not stamped:
        problems.append("  the step list carries no provenance table, so staleness cannot be seen")
    for path in sorted(provenance_inputs(gear) - set(stamped)):
        problems.append("  provenance omits required source %s" % path)
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

    # unverified calls, reported only
    if findings:
        print('unverified: %d call(s) the API database does not back — reported, not blocking, '
              'not waived' % len(findings))
        for line in findings:
            print('  %s' % line)

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
