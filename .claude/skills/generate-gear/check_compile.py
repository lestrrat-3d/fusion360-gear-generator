#!/usr/bin/env python3
"""Gate the compile stage: does the step list agree with its spec and its proof?

Compile turns `spec/<gear>/instructions.md` (plus `fusion.md`) into two artifacts, a step list at
`spec/<gear>/steps.md` and a runnable proof at `proof/<gear>/`. Neither is authoritative over the
other; this checks that they describe the same build and that both still match the prose they came
from.

Four checks gate, and one is reported:

  1. CITATIONS RESOLVE. Every step has a nonempty `**From:**` line naming real files and line
     ranges that exist.
  2. STEPS AND PROOF AGREE. Every `[GO]` step names the proof function that realises it, every
     proof function is claimed by a step, and every proof run's build argument is a literal
     `step<Title>` identifier so a step can claim it. Drift in either direction means one
     artifact moved without the other, and a build function under any other name is a proof no
     step can reach. The argument has to be written out, not forwarded with the rest of the
     argument list and not reached through a table or a variable, because the gate reads Go by
     matching braces rather than by compiling it: a run it cannot read to a function name is
     reported as unreadable, and it then says nothing about which steps are registered, rather
     than calling a registered step unregistered. A run is also unreadable when its own Test
     body binds a `step<Title>` name, when a condition the gate cannot read guards it, or when
     it sits outside every Test body. See THE CHOKEPOINT below for why those three are answered
     by refusing to read rather than by deciding.
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


def named_call_shapes(src):
    """Return the calls named in inline step-list code spans with receivers intact."""
    body = re.sub(r'```.*?```', '', src, flags=re.S)
    shapes = set()
    for span in re.findall(r'`([^`\n]+)`', body):
        shapes.update(call_shapes(span))
    for line in re.findall(r'<!--\s*check-compile:\s*ignore\s+([^>]*?)-->', src):
        ignored = set(line.split())
        shapes = {shape for shape in shapes if shape[0] not in ignored}
    return shapes


def is_watched_call(name, receiver):
    """Return whether a named call matches an unverified entry's receiver."""
    return any(
        watched_name == name and fusion_api.receiver_matches(receivers, receiver)
        for watched_name, _, receivers, _ in fusion_api.UNVERIFIED_CALLS)


def watched_calls(src, path):
    """Where the step list names a call from the unverified watchlist, receiver and all.

    An entry that names receivers only counts a call written on one of them, so the legitimate
    `chamferFeatures.createInput2()` is not dragged in beside `sketchTexts.createInput2(...)`.
    """
    seen = {}
    for name, _, receivers, _ in fusion_api.UNVERIFIED_CALLS:
        lines = sorted({
            line
            for match in re.finditer(r'`([^`\n]+)`', src)
            for called, receiver in call_shapes(match.group(1))
            if called == name and is_watched_call(called, receiver)
            for line in [src[:match.start()].count('\n') + 1]
        })
        if lines:
            seen[name] = '%s:%s' % (path, ','.join(str(line) for line in lines))
    return seen


def api_owner_matches_receiver(hits, receiver):
    """Return whether a database hit is declared on the named receiver class."""
    if receiver is None:
        return False
    receiver_name = receiver.rsplit('.', 1)[-1].lower()
    return any(
        qualified.rsplit('.', 2)[-2].lower() == receiver_name
        for qualified, _ in hits)


def proof_paths(src):
    """Return proof paths named by the step-list summary.

    The summary is the contract between the compiled step list and its committed proof. Keep
    this scan limited to paths under `proof/` or `.tmp/` so ordinary temporary build commands in
    later step prose do not become proof references.
    """
    summary = src.split('## Provenance', 1)[0]
    return sorted(set(re.findall(r'(?<![\w./-])(?:proof|\.tmp)/[\w./-]+', summary)))


def proof_path_is_tracked_or_committed(path):
    """Return whether an existing proof path is tracked by the index or present in HEAD."""
    if not os.path.isfile(path):
        return False
    tracked = subprocess.run(
        ['git', 'ls-files', '--error-unmatch', '--', path],
        capture_output=True, text=True)
    if tracked.returncode == 0:
        return True
    committed = subprocess.run(
        ['git', 'cat-file', '-e', 'HEAD:%s' % path],
        capture_output=True, text=True)
    return committed.returncode == 0


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


def go_func_body_spans(src, name_pattern):
    """Yield (name, start, end) for top-level Go functions whose names match name_pattern.

    start and end bound the body between the function's own braces. Positions come out with
    the body because a run has to be placed inside or outside a Test body before it is read.
    """
    pattern = r'(?m)^func\s+(%s)\s*\(' % name_pattern
    for m in re.finditer(pattern, src):
        open_brace = src.find('{', m.end())
        if open_brace == -1:
            continue
        close_brace = matching_delimiter(src, open_brace)
        if close_brace is None:
            continue
        yield m.group(1), open_brace + 1, close_brace


def go_func_bodies(src, name_pattern):
    """Yield (name, body) for top-level Go functions whose names match name_pattern."""
    for name, start, end in go_func_body_spans(src, name_pattern):
        yield name, src[start:end]


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


def go_open_delimiter(src, closing):
    """Return the index of the delimiter that src[closing] closes, or None."""
    pairs = {')': '(', ']': '[', '}': '{'}
    stack = [src[closing]]
    for index in range(closing - 1, -1, -1):
        char = src[index]
        if char in pairs:
            stack.append(char)
        elif char in '([{':
            if pairs[stack[-1]] != char:
                return None
            stack.pop()
            if not stack:
                return index
    return None


def go_statement_start(src, opening):
    """Where the statement whose block opens at `opening` begins.

    The scan walks back from the brace over whole bracket groups, so a condition wrapped
    across lines, and an arm the chain has already closed, are stepped over rather than
    ending it. It stops at the first statement boundary: the unmatched `{`, `(` or `[` of the
    construct this statement sits inside, or a newline where Go's own semicolon insertion
    would end the line. See THE GUARD AXIS below for what this reading can and cannot reach.
    """
    index = opening
    while index > 0:
        char = src[index - 1]
        if char in ')]}':
            start = go_open_delimiter(src, index - 1)
            if start is None:
                return index
            index = start
            continue
        if char in '([{':
            return index
        if char == '\n':
            if go_statement_ends_at_line(src[:index - 1]):
                return index
            index -= 1
            continue
        index -= 1
    return 0


def go_block_header(src, opening):
    """The whole header of the block opening at `opening`, on one line.

    It runs from the statement start, so it carries the construct's keyword whatever the lines
    between look like: an `if` whose condition wraps, an init statement, and for an `else` arm
    the whole chain it hangs off, up to but not including its own brace.
    """
    return re.sub(r'\s+', ' ', src[go_statement_start(src, opening):opening]).strip()


GUARD_HEADER = re.compile(r'^(?:[A-Za-z_]\w*\s*:\s+)*(if|else)\b')
ARM_KEYWORD = re.compile(r'\s*(else if|if|else)\b')


def go_header_is_guard(header):
    """Whether a header opens an arm of an if chain rather than an enclosure.

    Go gives only the if / else if / else family the power to skip its own block on a
    condition. Every other brace a walk crosses — `for`, `switch`, `select`, a func literal, a
    plain block, a composite literal, a labelled statement — encloses what is written inside
    it. So the keyword the header opens with settles this, and a header read back to the
    statement start always has that keyword in it.
    """
    return GUARD_HEADER.match(header) is not None


def go_arm_condition(header, start):
    """One arm's condition text from start, with the position after that arm's body.

    A `{` at the header's own depth ends the condition only when an `else` follows the group
    it opens, which is what a closed arm looks like from here. Any other group at that depth
    is a composite literal inside an init statement or a condition, so the scan steps over it
    and keeps reading.
    """
    index = start
    depth = 0
    while index < len(header):
        char = header[index]
        if char in '([':
            depth += 1
        elif char in ')]':
            depth -= 1
        elif char == '{' and depth == 0:
            closing = matching_delimiter(header, index)
            if closing is None:
                break
            if re.match(r'\s*else\b', header[closing + 1:]):
                return header[start:index].strip(), closing + 1
            index = closing
        index += 1
    return header[start:].strip(), len(header)


def go_guard_arms(header):
    """The conditions an if chain header states: the arms it falls through, and its own.

    Go writes a whole if / else if / else chain as one statement, so the header of any arm
    starts at the chain's `if` and carries every arm before it. The arm this header opens is
    the last one, and its own condition is None when it is a bare `else`.
    """
    text = header[GUARD_HEADER.match(header).start(1):]
    preceding = []
    own = None
    position = 0
    while position < len(text):
        keyword = ARM_KEYWORD.match(text, position)
        if keyword is None:
            break
        position = keyword.end()
        if keyword.group(1) == 'else':
            own = None
            break
        condition, position = go_arm_condition(text, position)
        own = condition
        if position >= len(text):
            break
        preceding.append(condition)
        own = None
    return preceding, own


def go_condition_value(condition):
    """True, False or None for one arm's condition: taken, never taken, or unreadable.

    The init statement an `if` may carry decides nothing, so only the expression after the
    last `;` at the condition's own depth is read. Three forms are known; anything else,
    a compound of literals included, is a condition this gate would have to evaluate, and it
    says it cannot read it rather than guessing.
    """
    if condition is None:
        return True
    depth = 0
    cut = -1
    for index, char in enumerate(condition):
        if char in '([{':
            depth += 1
        elif char in ')]}':
            depth -= 1
        elif char == ';' and depth == 0:
            cut = index
    expression = condition[cut + 1:].strip()
    if expression == 'false':
        return False
    if expression in ('true', 't != nil'):
        return True
    return None


def go_block_condition(src, opening):
    """Return the known reachability of a block, or None when it is unknown.

    An arm of an if chain runs when no arm before it was taken and its own condition holds, so
    a bare `else` inherits the readability of the chain it closes: it is unreadable whenever
    any condition before it is. Every other brace encloses rather than guards, and what an
    enclosure holds runs when the statement around it does. Reading those enclosures as guards
    is how a proof that registers its steps from a table used to look as though no Test
    registered anything.
    """
    header = go_block_header(src, opening)
    if not go_header_is_guard(header):
        return True
    preceding, own = go_guard_arms(header)
    values = [go_condition_value(condition) for condition in preceding]
    own_value = go_condition_value(own)
    if own_value is False or any(value is True for value in values):
        return False
    if any(value is None for value in values):
        return None
    return own_value


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
        # Only an if arm can return early on a known-true condition. A loop or a closure whose
        # whole body is a return says nothing about what follows it.
        if not go_header_is_guard(go_block_header(src, opening)):
            continue
        if go_block_condition(src, opening) is not True:
            continue
        if re.fullmatch(r'\s*return\s*;?\s*', src[opening + 1:closing]):
            return True
    return False


def go_call_reachability(src, pos, brace_pairs):
    """Return True, False or None for whether a proof run at pos runs on the test path.

    True when nothing the gate reads keeps it from running, False when a known-false
    condition or an unconditional early return does, and None when a block encloses it whose
    condition the gate cannot read.

    None is not False. A run the gate cannot decide about is one it cannot report on, so the
    caller reads it as unreadable rather than dropping it. Dropping it was worse than saying
    nothing: the steps such a run registers were then reported as registered nowhere, which
    sends a drafter to fix a proof that already registers them.
    """
    if go_top_level_return_before(src, pos) or go_early_return_before(src, pos, brace_pairs):
        return False
    unknown = False
    for opening, closing in brace_pairs.items():
        if not opening < pos < closing:
            continue
        condition = go_block_condition(src, opening)
        if condition is False:
            return False
        if condition is None:
            unknown = True
    return None if unknown else True


def go_argument_label(argument):
    """A one-line label for a Go argument expression, for diagnostics."""
    label = re.sub(r'\s+', ' ', argument).strip()
    if len(label) > 60:
        label = label[:57] + '...'
    return label


def go_statement_ends_at_line(text):
    """Whether Go would end a statement at the end of this line.

    Go inserts the semicolon itself, and only when the line's last token can end a statement:
    an identifier, a literal, a closing bracket, or ++/--. A line ending in `=`, a comma or an
    operator continues into the next one, which is what keeps a multi-line declaration from
    being read as several.
    """
    stripped = text.rstrip()
    if not stripped:
        return True
    return re.search(r'(?:\w|[)\]}\'"]|\+\+|--)$', stripped) is not None


def go_var_specs(body, start):
    """The declaration specs of the var declaration at start.

    One spec for a single `var` declaration, and one per declaration for a parenthesised
    block. Specs are cut at the block's own nesting depth, so a composite literal or a call in
    an initializer cannot split one, and only where Go would end the statement, so a wrapped
    initializer cannot either. Only the names a spec binds are read from it.
    """
    rest = body[start:]
    offset = len(rest) - len(rest.lstrip())
    if rest[offset:offset + 1] != '(':
        return [rest]
    closing = matching_delimiter(body, start + offset)
    if closing is None:
        return []
    specs = []
    current = ''
    depth = 0
    first = start + offset + 1
    for ch in body[first:closing]:
        if ch in '([{':
            depth += 1
        elif ch in ')]}':
            depth -= 1
        if depth == 0 and (ch == ';' or (ch == '\n' and go_statement_ends_at_line(current))):
            specs.append(current)
            current = ''
            continue
        current += ch
    specs.append(current)
    return specs


def go_declared_names(spec):
    """The names one var declaration binds: the identifier list before the type and any `=`.

    Only the left-hand side, because a declaration's initializer may name a real step function,
    and treating that name as a binding would make a literal registration of it anywhere in the
    same Test body look unreadable.
    """
    match = re.match(r'\s*([A-Za-z_]\w*(?:\s*,\s*[A-Za-z_]\w*)*)', spec)
    if match is None:
        return []
    return re.findall(r'[A-Za-z_]\w*', match.group(1))


def go_short_declaration_names(body, assign):
    """The identifiers the `:=` at assign binds, read backwards from it.

    Go's short variable declaration takes an identifier list on its left, so nothing but
    identifiers, commas and whitespace can stand there. The scan walks back over exactly that
    and stops at anything else. That is why it can cross the newline of a left-hand side
    written over several lines and still cannot run into the statement before it: a line whose
    last token can end a statement stops the scan, which is the same rule Go's own semicolon
    insertion uses.

    A keyword swept up in front of the list — `for`, `if`, `case` — costs nothing, because
    these names are judged against `step<Title>` at the chokepoint and no Go keyword can match
    it. What a name here costs is measured on that path, not on the build-argument path.
    """
    start = assign
    while start > 0:
        ch = body[start - 1]
        if ch == '\n':
            if go_statement_ends_at_line(body[:start - 1]):
                break
            start -= 1
            continue
        if ch.isalnum() or ch in '_, \t':
            start -= 1
            continue
        break
    return re.findall(r'[A-Za-z_]\w*', body[start:assign])


# A type keyword can lead a parameter group without any name standing in front of it, so a
# group starting with one of these is a bare type: `chan int`, `map[string]int`, `struct {`,
# `interface {`, `func(`. Only `chan` and the two brace forms can hold whitespace at the
# group's own top level, which is why the keyword has to be recognised rather than the space.
GO_TYPE_KEYWORDS = frozenset(('chan', 'map', 'struct', 'interface', 'func'))

GO_GROUP_HEAD = re.compile(r'\s*([A-Za-z_]\w*)(\s+(?=\S))?', re.S)


def go_skip_line_space(text, index):
    """The index of the first character at or after index that is not a space or a tab.

    Newlines are not crossed. Go ends a statement at a line break whose last token can end
    one, so a `func` signature and the brace opening its body stand on one line, and a `{` on
    the next line opens a plain block after a func TYPE instead.
    """
    while index < len(text) and text[index] in ' \t':
        index += 1
    return index


def go_type_end(text, index):
    """The index just past the type written at index, or None if no type is written there.

    Only enough of Go's type grammar to walk over one result type: the prefixes `*`, `[]`,
    `[N]`, `<-` and `...`, the four keyword forms, and a plain or qualified name with an
    optional generic instantiation. It exists to find the brace after a func literal's single
    unparenthesised result, `func(arg stepType) error {`, which is a literal whose parameters
    bind and which a bare "is the next character a brace" test would drop.
    """
    while True:
        index = go_skip_line_space(text, index)
        if text.startswith('...', index) or text.startswith('<-', index):
            index += 3 if text.startswith('...', index) else 2
            continue
        if text[index:index + 1] == '*':
            index += 1
            continue
        if text[index:index + 1] == '[':
            closing = matching_delimiter(text, index)
            if closing is None:
                return None
            index = closing + 1
            continue
        break
    word = re.match(r'[A-Za-z_]\w*', text[index:])
    if word is None:
        return None
    index += word.end()
    if word.group() == 'chan':
        return go_type_end(text, index)
    if word.group() == 'map':
        index = go_skip_line_space(text, index)
        if text[index:index + 1] != '[':
            return None
        closing = matching_delimiter(text, index)
        return None if closing is None else go_type_end(text, closing + 1)
    if word.group() in ('struct', 'interface'):
        index = go_skip_line_space(text, index)
        if text[index:index + 1] != '{':
            return None
        closing = matching_delimiter(text, index)
        return None if closing is None else closing + 1
    if word.group() == 'func':
        index = go_skip_line_space(text, index)
        if text[index:index + 1] != '(':
            return None
        closing = matching_delimiter(text, index)
        if closing is None:
            return None
        after = go_skip_line_space(text, closing + 1)
        if text[after:after + 1] == '(':
            results = matching_delimiter(text, after)
            return closing + 1 if results is None else results + 1
        # A func type's own result is optional, and what follows may be the brace of the body
        # this whole type is the result of, so an unreadable one ends the type here.
        return go_type_end(text, after) or closing + 1
    qualified = re.match(r'\s*\.\s*[A-Za-z_]\w*', text[index:])
    if qualified is not None:
        index += qualified.end()
    if text[index:index + 1] == '[':
        closing = matching_delimiter(text, index)
        if closing is None:
            return None
        index = closing + 1
    return index


def go_func_literal_body_start(body, open_paren):
    """The index of the brace opening the body of the func whose list opens at open_paren.

    None when no brace follows the signature, which is what a func TYPE writes. A type binds
    nothing. `var handle func(arg stepType)`, `type handler func(arg stepType)`, a func-typed
    struct field and a func type nested in another literal's parameter list all write a
    parameter list that no body can read, so reading names out of one collects identifiers
    that nothing in the file binds.

    Requiring a body drops exactly those and excludes nothing that binds, because every func
    literal in Go has a body. What stands between the parameter list and that body is either
    nothing, a parenthesised result list, or one unparenthesised result type.

    This test is local, so it can ask only whether a brace FOLLOWS the signature, never whose
    brace it is. A func type written inside other type text is followed by a brace that opens
    something else — the body of the literal whose result the type is, or a composite
    literal's value — and this test accepts it. Whose brace it is depends on the text around
    the match, so it is decided by the scan in `go_func_literal_parameter_lists`, which is
    what callers wanting literals only must use.
    """
    closing = matching_delimiter(body, open_paren)
    if closing is None:
        return None
    index = go_skip_line_space(body, closing + 1)
    if body[index:index + 1] == '(':
        results = matching_delimiter(body, index)
        if results is None:
            return None
        index = go_skip_line_space(body, results + 1)
    elif body[index:index + 1] not in ('{', ''):
        index = go_type_end(body, index)
        if index is None:
            return None
        index = go_skip_line_space(body, index)
    return index if body[index:index + 1] == '{' else None


def go_element_type_bracket_before(body, keyword):
    """Whether the `func` keyword at keyword is the element type of a bracketed type.

    `[]func(arg stepType)`, `[2]func(arg stepType)` and `map[string]func(arg stepType)` all
    write a func TYPE, and all three announce it the same way: with the `]` that closes their
    brackets standing immediately before the `func`. Go writes no expression that puts a `]`
    there, so the character decides it, and it is the only character that can — a func literal
    in expression position follows an operator, a delimiter, a keyword, or nothing.

    Only spaces and tabs are crossed looking back, never a line break. A statement ending in
    `]` on the line above says nothing about the `func` opening the next one, and gofmt writes
    the bracketed type and its element type together on one line.
    """
    index = keyword - 1
    while index >= 0 and body[index] in ' \t':
        index -= 1
    return index >= 0 and body[index] == ']'


def go_func_literal_parameter_lists(body):
    """Every func LITERAL's parameter list in body, as open-paren indexes in source order.

    A `func(` match is a literal only when it opens one. The same characters write a func TYPE
    inside larger type text, where the brace `go_func_literal_body_start` finds belongs to
    something else, and a type binds nothing anywhere in the program. Two shapes reach it:

      1. A result type. In `factory := func() func(arg stepType) {`, the returned type's own
         parameter list is followed by the OUTER literal's body brace.
      2. A bracketed element type. In `table := []func(arg stepType){}`, the parameter list is
         followed by the composite literal's value brace.

    Neither is decidable from the brace, and both are decidable from the scan. A literal's
    result type runs from its parameter list to its body brace, so every match inside that
    span is type text, and the scan runs in source order, which puts the enclosing literal's
    span in hand before the matches it covers. Case 2 is decided by the bracket that precedes
    the keyword.

    What this does NOT decide is scope. A func literal nested in another one is a literal, and
    its parameters are collected, because the chokepoint below asks only whether a name is
    bound somewhere in the body. That conservative reading is the design, not an oversight.
    """
    open_parens = []
    type_spans = []
    for match in re.finditer(r'\bfunc\s*\(', body):
        if any(start <= match.start() < end for start, end in type_spans):
            continue
        if go_element_type_bracket_before(body, match.start()):
            continue
        open_paren = match.end() - 1
        brace = go_func_literal_body_start(body, open_paren)
        if brace is None:
            continue
        open_parens.append(open_paren)
        type_spans.append((matching_delimiter(body, open_paren) + 1, brace))
    return open_parens


def go_signature_groups(text):
    """One parameter or result list, split into groups at its own top-level commas.

    A comma inside brackets, parentheses or braces belongs to a type — `pair[A, B]`, a nested
    `func(a, b int)`, a `struct{ x, y int }` field — and separates nothing at this level, so
    the split tracks nesting depth instead of splitting the text plainly. Groups holding only
    whitespace are dropped, which is what a list gofmt wrapped over several lines writes after
    its trailing comma.
    """
    groups = []
    current = ''
    depth = 0
    for ch in text:
        if ch in '([{':
            depth += 1
        elif ch in ')]}':
            depth -= 1
        elif ch == ',' and depth == 0:
            groups.append(current)
            current = ''
            continue
        current += ch
    groups.append(current)
    return [group for group in groups if group.strip()]


def go_group_head(group):
    """The identifier leading this group, and whether type text follows it.

    Only the LEADING identifier can be a name. Everything from the first type token on is type
    text and is skipped whole, which is what keeps `arg` and `stepType` out of
    `func(cb func(arg stepType))` and a field name out of `func(cfg struct{ stepField int })`.

    Whitespace is what separates a name from its type. `pair[stepType]` and `testing.T` open
    with an identifier too, and gofmt writes no space before the bracket or dot continuing
    one, so an identifier running straight into either is the head of a type and not a name.
    A group opening with a type keyword is a bare type whatever follows it.
    """
    match = GO_GROUP_HEAD.match(group)
    if match is None or match.group(1) in GO_TYPE_KEYWORDS:
        return None, False
    if match.group(2) is not None:
        return match.group(1), True
    if group[match.end():].strip():
        return None, False
    return match.group(1), False


def go_parameter_names(text):
    """The names one parameter or result list binds, and none if the list is unnamed.

    Named and unnamed cannot be told apart one group at a time. `(stepType, error)` and
    `(a, b int)` both hold groups of a single identifier, and only the list as a whole
    separates them: Go forbids mixing the two forms, so ONE group carrying a name and then a
    type puts the whole list in named form, and every group's leading identifier is a name.
    Deciding per group instead reads the type in `func(stepType)` as a name, or loses `a` in
    `func(a, b stepType)`.
    """
    heads = [go_group_head(group) for group in go_signature_groups(text)]
    if not any(carries_type for _, carries_type in heads):
        return []
    return [name for name, _ in heads if name is not None]


def go_signature_names(body, open_paren):
    """The names a func literal's parameter list, and its named result list, bind.

    Names only, never the types standing beside them. These names reach `go_bound_step_names`,
    which keeps everything matching `step<Title>`, so a type name swept up here is read as a
    binding: a package-level `type stepResult struct{}` named in a parameter's type would make
    every run in the body unreadable, suppress the whole proof directory's registration
    verdict, and tell the drafter to rename a local that was never written. That is the
    chokepoint path. Reading the lists whole was justified by the build-argument path, where a
    type name really does cost nothing, and the two paths are not the same one.

    Each list is read on its own, because each carries its own named-or-unnamed form. The
    result list is read only when it is parenthesised, since an unparenthesised single result
    is a bare type and binds nothing.
    """
    closing = matching_delimiter(body, open_paren)
    if closing is None:
        return []
    names = go_parameter_names(body[open_paren + 1:closing])
    rest = body[closing + 1:]
    offset = len(rest) - len(rest.lstrip())
    if rest[offset:offset + 1] == '(':
        results = matching_delimiter(body, closing + 1 + offset)
        if results is not None:
            names += go_parameter_names(body[closing + offset + 2:results])
    return names


def go_bound_names(body):
    """Every name a Go function body binds, with no scope attached to any of them.

    Three constructs write a binding: a `:=`, a `var` declaration, and a func literal's
    parameter or named result list. Every other binding form in Go's grammar — a for-range
    header, a three-clause `for` init, an `if` or `switch` init, a type-switch guard, a
    `case v := <-ch:` in a select, a case clause body — is one of those three wearing a
    keyword, so reading the three reads them all.

    No position is recorded, because the chokepoint below asks only whether a name is bound
    somewhere in the body, never where.

    `const` and `type` are deliberately NOT read, and that is a claim you can falsify rather
    than a gap. Neither can occupy a build slot: every `PROOF_RUN_CALLS` entry points at slot
    2, which is `proofkit.Build` or `proofkit3d.Build`, both func types. Go restricts constants
    to boolean, rune, integer, float, complex and string, so `const stepOne proofkit.Build =
    nil` is rejected as `invalid constant type`, and `const stepOne = 0` passed to a run is
    rejected as `cannot use stepOne (untyped int constant 0) as proofkit.Build value`. A type
    name is never an expression, so `type stepOne struct{}`, a `type stepOne = proofkit.Build`
    alias and a func-typed declaration are all rejected as `stepOne (type) is not an
    expression`. Checked against the real packages with go1.26.1.

    Reading them would therefore add false failures and no soundness: the only step-named
    `const` or `type` declarations that COMPILE are ones a run does not resolve against —
    declared after the run, or in a sibling block — and in each the run's argument really is
    the package-level step function, so counting it registered is the correct answer. The one
    shape where such a name reaches a run's argument text is a conversion,
    `stepOne(buildX)`, and that is already unreadable because it is not a bare identifier.

    To falsify this, exhibit a `const` or `type` declaration that COMPILES as a build argument
    and shadows a package-level step function. `go vet` deciding it is the whole test.
    """
    names = set()
    for match in re.finditer(r':=', body):
        names.update(go_short_declaration_names(body, match.start()))
    for match in re.finditer(r'\bvar\b', body):
        for spec in go_var_specs(body, match.end()):
            names.update(go_declared_names(spec))
    for open_paren in go_func_literal_parameter_lists(body):
        names.update(go_signature_names(body, open_paren))
    return names


# THE CHOKEPOINT, AND WHAT IT GIVES UP
#
# A Test body that binds a `step<Title>` name itself makes every proof run in that body
# unreadable. The rule makes no scope decision at all, so no header shape can defeat it.
#
# A root-cause map of this gate's local-binding handling found seven gaps across the Go binding
# constructs and offered two ways out: this one, the conservative chokepoint, and a real parse.
# The conservative one was ruled, and the reasoning on both sides is kept here rather than in a
# commit message, because the next reader of this file is the one who needs it.
#
# It replaces a character-level guess at Go's scoping rule — "the brace group whose close is
# the last thing on the statement is the block this header opens". Three review rounds each
# repaired one header shape that defeated that guess, and each time a fourth shape appeared,
# because the guess is not an approximation of Go's grammar but a different rule that happens
# to agree with it on the shapes anyone had written down yet.
#
# What this deliberately gives up is precision. It reports runs unreadable that a real scope
# analysis would accept: a `step<Title>` local declared in a sibling block, or after the run,
# or inside a loop the run is not in, now blocks the whole body. That is the price of a rule
# with no grammar of its own, and it is paid in false failures, which are loud, rather than in
# false passes, which are silent and let unproven geometry through a green gate. A proof pays
# it only by naming a local `step<Title>`, which no proof needs to do.
#
# THREE THINGS BRACE MATCHING CANNOT DECIDE, EVEN IN PRINCIPLE
#
#   1. A brace group that ends a header clause. In `for a, b := f(), []T{x}; cond; post {` the
#      clause-ending group and the block are character-identical, and gofmt spaces them the
#      same way. Telling them apart means knowing how many `;`-separated clauses the header
#      has and which one the scan is in, which is parsing, not matching.
#   2. A binding with no brace to hang it on. A `switch` case clause is an implicit block that
#      writes no brace; func-literal parameters and named results bind inside parentheses;
#      `const`, `type` and labelled statements are grammar facts with no delimiter of their
#      own. No refinement of "what follows the brace" reaches any of them.
#   3. Whether a guarded run executes. `if cond { run }` is decidable only by evaluating
#      `cond`, which is undecidable in general. The gate reads such a run as unreadable, which
#      is the honest answer, rather than deciding it either way.
#
# Item 3's sibling is the run written in a helper rather than in a `Test` body. Whether a Test
# reaches it is a call-graph question this gate does not ask, so such a run is reported
# unreadable too. That is a known limit, not a fix: a helper that a Test really does call
# still has to be reported, because nothing here can tell it from one that no Test calls.
#
# OPTION B, DEFERRED AND KEPT OPEN
#
# The sound fix is a real parse: a small `go/parser` plus `go/ast` helper emitting binding
# spans and the resolved build-argument identifier, leaving this module with one lookup and no
# grammar of its own. It closes all three items above, because a parser answers by
# construction what brace matching can only guess at. The repo already ships a Go module and
# toolchain under `proof/`, so the dependency exists. It is deferred, not rejected, because it
# makes this Python gate depend on the Go toolchain being present to run at all, which is a
# structural change beyond the scope this decision was taken in. Take it up and this whole
# chokepoint, along with the machinery above it, is deleted rather than refined.
#
# THE GUARD AXIS: IS A BRACE A GUARD OR AN ENCLOSURE
#
# The map above crossed binding constructs with header shapes to decide which NAMES a body
# binds. This is the other question the walk asks of a brace it crosses on its way to a run:
# does that brace guard the run, and if it does, can its condition be read. The two axes are
# independent, and the chokepoint answers only the first, so this one is answered here, by
# `go_block_header` and `go_block_condition`.
#
# Go's grammar closes this axis, not this repo's conventions, which is why it can be listed
# once. Only the if / else if / else family skips its own block on a condition. A `for`, a
# `switch`, a `select`, a func literal (invoked, deferred, started with `go`, or handed to
# `t.Run`), a plain block, a labelled statement and a composite literal all enclose what is
# written inside them, and what an enclosure holds runs when the statement around it runs. So
# the walk reads the header back to the statement start and dispatches on the keyword standing
# there: the enclosure verdict is what that keyword says, rather than what an unread header
# happened to fall through to. Reading only the last line before the brace was the earlier
# rule, and it left a wrapped `if` header as `)`, which matched no condition and made the guard
# read as an enclosure — the same guard passing when written over three lines and failing when
# written on one.
#
# An `else` arm carries its whole chain in its header, because Go writes a chain as one
# statement. It runs when every condition before it was false, so it inherits their
# readability: the `else` closing an unreadable `if` is unreadable itself, and a run inside it
# is reported rather than counted.
#
# WHAT THIS AXIS STILL CANNOT READ
#
#   1. A condition that is not one of three literal forms. `false` is dead, `true` and
#      `t != nil` are live, and everything else — a call, a variable, a comparison, and a
#      compound of literals such as `false || false` — is unreadable. That is item 3 of the
#      list above reached from the other side. The compound is decidable and is still refused,
#      because deciding it means evaluating Go, and this axis reads keywords.
#   2. A statement boundary gofmt does not write. The scan ends a statement where Go's own
#      semicolon insertion ends a line, so two statements written on one line with a `;`
#      between them would be read as a single header. gofmt splits them, and no proof here
#      writes them.
#   3. An `if` left alone on its line with its condition starting the next one. That is legal
#      Go, the scan stops at the line break, and the block then reads as an enclosure. gofmt
#      rewrites it to one line, so no committed proof can hold the shape; the test table pins
#      both the misreading and the rewrite rather than leaving the gap unwritten.


STEP_FUNCTION_NAME = re.compile(r'step[A-Z]\w*')


def go_bound_step_names(bound_names):
    """The step<Title> names among a Test body's bindings: the chokepoint's whole input."""
    return sorted(name for name in bound_names if STEP_FUNCTION_NAME.fullmatch(name))


def go_build_argument_names_a_function(argument, bound_names):
    """Return whether a build argument says, on its own, which function the run builds with.

    A package-level identifier says it, and so does a function literal, which says it is
    anonymous and therefore no step's. Anything else — a name the Test body binds itself, a
    struct field, an index, a call, a qualified name — is an expression the gate would have to
    evaluate to know what the run builds with, and this is brace matching over scrubbed
    source, not a Go compiler.
    """
    if re.fullmatch(r'[A-Za-z_]\w*', argument):
        return argument not in bound_names
    return re.match(r'func\s*\(', argument) is not None


UNREADABLE_ARGUMENT = ("write the run's arguments out one by one, with the build argument a "
                       "literal step<Title> identifier so a step can claim the run")
UNREADABLE_GUARD = ("the gate cannot read the condition that guards it, so it cannot say "
                    "whether the run executes; write the run where nothing it cannot read "
                    "guards it")
UNREADABLE_OUTSIDE_TEST = ("the run is not inside a Go Test function, so the gate cannot say "
                           "which Test reaches it; register the step from a Test body")


def unreadable_local_step_reason(names):
    """Why a Test body that binds a step<Title> name of its own makes its runs unreadable."""
    return ("this Test body binds %s itself, so a step<Title> build argument written here may "
            "be that local rather than the step function of the same name; rename the local so "
            "no step<Title> name is bound in a Test body that registers a run"
            % ', '.join(names))


def registered_step_functions(src):
    """Return (step functions, misnamed builds, unreadable runs) for reachable proofkit runs.

    All three come out of the same parse of the same run, because they are the same fact read
    three ways: a run's build argument is a step function, or is a build the step list has no
    name for, or is something this parse cannot read to a function name at all. Dropping the
    second is how a proof registered under a name like buildSolid used to escape every check —
    no step could claim it, and nothing looked for it. Dropping the third is worse, because the
    gate then reports the steps such a run registers as registered nowhere, which is false.
    Only the build argument is judged; the gate and assertion arguments are not steps.

    A run is unreadable in five ways, and they are one category because they leave the gate in
    one state — it cannot say which function the run builds with. The Test body may bind a
    step<Title> name itself, which is the chokepoint above and covers every scope question at
    once. A block the gate cannot read may guard the run. The argument list may never close; it
    may be shorter than the build slot, which is what a run written as one multi-value call
    parses to, the arguments forwarded from a helper rather than written out; or the build
    argument may be an expression, a bound name or a table entry rather than a name. Each
    compiles, so silently skipping any of them would let a build argument escape the
    step<Title> check. Each unreadable run carries the label of the thing to look at and the
    reason the gate stopped there. The label carries the whole call where no argument reached
    the slot, and the argument itself where one did.

    A run outside every Test body is reported the same way, because whether a Test reaches the
    helper holding it is a call-graph question this gate does not ask.
    """
    registered = set()
    misnamed = set()
    unreadable = set()
    test_spans = []
    for _, body_start, body_end in go_func_body_spans(src, r'Test[A-Z]\w*'):
        test_spans.append((body_start, body_end))
        body = src[body_start:body_end]
        brace_pairs = go_brace_pairs(body)
        bound_names = go_bound_names(body)
        bound_step_names = go_bound_step_names(bound_names)
        for pattern, build_arg in PROOF_RUN_CALLS:
            for m in re.finditer(pattern, body):
                reachable = go_call_reachability(body, m.start(), brace_pairs)
                if reachable is False:
                    continue
                open_paren = m.end() - 1
                close_paren = matching_delimiter(body, open_paren)
                call = (body[m.start():] if close_paren is None
                        else body[m.start():close_paren + 1])
                if reachable is None:
                    unreadable.add((go_argument_label(call), UNREADABLE_GUARD))
                    continue
                if close_paren is None:
                    unreadable.add((go_argument_label(call), UNREADABLE_ARGUMENT))
                    continue
                args = split_go_args(body[open_paren + 1:close_paren])
                if len(args) <= build_arg:
                    unreadable.add((go_argument_label(call), UNREADABLE_ARGUMENT))
                    continue
                argument = args[build_arg]
                if bound_step_names:
                    unreadable.add((go_argument_label(argument),
                                    unreadable_local_step_reason(bound_step_names)))
                elif STEP_FUNCTION_NAME.fullmatch(argument):
                    registered.add(argument)
                elif go_build_argument_names_a_function(argument, bound_names):
                    misnamed.add(go_argument_label(argument))
                else:
                    unreadable.add((go_argument_label(argument), UNREADABLE_ARGUMENT))
    for pattern, _ in PROOF_RUN_CALLS:
        for m in re.finditer(pattern, src):
            if any(start <= m.start() < end for start, end in test_spans):
                continue
            close_paren = matching_delimiter(src, m.end() - 1)
            call = src[m.start():] if close_paren is None else src[m.start():close_paren + 1]
            unreadable.add((go_argument_label(call), UNREADABLE_OUTSIDE_TEST))
    return registered, sorted(misnamed), sorted(unreadable)


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
    """Return the registrations a proof directory makes, as three sets.

    They are the step functions registered from a Go Test, the misnamed builds as (file,
    label) pairs, and the unreadable runs as (file, label, reason) triples. A label carries its
    file, because the argument or the call alone does not say where to go and fix it, and an
    unreadable run carries its reason, because the five ways a run can be unreadable are fixed
    in five different places.
    """
    found = set()
    misnamed = set()
    unreadable = set()
    if not os.path.isdir(proof_dir):
        return found, [], []
    for entry in sorted(os.listdir(proof_dir)):
        if not entry.endswith('_test.go'):
            continue
        path = os.path.join(proof_dir, entry)
        src = strip_go_comments_and_literals(read(path))
        registered, other, unread = registered_step_functions(src)
        found.update(registered)
        misnamed.update((path, argument) for argument in other)
        unreadable.update((path, label, reason) for label, reason in unread)
    return found, sorted(misnamed), sorted(unreadable)


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

    # The summary must point at the committed proof, not an ignored compiler output.
    for path in proof_paths(src):
        if not os.path.exists(path):
            problems.append("  proof path %s does not exist" % path)
        elif not proof_path_is_tracked_or_committed(path):
            problems.append("  proof path %s is not tracked or committed" % path)

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
    registered, misnamed_builds, unreadable_runs = proof_registrations(proof_dir)
    # One run the gate cannot read leaves every registration in this proof unknown, because
    # that run may be the one registering any of these step functions. Say the run is
    # unreadable, and say nothing about who is registered — a false "no Go Test registers it"
    # sends a drafter to fix a proof that is already doing the thing it is accused of skipping.
    registration_is_known = not unreadable_runs
    claimed = set()
    for sid, tag, body in steps:
        named = set(re.findall(r'\b(step[A-Z]\w*)\b', body))
        if tag == 'GO' and not named:
            problems.append("  %s is tagged [GO] but names no proof function" % sid)
        for fn in named:
            if fn not in functions:
                problems.append("  %s names proof function %s, which %s/ does not define"
                                % (sid, fn, proof_dir))
            elif registration_is_known and fn not in registered:
                problems.append("  %s names proof function %s, but no Go Test registers it "
                                "in a proofkit run" % (sid, fn))
            claimed.add(fn)
    for fn in sorted(functions - claimed):
        problems.append("  proof function %s is not claimed by any step" % fn)
    if registration_is_known:
        for fn in sorted(functions - registered):
            problems.append("  proof function %s is defined but is not registered in any "
                            "proofkit run inside a Go Test" % fn)
    for path, argument in misnamed_builds:
        problems.append("  %s registers %s as a proof run's build argument, but that argument "
                        "must be a step<Title> function so a step can claim it"
                        % (path, argument))
    for path, label, reason in unreadable_runs:
        problems.append("  %s has a proof run the gate cannot read as a registration: %s; %s"
                        % (path, label, reason))

    # 3. API calls are real
    local = PYTHON_METHODS | defined_names(FRAMEWORK) | contract_names(gear)
    watched = {name for name, _, _, _ in fusion_api.UNVERIFIED_CALLS}
    shapes = named_call_shapes(src)
    wrong_watchlist_receivers = {}
    for called, receiver in shapes:
        if called in watched and not is_watched_call(called, receiver):
            wrong_watchlist_receivers.setdefault(called, set()).add(receiver)
    candidates = sorted(
        name for name in named_calls(src)
        if name not in local
        and (name not in watched or any(
            called == name and not is_watched_call(called, receiver)
            for called, receiver in shapes)))
    try:
        # Every candidate is either an ordinary call or a watchlist method on the wrong
        # receiver. The latter must reach the database instead of being exempted by name.
        hits = fusion_api.lookup_many(candidates)
        findings = fusion_api.unverified_findings(watched_calls(src, steps_path))
    except fusion_api.Unavailable as exc:
        print('check_compile: %s' % exc, file=sys.stderr)
        return 2
    for call in candidates:
        wrong_receivers = wrong_watchlist_receivers.get(call, ())
        if (hits[call]
                and all(api_owner_matches_receiver(hits[call], receiver)
                        for receiver in wrong_receivers)):
            continue
        if hits[call] and wrong_receivers:
            owners = sorted({qualified.rsplit('.', 2)[-2] for qualified, _ in hits[call]})
            for receiver in sorted(wrong_receivers, key=lambda value: value or ''):
                if not api_owner_matches_receiver(hits[call], receiver):
                    problems.append(
                        "  the step list names '%s(' on receiver '%s', but the Fusion API "
                        "database declares it on %s"
                        % (call, receiver, ', '.join(owners)))
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
