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
     proof function is claimed by a step, and every proof function is built by the Go Test of its
     own title: `stepGearProfileSketch` by `TestGearProfileSketch`, and by no other. Drift in
     either direction means one artifact moved without the other, and a build the matching Test
     does not name is a proof no step can reach. The registration is written in one fixed shape
     the gate matches by line ("Registration is read by SHAPE" below says why and states it); a
     run written any other way is reported rather than read, because an unread build argument is
     an unchecked one.
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


# Registration is read by SHAPE, not by parsing Go.
#
# The gate has one question about the proof: which step function does each proof run build with?
# Answering it by reading Go is answering a much harder question than the one asked, and the
# checker spent a long time getting Go's grammar wrong at the edges — a type brace in a range
# header, a func literal in a for-init, a wrapped if, a func type in a result position. Every one
# of those was a shape no committed proof contains.
#
# So the proof declares its registrations in a fixed shape instead, and the gate matches that
# shape. A registration is three lines and nothing else:
#
#     func Test<Title>(t *testing.T) {
#             proofkit.Run(t, <cases>, step<Title>)
#     }
#
# with `proofkit3d.Run`, `proofkit3d.RunSolid` or `proofkit3d.RunWithGate` in place of
# `proofkit.Run`, and an assertion argument after the build where the run takes one. The build
# argument is always the third. The Test's title and the step's title are the same word, which is
# what lets a step list claim a function by name alone.
#
# What this buys is that the gate never has to decide what a brace means. What it costs is that a
# run written any other way is refused rather than read. That is the safe direction: a refusal
# names the file and line and says what to write, while a misreading silently credits a step no
# test builds. `.claude/skills/compile-gear/SKILL.md` states the same shape to the drafter, so the
# refusal is a contract violation rather than a surprise.
#
# Comments and literals are blanked before matching, so a registration quoted inside a doc comment
# or a raw string is not mistaken for a real one. Blanking preserves newlines, so a line number in
# a complaint is a line number in the file.

GO_BUILD_CONSTRAINT = re.compile(r'^\s*//\s*(?:go:build|\+build)\b')

STEP_DEFINITION = re.compile(r'^func\s+(step[A-Z]\w*)\s*[\[(]')

TEST_HEADER = re.compile(r'^func\s+(Test[A-Z]\w*)\s*\(\s*\w+\s+\*testing\.T\s*\)\s*\{\s*$')

# The build slot is a bare name, because it has to be `step<Title>` for a step to claim it. Every
# other argument may be qualified, so a gate like `proofkit3d.RequireSolid` reads as one argument.
# The suffix is tied to its namespace, because the four run methods are the four that exist:
# `proofkit` defines only `Run`, and the `Solid` and `WithGate` variants belong to `proofkit3d`
# alone. Letting the namespace and the suffix vary independently would credit a registration built
# on `proofkit.RunSolid`, which no package defines and Go cannot compile.
PROOF_RUN = re.compile(
    r'^\s*(?:proofkit\s*\.\s*Run|proofkit3d\s*\.\s*Run(?:Solid|WithGate)?)\s*\('
    r'\s*[\w.]+\s*,\s*[\w.]+\s*,\s*(\w+)\s*(?:,\s*[\w.]+\s*)*\)\s*$')

# The mention pattern stays deliberately wider than the shape above: it is what turns a run the
# shape refuses into a named complaint. A line calling `proofkit.RunSolid` has to be seen here, or
# a registration on a method that does not exist would pass by going unnoticed instead of failing.
PROOF_RUN_MENTION = re.compile(r'\bproofkit(?:3d)?\s*\.\s*Run(?:Solid|WithGate)?\s*\(')

BLOCK_END = re.compile(r'^\}\s*$')

STEP_NAME = re.compile(r'step[A-Z]\w*')

REGISTRATION_SHAPE = ('a registration is three lines and nothing else: '
                      '`func Test<Title>(t *testing.T) {`, one proof run whose third argument is '
                      '`step<Title>`, then `}`')


def scan_proof_file(path):
    """Return one proof file's step definitions, registrations and complaints.

    A registration comes back as (line number, Test name, build argument) so a complaint can say
    where to go. Complaints here are the ones only this file can see: a build constraint in the
    file header, and a proof run written outside the registration shape.
    """
    src = read(path)
    raw = src.splitlines()
    code = strip_go_comments_and_literals(src).splitlines()
    defined = set()
    registrations = []
    complaints = []
    run_lines = set()

    for index, line in enumerate(code):
        match = STEP_DEFINITION.match(line)
        if match:
            defined.add(match.group(1))
        header = TEST_HEADER.match(line)
        if not header:
            continue
        run = PROOF_RUN.match(code[index + 1]) if index + 1 < len(code) else None
        closed = BLOCK_END.match(code[index + 2]) if index + 2 < len(code) else None
        if not run or not closed:
            continue
        run_lines.add(index + 1)
        registrations.append((index + 2, header.group(1), run.group(1)))

    for index, line in enumerate(code):
        if PROOF_RUN_MENTION.search(line) and index not in run_lines:
            complaints.append(
                "  %s:%d runs a proof outside the shape this gate reads, so its build argument "
                "cannot be checked; %s" % (path, index + 1, REGISTRATION_SHAPE))

    # The build constraint is read from the raw source, because blanking has already removed it
    # from the scrubbed copy. A constrained file decides outside itself whether it compiles, so
    # the gate would otherwise credit registrations Go never builds.
    #
    # Only the header is read, because that is the only place Go reads one: a constraint is taken
    # from the lines before the package clause, so the same text lower down is ordinary content,
    # most often a line inside a raw string. Scanning the whole file on raw lines refused proofs Go
    # builds happily. The blanked copy locates the package clause at no cost, since every header
    # comment is already whitespace there, which leaves the first line carrying anything at all as
    # the first line of real code.
    header_end = len(raw)
    for index, line in enumerate(code):
        if line.strip():
            header_end = index
            break
    for number, line in enumerate(raw[:header_end], 1):
        if GO_BUILD_CONSTRAINT.match(line):
            complaints.append(
                "  %s:%d carries a build constraint, so whether Go ever compiles these "
                "registrations is decided outside the file; a proof file needs no build "
                "constraint, so remove it" % (path, number))

    return defined, registrations, complaints


def proof_registrations(proof_dir):
    """Return the proof's step functions, its registered ones, and every complaint about them.

    A step function is registered when the Test of the same title builds with it. Requiring the
    titles to agree is what makes the mapping readable from names alone, and it catches a crossed
    pair — a Test building with some other step — which a gate that only looked for a `step<Title>`
    argument would pass.
    """
    defined = set()
    registered = set()
    complaints = []
    if not os.path.isdir(proof_dir):
        return defined, registered, complaints
    for entry in sorted(os.listdir(proof_dir)):
        if not entry.endswith('.go'):
            continue
        path = os.path.join(proof_dir, entry)
        found, registrations, file_complaints = scan_proof_file(path)
        defined |= found
        complaints.extend(file_complaints)
        for number, test, build in registrations:
            if not entry.endswith('_test.go'):
                complaints.append(
                    "  %s:%d registers %s, but `go test` only runs tests in a _test.go file, so "
                    "nothing here ever builds it" % (path, number, build))
                continue
            expected = 'step' + test[len('Test'):]
            if build == expected:
                registered.add(build)
            elif STEP_NAME.fullmatch(build):
                complaints.append(
                    "  %s:%d registers %s inside %s, but a step is registered by the Test of its "
                    "own title, so this build belongs in Test%s"
                    % (path, number, build, test, build[len('step'):]))
            else:
                complaints.append(
                    "  %s:%d registers %s as a proof run's build argument, but that argument "
                    "must be a step<Title> function so a step can claim it"
                    % (path, number, build))
    return defined, registered, complaints


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
    functions, registered, registration_problems = proof_registrations(proof_dir)
    problems.extend(registration_problems)
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
                problems.append("  %s names proof function %s, which Test%s does not build with; "
                                "%s" % (sid, fn, fn[len('step'):], REGISTRATION_SHAPE))
            claimed.add(fn)
    for fn in sorted(functions - claimed):
        problems.append("  proof function %s is not claimed by any step" % fn)
    for fn in sorted(functions - registered - claimed):
        problems.append("  proof function %s is defined but no Test%s builds with it; %s"
                        % (fn, fn[len('step'):], REGISTRATION_SHAPE))

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
