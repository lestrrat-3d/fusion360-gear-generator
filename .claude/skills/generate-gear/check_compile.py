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
     an unchecked one. A call to a run method the harness does not declare is reported wherever it
     is written, registration or not, because Go calls that `undefined` wherever it sits.
     Registrations are generated from the step list rather than drafted, so each `[GO]` step also
     carries a `proof-run` annotation naming the run its registration is built from, and an
     annotation and a registration that disagree are a stale generated file.
  3. API CALLS ARE REAL. Every Fusion call the step list names exists in the API database the
     `fusion` plugin ships. Catches a spec that names a method Fusion does not have. Each
     unresolved call carries a second `fault:` line saying whether a prose source names it: a
     word-boundary search of the provenance input set decides `fault: prose` (a spec file or the
     playbook wrote the name, so the fix belongs there) from `fault: draft` (only the step list
     under test wrote it, so the failure goes back to the drafter). That is the compile-gear
     fault table's own discriminator, so the reader no longer has to grep the spec by hand.
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

Run from the repo root. Exit 0 = OK, 1 = BLOCKING, 2 = an input is missing or unreadable.
"""
import collections
import json
import os
import platform
import re
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.abspath(os.path.join(HERE, os.pardir, os.pardir, os.pardir))
sys.path.insert(0, HERE)
import fusion_api  # noqa: E402  (sibling module; sys.path is fixed up just above)
from call_parser import call_shapes  # noqa: E402
from provenance import (  # noqa: E402  (sibling module; sys.path is fixed up just above)
    DOCUMENT_REF, STAMPED_ROW, ProvenanceError, blob_hash, provenance_inputs, read,
    referenced_documents)

PATH_REF = r'[\w./-]+\.(?:md|go|py|json|sh)'
PATH_TOKEN = re.compile(r'`(%s)`' % PATH_REF)

INLINE_CITATION = re.compile(r'`(%s):(\d+)(?:\s*[-\u2013]\s*(\d+))?`' % PATH_REF)
LINE_RANGE = re.compile(r'\bL(\d+)(?:\s*[-\u2013]\s*(\d+))?\b')


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


def spec_namings(names, gear):
    """Map each bare call name to the spec sources that name it, as 'path:line' strings.

    The sources are the provenance input set: the gear's instructions, its fusion sidecar,
    the playbook, and the auxiliary documents those reference. The step list under test is
    not a source. A name counts wherever it appears as a whole word, call syntax or not,
    because the fault table's bar is only that the prose named it.
    """
    names = sorted(set(names))
    if not names:
        return {}
    patterns = [(name, re.compile(r'(?<!\w)%s(?!\w)' % re.escape(name))) for name in names]
    found = {name: [] for name in names}
    # One read per file, however many names are failing, and one hit per file: the annotation
    # points the reader at a source to fix, not at every mention inside it.
    for path in sorted(provenance_inputs(gear)):
        lines = read(path).splitlines()
        for name, pattern in patterns:
            for number, line in enumerate(lines, 1):
                if pattern.search(line):
                    found[name].append('%s:%d' % (path, number))
                    break
    return found


def fault_note(namings):
    """The second line under an unresolved call, blaming the prose or the draft."""
    if namings:
        return ("\n    fault: prose — named in %s — fix the spec, not the draft"
                % ', '.join(namings))
    return "\n    fault: draft — no spec source names it — send the failure back to the drafter"


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
# with any run method `proof/proofkit/` or `proof/proofkit3d/` declares in place of `proofkit.Run`,
# and an assertion argument after the build where the run takes one. The methods and their
# argument counts are read from those two packages, so this comment does not list them. The build
# argument is always the third, and the run passes exactly the arguments its own method declares,
# no more and no fewer. The Test's title and the step's title are the same word, which is what
# lets a step list claim a function by name alone.
#
# The header and the closing brace are read at column 1 only, which is part of the shape rather
# than a fact about Go: Go compiles an indented one. `TEST_HEADER` and `BLOCK_END` each say why
# they hold that column, and `GO_DECLARATION_INDENT` says where an ordinary declaration may start.
#
# What this buys is that the gate never has to decide what a brace means. What it costs is that a
# run written any other way is refused rather than read. That is the safe direction: a refusal
# names the file and line and says what to write, while a misreading silently credits a step no
# test builds. `.claude/skills/generate-gear/scaffold_proof.py` writes exactly this shape, from
# the step list's `proof-run` annotations, so a refusal here means something other than the
# scaffolder wrote the registration.
#
# Comments and literals are blanked before matching, so a registration quoted inside a doc comment
# or a raw string is not mistaken for a real one. Blanking preserves newlines, so a line number in
# a complaint is a line number in the file.

# Which files Go puts in a package is decided by their names alone, and by two rules nothing in
# the file itself shows. A basename starting with `_` or `.` is invisible to `go/build`: it lands
# in neither `TestGoFiles` nor `IgnoredGoFiles`, and `go test` reports the package as having no
# test files. A basename whose trailing `_`-separated words name a GOOS, a GOARCH, or a GOOS and a
# GOARCH, with a `_test` word stripped first, carries a build constraint and is compiled only
# where those match.
#
# The gate needs both, because a proof file Go never builds cannot register anything, and reading
# `_test.go` off the end of a name credits steps whose tests never run. The name lists and the
# matching order below are transcribed from `go/build`'s `syslist.go` and `goodOSArchFile`.
GO_KNOWN_OS = frozenset((
    'aix', 'android', 'darwin', 'dragonfly', 'freebsd', 'hurd', 'illumos', 'ios', 'js', 'linux',
    'nacl', 'netbsd', 'openbsd', 'plan9', 'solaris', 'wasip1', 'windows', 'zos',
))

GO_KNOWN_ARCH = frozenset((
    '386', 'amd64', 'amd64p32', 'arm', 'armbe', 'arm64', 'arm64be', 'loong64', 'mips', 'mipsle',
    'mips64', 'mips64le', 'mips64p32', 'mips64p32le', 'ppc', 'ppc64', 'ppc64le', 'riscv',
    'riscv64', 's390', 's390x', 'sparc', 'sparc64', 'wasm',
))

# Go's name for the machine this checker runs on. `go env` would answer directly, but running the
# toolchain is out of scope here, so the two identifiers are translated from Python's own. Only
# the platforms this repo is built on need an entry; anything else falls back to the Python name
# with a trailing release number dropped, which is already Go's spelling for the BSDs.
GOOS_BY_SYS_PLATFORM = {
    'aix': 'aix', 'cygwin': 'windows', 'darwin': 'darwin', 'linux': 'linux', 'sunos': 'solaris',
    'win32': 'windows',
}

GOARCH_BY_MACHINE = {
    '386': '386', 'aarch64': 'arm64', 'amd64': 'amd64', 'arm': 'arm', 'arm64': 'arm64',
    'armv6l': 'arm', 'armv7l': 'arm', 'i386': '386', 'i686': '386', 'loongarch64': 'loong64',
    'mips': 'mips', 'mips64': 'mips64', 'mips64le': 'mips64le', 'mipsel': 'mipsle',
    'ppc64': 'ppc64', 'ppc64le': 'ppc64le', 'riscv64': 'riscv64', 's390x': 's390x',
    'sparc64': 'sparc64', 'x86': '386', 'x86_64': 'amd64',
}


def current_goos():
    """Go's name for this operating system."""
    name = sys.platform
    if name in GOOS_BY_SYS_PLATFORM:
        return GOOS_BY_SYS_PLATFORM[name]
    return name.rstrip('0123456789')


def current_goarch():
    """Go's name for this processor architecture."""
    machine = platform.machine().lower()
    return GOARCH_BY_MACHINE.get(machine, machine)


GOOS = current_goos()
GOARCH = current_goarch()


def go_platform_tag_matches(tag, goos, goarch):
    """Whether one GOOS or GOARCH filename suffix is satisfied here.

    The three aliases are Go's own: an `android` build also takes `linux` files, `illumos` takes
    `solaris`, and `ios` takes `darwin`.
    """
    if tag in (goos, goarch):
        return True
    return (goos, tag) in (('android', 'linux'), ('illumos', 'solaris'), ('ios', 'darwin'))


def go_ignores_file(name, goos=None, goarch=None):
    """Why Go never compiles this basename here, or None when it does.

    `unix` is a build tag and never a filename suffix, the name match is case-sensitive, and a
    basename that is only a GOOS with nothing before it carries no constraint at all, because Go
    reads the suffix from the first `_` onwards. All three follow from `goodOSArchFile` and all
    three are names Go compiles.
    """
    goos = GOOS if goos is None else goos
    goarch = GOARCH if goarch is None else goarch
    if name.startswith('_') or name.startswith('.'):
        return "a basename starting with `%s` is invisible to the Go build" % name[0]
    stem = name.split('.', 1)[0]
    cut = stem.find('_')
    if cut < 0:
        return None
    words = stem[cut:].split('_')
    if words[-1] == 'test':
        words = words[:-1]
    if len(words) >= 2 and words[-2] in GO_KNOWN_OS and words[-1] in GO_KNOWN_ARCH:
        if (go_platform_tag_matches(words[-2], goos, goarch)
                and go_platform_tag_matches(words[-1], goos, goarch)):
            return None
        return ("a `_%s_%s` suffix builds only on %s/%s, and this is %s/%s"
                % (words[-2], words[-1], words[-2], words[-1], goos, goarch))
    if words and (words[-1] in GO_KNOWN_OS or words[-1] in GO_KNOWN_ARCH):
        if go_platform_tag_matches(words[-1], goos, goarch):
            return None
        return ("a `_%s` suffix builds only where GOOS or GOARCH is %s, and this is %s/%s"
                % (words[-1], words[-1], goos, goarch))
    return None


# The whitespace `go/build` trims off a header line, and nothing else. This set is for the
# textual pass Go makes over a file's header before the compiler scans anything, so it is the
# right one in `go_header_constraint_lines` and `PLUS_BUILD_DIRECTIVE` and the wrong one anywhere
# a Go token is being read: it holds U+000B, U+000C, U+0085, U+00A0 and the Unicode space
# separators, every one of which Go's scanner refuses outright. `GO_DECLARATION_INDENT` below is
# the set for separating tokens on a line.
#
# Go trims every header line with `bytes.TrimSpace`, which uses `unicode.IsSpace`, before it looks
# at the line at all. Python's `\s` is nearly that set but also counts U+001C to U+001F, which Go
# does not, so a header opening with one of those reads to Python as an indented comment and to Go
# as a line that does not begin with `//`. The set is written out rather than borrowed. A
# non-breaking space is in it, which is why `\xa0//go:build ignore` is a constraint Go honours.
GO_SPACE = ('\t\n\v\f\r \u0085\u00a0\u1680'
            '\u2000\u2001\u2002\u2003\u2004\u2005\u2006\u2007\u2008\u2009\u200a'
            '\u2028\u2029\u202f\u205f\u3000')

GO_BUILD_DIRECTIVE = '//go:build'

GO_BYTE_ORDER_MARK = '\ufeff'


def go_reads_build_constraint(line):
    """Whether Go reads this trimmed header line as a `//go:build` constraint.

    Transcribed from `go/build`'s `isGoBuildComment`. Two rules, and the gate has to hold both or
    it disagrees with Go about a spelling.

    Nothing may sit between the slashes and the directive, so `// go:build is discussed here` is
    ordinary prose and its file is built.

    The directive must be followed by whitespace or the end of the line. `//go:build!ignore` and
    `//go:build/ignore` are prose by that rule, and `go list` reports both in `GoFiles`; ending
    the directive at a word boundary instead refused files Go compiles, at line 1, for a `!` or a
    `/`. Bare `//go:build`, and `//go:build` followed only by spaces or tabs, are constraints Go
    cannot parse — it reports the file in `InvalidGoFiles` with "unexpected end of expression" —
    so they stay refused.
    """
    if not line.startswith(GO_BUILD_DIRECTIVE):
        return False
    rest = line[len(GO_BUILD_DIRECTIVE):]
    return not rest or rest[0] in GO_SPACE


# Every `+build` spelling is refused, and that is this gate's decision rather than Go's rule. Go
# reads the legacy form only when the directive is followed by whitespace or the end of the line,
# and only when the comment run holding it is closed by a blank line, so `// +build!ignore` and a
# `// +build ignore` written directly above the package clause are both in `GoFiles`. The gate
# refuses them anyway. A proof file needs no build constraint in any form, so an over-refusal here
# costs one message and a rewritten header, while matching Go would add two more rules to keep
# right for a form nothing in this repository should be writing. The `//go:build` side above is
# matched exactly instead, because there a wider gate refuses files Go builds and buys nothing.
PLUS_BUILD_DIRECTIVE = re.compile(r'//[%s]*\+build' % re.escape(GO_SPACE))


def go_style_build_comment(line):
    """Whether this trimmed header line is refused as a build constraint."""
    return go_reads_build_constraint(line) or bool(PLUS_BUILD_DIRECTIVE.match(line))


# Where a package-level declaration may start, for every pattern below that reads one. Go's layout
# is free and nothing in this repository holds a `.go` file to `gofmt`: `proof/run.sh` runs only
# `go work init/edit` and `go test ./...`, and neither `.github/workflows/3d-proof.yml` nor
# `sketch-bench.yml` has a gofmt, vet or lint step. An indented declaration therefore reaches the
# harness sources and the proof files alike, and Go compiles it, exports it and runs the test built
# on it.
#
# Allowing the indentation stays exact rather than becoming a guess. Go does not permit a named
# function declaration inside a function body — an indented `func Inner(x int) {}` there is
# `syntax error: unexpected name Inner, expected (` — so `func` followed by a name can only be a
# package-level declaration, whatever column it sits in. A function literal has no name, so a
# `func(` opening a continuation line never matches. A method is `func (r *T) Name…`, whose
# parenthesis follows `func` directly, and the name each pattern requires already excludes it.
# Comments and string literals are blanked to spaces by `strip_go_comments_and_literals` before
# every scan these patterns run in, so a `func` quoted in either cannot match.
#
# The indentation set is Go's rather than Python's `\s`. Go's scanner skips space, tab, carriage
# return and newline between tokens and nothing else, and a file indented with anything wider is
# one Go refuses outright: the compiler reports a leading U+00A0 as `invalid character U+00A0 in
# identifier`, and `go test` reports the same file `illegal character U+00A0`, with U+000B and
# U+000C refused the same way. Reading those would find a declaration in a file that never
# compiles, which is what every rule on this path exists to stop. Newline is left out because a
# line is what these anchor to.
#
# Which pattern uses this is a decision per pattern rather than a blanket widening. The two lines
# that carry the declared registration shape — `TEST_HEADER` and `BLOCK_END` — stay anchored at
# column 1 on purpose, and each says so at its own site.
#
# The same set separates one token from the next, which is the other job it has here. Go's scanner
# makes no distinction between the run of whitespace in front of `func` and the run between `func`
# and the name, so every gap in every pattern below is this set and never Python's `\s`. `GO_SPACE`
# above is a different set for a different job and would be wrong in either position.
GO_DECLARATION_INDENT = ' \t\r'


def go_word_exclusion_ranges():
    """Code point ranges Python's `\\w` matches that Go's identifier rule refuses.

    `\\w` is exactly the alphanumeric characters plus `_`, and Go's rule is a Unicode letter or `_`
    to start, then letters, Unicode decimal digits or `_`. Go's two halves are `str.isalpha` (the
    letter categories) and `str.isdecimal` (the decimal digits), so what `\\w` has and Go does not
    is what is alphanumeric to Python and neither of those — the numeral-like characters such as
    U+00B2, U+00BD and U+2160, which Go reports as `invalid character U+00B2 in identifier` and
    `go test` reports as `illegal character U+00B2`.

    The set is derived rather than typed out, for the reason `PROOF_RUN_PACKAGES` gives about the
    run table: a list of code points copied into Python is a fact about Unicode that nothing keeps
    honest. `test_check_compile.py` holds the derived classes against the Go toolchain's own
    tables, so a release that moved the boundary fails there.
    """
    ranges = []
    for code in range(sys.maxunicode + 1):
        character = chr(code)
        if not character.isalnum() or character.isalpha() or character.isdecimal():
            continue
        if ranges and ranges[-1][1] == code - 1:
            ranges[-1][1] = code
        else:
            ranges.append([code, code])
    return [(first, last) for first, last in ranges]


def character_class_body(ranges):
    """Those ranges as the inside of a character class, every code point written as an escape."""
    def escaped(code):
        return '\\u%04x' % code if code <= 0xFFFF else '\\U%08x' % code
    return ''.join(
        escaped(first) if first == last else '%s-%s' % (escaped(first), escaped(last))
        for first, last in ranges)


GO_WORD_EXCLUSION = character_class_body(go_word_exclusion_ranges())

# Go's identifier rule as two classes: `\w` with the numerals Go refuses taken out, and without
# the decimal digits as well for the first rune, since a Go identifier cannot open with one. Go
# identifiers do include Unicode letters, so `stepPrüfung` is a name a proof may declare and these
# classes read it.
GO_IDENTIFIER_START = '[^\\W\\d%s]' % GO_WORD_EXCLUSION
GO_IDENTIFIER_PART = '[^\\W%s]' % GO_WORD_EXCLUSION
GO_IDENTIFIER = '%s%s*' % (GO_IDENTIFIER_START, GO_IDENTIFIER_PART)

# The pieces every pattern that reads Go source is written from, so no pattern below spells a
# character class of its own:
#
#   sp, sp1     a run of token separation, optional and required — `GO_DECLARATION_INDENT`
#   part        one rune a Go identifier may carry after its first
#   name        a whole Go identifier
#   qualified   a Go name, optionally package qualified: `proofkit3d.RequireSolid`
#   left        the left edge of a Go token: the rune before it is not one an identifier carries
#   no_word_left, no_word_right
#               the edges of a token that has to stand alone: nothing Python counts as a word
#               character touches it on that side
#
# `left` is a lookbehind rather than `\b` because `\b` is defined by Python's `\w`, so a
# neighbouring U+00B2 counts to Python as part of a word and to Go does not, and the match is
# suppressed. That is the one place on this path where Python's class is too narrow rather than
# too wide, and a match suppressed there silences a diagnostic instead of over-accepting.
#
# `no_word_left` and `no_word_right` are the opposite edge rule, and they are wider than Go's own
# on purpose. Which of the two a pattern takes follows from what its match does. `left` belongs on
# a pattern whose match raises a complaint, where a neighbouring rune Go refuses must not silence
# it. A pattern whose match grants credit needs the reverse: a rune Go refuses glued to the token
# means the token as written is not a Go name at all, so the whole match has to be suppressed
# rather than truncated to the valid prefix it starts or ends with. Python's `\w` is the class
# that spans both the identifier runes and the ones Go refuses, so it is the edge that suppresses.
# `STEP_NAME_CLAIM` is the only pattern here whose match grants credit, and it is the only one
# written from this pair.
GO_TOKENS = {
    'sp': '[%s]*' % re.escape(GO_DECLARATION_INDENT),
    'sp1': '[%s]+' % re.escape(GO_DECLARATION_INDENT),
    'part': GO_IDENTIFIER_PART,
    'name': GO_IDENTIFIER,
    'qualified': '%s(?:\\.%s)*' % (GO_IDENTIFIER, GO_IDENTIFIER),
    'left': '(?<!%s)' % GO_IDENTIFIER_PART,
    'no_word_left': '(?<!\\w)',
    'no_word_right': '(?!\\w)',
}

# The run of separation between two tokens, for stripping it out of a captured qualifier.
GO_TOKEN_SEPARATION = re.compile('%(sp1)s' % GO_TOKENS)

# A step is read only from a function declaration. Go would also accept `var stepFoo = func(...)`,
# and that form is deliberately not recognised: the drafting contract in
# `.claude/skills/compile-gear/prompt.md` is one function per step, and a gate that reads both forms
# has two shapes to keep right instead of one. What the refusal must not do is call a
# variable-bound step undefined, so the complaint says the step has to be declared as a function.
#
# The declaration is read wherever it starts on its line. This pattern is not part of the declared
# registration shape; it only discovers which steps a proof directory defines. Go compiles an
# indented `func stepFoo`, `go test` runs the registration built on it, and anchoring at column 1
# hid the definition and reported the step as one the proof directory does not declare as a
# function — a complaint that sends the reader to a proof where the step is already written as a
# function, and hides the whitespace that is the real difference.
STEP_DEFINITION = re.compile(
    r'^%(sp)sfunc%(sp1)s(step[A-Z]%(part)s*)%(sp)s[\[(]' % GO_TOKENS)

# The header is anchored at column 1, and that is a decision rather than an oversight. It is the
# first of the three lines of the registration shape `.claude/skills/generate-gear/scaffold_proof.py`
# writes, and that shape is a contract: the scaffolder emits the header at column 1,
# `gofmt` writes it there, and holding the contract is what this pattern is for. Go does run an indented
# `Test`, so the refusal is this gate holding its own shape rather than following Go, and it is
# intended. It is also loud rather than silent: `TEST_FUNCTION` below reads the indented header and
# names it as the header problem, at its own line, instead of leaving the run beneath it to be
# blamed.
TEST_HEADER = re.compile(
    r'^func%(sp1)s(Test[A-Z]%(part)s*)%(sp)s\(%(sp)s%(name)s%(sp1)s\*testing\.T%(sp)s\)'
    r'%(sp)s\{%(sp)s$' % GO_TOKENS)

# `go test` runs any function named `Test` followed by a rune that is not a lowercase letter, and
# it does not care how the testing package is spelled, so `Test_Foo`, `Test1x` and a header written
# against an aliased import all run. The header shape above is narrower on purpose. This wider
# pattern exists so a Test that Go runs but this gate cannot read is named as the header problem it
# is: the run inside such a Test is often perfectly formed, and reporting the run would send the
# drafter to the wrong line.
#
# So this one is read wherever it starts on its line, unlike the shape pattern above. An indented
# header is precisely a header Go runs and the shape does not read, which is the case this
# diagnostic exists for; while it was anchored too, the diagnostic went silent for exactly that
# file and the complaint fell back to the run beneath it.
TEST_FUNCTION = re.compile(
    r'^%(sp)sfunc%(sp1)s(Test%(part)s*)%(sp)s\(' % GO_TOKENS)

TEST_HEADER_SHAPE = ('a proof Test is headed `func Test<Title>(t *testing.T) {`, with the testing '
                     'package named in full')


def go_runs_test(name):
    """Whether `go test` would run a function of this name.

    Go's rule is `Test` followed by a rune that is not a lowercase letter, with bare `Test`
    allowed. Transcribed from the `testing` package's own `isTest`.
    """
    if not name.startswith('Test'):
        return False
    rest = name[len('Test'):]
    return not rest or not rest[0].islower()


# The build slot is a bare name, because it has to be `step<Title>` for a step to claim it. Every
# other argument may be qualified, so a gate like `proofkit3d.RequireSolid` reads as one argument.
# The suffix is tied to its namespace, because the run methods that exist are the ones the two
# harness packages declare: `proofkit` declares only `Run`, and the `Solid` and `WithGate` variants
# belong to `proofkit3d` alone. Letting the namespace and the suffix vary independently would
# credit a registration built on `proofkit.RunSolid`, which no package defines and Go cannot
# compile.
#
# The argument count is tied to the method for the same reason. A tail of any length accepted a
# three-argument `proofkit3d.RunWithGate` and a five-argument `proofkit.Run`, neither of which Go
# compiles, so the shape read a step as registered by a call that never builds.
#
# Neither the method names nor the counts are written out here. They are derived from the harness
# sources below, because a table typed into Python is a copy of a Go fact that nothing keeps
# honest: a new run method, or a changed signature, would leave it wrong with every test still
# green. Reading the `.go` files as text adds no toolchain dependency.
PROOF_RUN_PACKAGES = ('proofkit', 'proofkit3d')

# The declaration is read wherever it starts on its line, under `GO_DECLARATION_INDENT` above.
# Anchoring at column 1 meant one stray tab in front of `func RunSolid` lost the method while
# `go build` and `go vet` stayed at exit 0, and every sound registration on it was then reported
# as a call no harness package declares, sending the reader to the proof for a defect in the
# harness.
GO_RUN_DECLARATION = re.compile(
    r'^%(sp)sfunc%(sp1)s(Run%(part)s*)%(sp)s\(' % GO_TOKENS, re.M)


def go_balanced_span(text, start):
    """The text between the bracket at `start` and its match, or None when unbalanced."""
    depth = 0
    for index in range(start, len(text)):
        if text[index] in '([{':
            depth += 1
        elif text[index] in ')]}':
            depth -= 1
            if depth == 0:
                return text[start + 1:index]
    return None


def go_parameter_count(parameters):
    """The number of parameters one Go parameter list declares.

    Go lets parameters share a type, so `a, b Case` is two parameters and the count is of
    top-level commas. A comma nested inside brackets belongs to a type — a func type's own list,
    a composite literal, a generic instantiation — and is not one of them.
    """
    if not parameters.strip():
        return 0
    depth = 0
    commas = 0
    for character in parameters:
        if character in '([{':
            depth += 1
        elif character in ')]}':
            depth -= 1
        elif character == ',' and depth == 0:
            commas += 1
    return commas + 1


def go_run_arities(package, path):
    """Map `<package>.<Run…>` to its declared parameter count, for one harness source file.

    The text comes from `harness_source`, so a file Go will not read, or one whose header decides
    outside the file whether Go compiles it, is a named error rather than a silent contribution.

    Comments and literals are blanked first, so a signature quoted in a doc comment is not read
    as a declaration.
    """
    table = {}
    code = strip_go_comments_and_literals(harness_source(path))
    for match in GO_RUN_DECLARATION.finditer(code):
        parameters = go_balanced_span(code, match.end() - 1)
        if parameters is None:
            continue
        table['%s.%s' % (package, match.group(1))] = go_parameter_count(parameters)
    return table


def derive_proof_run_arguments(root=REPO_ROOT):
    """Every run method the harness packages declare, with the arguments each one takes.

    The harness sources are held to the same Go rules as the proof files, because the derived set
    is only as true as the reading it comes from: a file read here but not compiled by Go invents
    a method that does not exist, and a file compiled by Go but not read here loses one that does.
    `harness_source` states which rule is a skip, which is a named error, and why each is which.

    `root` is the tree the harness is read from. The default is the repository this checker ships
    in; a test passes its own so these rules can be exercised on fixtures.
    """
    table = {}
    for package in PROOF_RUN_PACKAGES:
        directory = os.path.join(root, 'proof', package)
        if not os.path.isdir(directory):
            continue
        for entry in sorted(os.listdir(directory)):
            path = os.path.join(directory, entry)
            if not entry.endswith('.go') or entry.endswith('_test.go'):
                continue
            if not os.path.isfile(path):
                continue
            if go_ignores_file(entry) is not None:
                continue
            table.update(go_run_arities(package, path))
    if not table:
        raise RuntimeError(
            'check_compile: no proof run methods found under %s; the registration shape is '
            'derived from those packages and cannot be built without them'
            % ', '.join(os.path.join('proof', name) for name in PROOF_RUN_PACKAGES))
    return table


def run_methods_by_package(methods):
    """Group derived `<package>.<Run…>` names by the package that declares each one."""
    grouped = {}
    for qualified in methods:
        package, name = qualified.split('.', 1)
        grouped.setdefault(package, set()).add(name)
    return grouped


def go_name_alternation(names):
    """A regex alternation over names, longest first so `RunSolid` is never cut short to `Run`."""
    return '|'.join(re.escape(name) for name in sorted(names, key=lambda name: (-len(name), name)))


# The closing brace is anchored at column 1 for the reason `TEST_HEADER` gives: it is the third
# line of the declared registration shape, the scaffolder writes it at column 1, and `gofmt`
# writes it there. Go compiles an indented `}`, so refusing one is this gate holding the contract
# rather than following Go, and the refusal is intended. It is loud: the three lines are reported
# as a run written outside the shape, and the shape quoted in that complaint names all three of
# them.
BLOCK_END = re.compile(r'^\}%(sp)s$' % GO_TOKENS)

# The build argument a registration passed, tested against the step naming rule. It is read from
# Go source and so follows Go's identifier rule.
STEP_NAME = re.compile(r'step[A-Z]%(part)s*' % GO_TOKENS)

# The step names one step's Markdown body claims. The subject is prose, but what the token names
# is a Go identifier compared against the names read from the proof sources, so the body of the
# name follows Go's rule too: a name Go cannot have is a name no proof can declare, and reading a
# wider one compares a name that cannot exist against a set that cannot hold it. Both sides read
# the body with Python's `\w`, which agreed with itself and credited a step in a proof `go test`
# refuses.
#
# The edges are the other pair, `no_word_left` and `no_word_right`, and this is the site that pair
# was written for. This is the one pattern here whose match grants credit rather than raising a
# complaint, so it is the one where a wider match is a false OK rather than a louder report. With
# no right edge the match stopped at the first rune outside the identifier class and handed back
# the valid prefix, so a step claiming `stepOne²` was credited to a proof declaring `stepOne` and
# the gate printed OK; with only `left` on the other side, `²stepOne` did the same. An edge
# symmetric with `left` does not close either hole, because U+00B2 is not an identifier rune and
# such a lookbehind or lookahead succeeds on it. The edge has to be the wider class, so that a
# neighbour Go refuses suppresses the claim instead of trimming it.
STEP_NAME_CLAIM = re.compile(
    r'%(no_word_left)s(step[A-Z]%(part)s*)%(no_word_right)s' % GO_TOKENS)

REGISTRATION_SHAPE = ('a registration is three lines and nothing else: '
                      '`func Test<Title>(t *testing.T) {`, one proof run whose third argument is '
                      '`step<Title>` and which passes exactly the arguments its method declares, '
                      'then `}`')


GO_BYTE_ORDER_MARK_BYTES = GO_BYTE_ORDER_MARK.encode('utf-8')

# The UTF-16 marks, in both byte orders. Go names this one specially — `illegal UTF-8 encoding
# (got UTF-16)` — because it is the byte pattern a file saved as UTF-16 by an editor opens with,
# and telling the drafter "not UTF-8" without naming the encoding sends them looking for a stray
# character instead of at the save dialog.
GO_UTF16_MARKS = (b'\xff\xfe', b'\xfe\xff')

# What each refused byte pattern is, in Go's own words, and what removing it means. Go reports
# `unexpected NUL in input` from the reader `go/build` uses for a file's header and `invalid NUL
# character` from the compiler for one further down; either way the file is refused, so the gate
# does not distinguish them.
GO_UTF16_REFUSAL = ('opens with a UTF-16 byte order mark, which Go reports as `illegal UTF-8 '
                    'encoding (got UTF-16)`', 'as UTF-8 rather than UTF-16')
GO_NOT_UTF8_REFUSAL = ('holds bytes that are not UTF-8, which Go reports as `illegal UTF-8 '
                       'encoding`', 'as UTF-8')
GO_NUL_REFUSAL = ('holds a NUL byte, which Go reports as `unexpected NUL in input`',
                  'without that byte')
GO_STRAY_MARK_REFUSAL = ('holds a byte order mark below the first character, which Go reports as '
                         '`illegal byte order mark`', 'without that mark')


def go_refused_bytes(data):
    """The first byte pattern in `data` that Go refuses to read, or None when there is none.

    Returns `(offset, reason, remedy)`, the offset being the byte the complaint points at.

    Go refuses to compile a source file whose bytes it cannot read, and a refused file registers
    nothing, so crediting one is a false pass of this gate. Four patterns do it, and each is
    checked here rather than guessed at:

    - `FF FE` or `FE FF` at the very start, which is a UTF-16 mark.
    - Any other byte sequence that is not UTF-8.
    - A NUL byte anywhere.
    - `EF BB BF` anywhere other than the first character.

    The last two decode as perfectly good UTF-8, so a strict decode alone catches neither, and
    both were credited with no complaint before they were looked for by hand.

    One leading `EF BB BF` is skipped before the scan, because Go strips that one; the search for
    a stray mark then starts below it, so a second mark is found at the offset Go names.

    Which pattern is reported when a file holds more than one is the earliest in the file, except
    that a leading UTF-16 mark is named first: it is also invalid UTF-8, and the specific message
    is the useful one.
    """
    if data.startswith(GO_UTF16_MARKS):
        return (0,) + GO_UTF16_REFUSAL
    start = len(GO_BYTE_ORDER_MARK_BYTES) if data.startswith(GO_BYTE_ORDER_MARK_BYTES) else 0
    found = []
    nul = data.find(b'\x00', start)
    if nul >= 0:
        found.append((nul,) + GO_NUL_REFUSAL)
    mark = data.find(GO_BYTE_ORDER_MARK_BYTES, start)
    if mark >= 0:
        found.append((mark,) + GO_STRAY_MARK_REFUSAL)
    try:
        data[start:].decode('utf-8')
    except UnicodeDecodeError as bad:
        found.append((start + bad.start,) + GO_NOT_UTF8_REFUSAL)
    if not found:
        return None
    return min(found, key=lambda refusal: refusal[0])


def go_byte_position(data, offset):
    """The 1-based line and column Go names for a byte offset, counted in bytes.

    Go counts a column in bytes, not in characters, so counting them here the same way makes the
    position the gate prints the one the compiler prints. Lines are cut at `\\n`, which is the
    only line ending Go recognises. The count runs over bytes because a file refused for its bytes
    may have no text form to count over at all.
    """
    line = data.count(b'\n', 0, offset) + 1
    column = offset - (data.rfind(b'\n', 0, offset) + 1) + 1
    return line, column


def go_source(path):
    """A `.go` file's text as Go reads it, or a complaint saying why Go will not read it.

    Returns `(text, complaint)`, exactly one of which is None. Nothing here raises: every byte
    pattern Go refuses becomes a complaint the drafter can act on, because a traceback fails the
    whole run without naming the file.

    Go strips a UTF-8 byte order mark only when it is the first code point in the file, and only
    one of them, before it looks for a build constraint. So `EF BB BF` above a `//go:build ignore`
    line leaves a constraint Go honours, and `go list` reports the file in `IgnoredGoFiles`.
    Reading the bytes as they sit left the line beginning with U+FEFF, which no `//` match
    reaches, and the gate credited registrations Go never compiles.

    The mark is removed as a character, not as a line, so every line number below it is the one
    Go, an editor and this gate's complaints all name.

    A second mark, one anywhere below the first character, a NUL byte, and bytes that are not
    UTF-8 are all refusals rather than text to scan, because `go build` fails on each of them and
    a file Go never compiles registers nothing. `go_refused_bytes` is the whole list and states
    what Go does with each. Reading such a file as text — with `errors='replace'` for the
    undecodable ones — credited its registrations and, for a UTF-16 mark, also hid the build
    constraint under it, because the two replacement characters pushed the line off its `//`
    start.
    """
    with open(path, 'rb') as fh:
        data = fh.read()
    refused = go_refused_bytes(data)
    if refused is not None:
        offset, reason, remedy = refused
        line, column = go_byte_position(data, offset)
        return None, (
            "  %s:%d:%d %s, so Go refuses the file: `go test` never compiles it, and nothing it "
            "registers is ever built; write the file %s" % (path, line, column, reason, remedy))
    text = data.decode('utf-8')
    if text.startswith(GO_BYTE_ORDER_MARK):
        text = text[len(GO_BYTE_ORDER_MARK):]
    return text, None


def go_header_constraint_lines(lines):
    """The 1-based numbers of the lines this gate refuses as a build constraint.

    Transcribed from `go/build`'s `parseFileHeader`. Go walks the file from the top tracking
    whether it is inside a `/* */` comment, and stops at the first text that is neither a comment
    nor blank — the package clause, in a proof file. A constraint is taken only from a `//` line
    comment outside a block comment, which is why the same text quoted inside a leading `/* */`
    note is content, and why `/* note */ //go:build ignore` written on one line is content too:
    the line does not begin with the directive, and Go never revisits it. Text below the package
    clause, most often a line inside a raw string, is never reached at all.

    Which trimmed lines are refused is `go_style_build_comment`: Go's `//go:build` rule exactly,
    and every `+build` spelling by this gate's own decision.
    """
    numbers = []
    in_block_comment = False
    for number, source in enumerate(lines, 1):
        line = source.strip(GO_SPACE)
        if not in_block_comment and go_style_build_comment(line):
            numbers.append(number)
        while line:
            if in_block_comment:
                end = line.find('*/')
                if end < 0:
                    break
                in_block_comment = False
                line = line[end + 2:].strip(GO_SPACE)
                continue
            if line.startswith('//'):
                break
            if line.startswith('/*'):
                in_block_comment = True
                line = line[2:].strip(GO_SPACE)
                continue
            return numbers
    return numbers


# The harness is read under Go's rules too, and which rule is a skip and which is an error is
# decided here rather than by whichever reader was nearest.
#
# Go's filename rule carries over unchanged, and a file it excludes is skipped exactly as Go skips
# it. The name is the whole reason, it is visible in a directory listing, and the method really
# does not exist here: `go vet` answers a call to one with `undefined: proofkit.RunWindows`, which
# is what the undeclared-method complaint below says at the call site.
#
# A build constraint is the opposite case and is a named error rather than a skip. Whether Go
# compiles a constrained file is settled outside the file, so the derived set would depend on
# where this gate runs. Skipping such a file would drop a method Go does build under a satisfied
# constraint — `//go:build linux` read on a linux builder — and every sound registration on that
# method would then be complained about as a call the harness does not declare, sending the reader
# to the proof for a defect that is in the harness. So the harness carries no build constraint at
# all, and a file that carries one is named as the harness problem it is.
#
# `go_header_constraint_lines` is the detector for that, and the breadth that makes it wrong for a
# skip is what makes it right here: it refuses every `+build` spelling as well as the `//go:build`
# ones Go reads, which is over-refusal against Go and is exactly the rule "no constraint in any
# spelling". On this path it must stay a detector and never become a skip.
def harness_source(path):
    """One harness source's text, or a named error saying why no run method can be read from it.

    Unlike the proof-file readers, this raises. A proof file the gate cannot read is one drafted
    artifact among many and becomes a complaint about that file, while a harness the gate cannot
    read leaves every registration in every proof checked against a run table that is missing
    methods Go has — a wrong answer everywhere rather than a complaint in one place.
    """
    text, refused = go_source(path)
    if refused is not None:
        raise RuntimeError(
            'check_compile: the harness source %s is one Go will not read, so the run methods it '
            'declares cannot be derived: %s' % (path, refused.strip()))
    numbers = go_header_constraint_lines(text.split('\n'))
    if numbers:
        raise RuntimeError(
            'check_compile: %s:%d carries a build constraint, so which run methods the harness '
            'declares would be decided outside the file and would depend on where this gate runs; '
            'a harness source must carry no build constraint, so remove it'
            % (path, numbers[0]))
    return text


def proof_run_shape(arguments):
    """The registration shape, over the run methods the harness packages declare.

    The four slots are named rather than numbered, because a registration is now compared
    argument by argument against the step list's `proof-run` annotation and a positional group
    number says nothing about which argument it holds. The first slot after the method is `t`,
    which is fixed and so is not captured.
    """
    method = '|'.join(
        '%s%s\\.%s(?:%s)' % (re.escape(package), GO_TOKENS['sp'], GO_TOKENS['sp'],
                             go_name_alternation(names))
        for package, names in sorted(run_methods_by_package(arguments).items(),
                                     key=lambda item: (-len(item[0]), item[0])))
    return re.compile(
        r'^%(sp)s(?P<method>%(method)s)%(sp)s\('
        r'%(sp)s%(qualified)s%(sp)s,%(sp)s(?P<table>%(qualified)s)%(sp)s,'
        r'%(sp)s(?P<build>%(name)s)'
        r'%(sp)s(?P<extras>(?:,%(sp)s%(qualified)s%(sp)s)*)\)%(sp)s$'
        % dict(GO_TOKENS, method=method))


ProofRunShapes = collections.namedtuple('ProofRunShapes', 'arguments shape')

_PROOF_RUN_SHAPES = None


# The harness read happens on first use, not at import. Running it at import ran it before `main`
# existed, so every raise from the derivation — a harness carrying a build constraint, one holding
# a NUL byte, one `open()` cannot read, both harness directories gone — killed the command with an
# uncaught traceback at exit 1. Exit 1 is this checker's code for findings in the artifact under
# review, and the harness is not that artifact; a missing or broken input is exit 2, which `main`
# already uses for a usage error, a missing step list, a step list with no steps and an
# unavailable API database. Importing without reading anything also gives the usage check back:
# with a broken harness in the tree, running the script with no arguments printed a traceback
# instead of the usage line.
#
# One read per process is the whole answer, because the derived table is a pure function of the
# on-disk harness sources and nothing mutates them between import and `main`.
def proof_run_shapes():
    """The derived run table and the registration shape built from it, read once per process.

    Raises `RuntimeError` for a harness Go will not read and `OSError` for one this process
    cannot open. `main` catches both and reports them as the broken input they are, so no caller
    below needs to; every use site here is already downstream of that check.
    """
    global _PROOF_RUN_SHAPES
    if _PROOF_RUN_SHAPES is None:
        arguments = derive_proof_run_arguments()
        _PROOF_RUN_SHAPES = ProofRunShapes(arguments, proof_run_shape(arguments))
    return _PROOF_RUN_SHAPES


# The mention pattern is built here rather than inside the accessor above, because it is not
# derived and reads no file: both halves are literal, the package names written above and any
# `Run`-prefixed name at all.
#
# That width is deliberate, and it is what makes the pattern useful. It turns a run the shape
# refuses into a named complaint, and it is the only thing that sees a run on a method no harness
# package declares — a name outside the derived set cannot appear in the shape above, so a
# pattern built from that set alone is blind to exactly the call it exists to catch.
#
# So `proofkit.RunSolid`, a pairing neither package declares, and a plain misspelling such as
# `proofkit3d.RunTypa` are both seen, and both are reported as the undeclared method Go calls
# `undefined`.
#
# The `Run` prefix is what keeps the ordinary harness calls in a step body silent, and it rests on
# a fact about the harness rather than on a rule Go enforces: no exported harness helper other
# than the run methods has a name starting with `Run`. A test in `test_check_compile.py` holds
# that fact against the harness sources, so a helper named `Run…` fails there rather than turning
# every call to it into a false complaint here.
#
# What follows that prefix is read under Go's identifier rule, like every other name here: the
# width this pattern needs is the `Run` prefix and the package alternation, not a wider class. The
# left edge is `left` from `GO_TOKENS` rather than `\b`, and this is the site that reason was
# written for. `\b` is defined by Python's `\w`, so a character in front of the package name that
# Python counts as part of a word and Go does not suppressed the whole mention, and with it the
# only complaint that names an undeclared method. That is the complaint-side edge rule; the
# credit-side one is the pair `STEP_NAME_CLAIM` is written from, and the comment on `GO_TOKENS`
# says which match takes which.
PROOF_RUN_MENTION = re.compile(
    r'%(left)s(%(packages)s)%(sp)s\.%(sp)s(Run%(part)s*)%(sp)s\('
    % dict(GO_TOKENS, packages=go_name_alternation(PROOF_RUN_PACKAGES)))


def proof_run_arguments(match):
    """Return a matched proof run's method name and the number of arguments it passes."""
    method = GO_TOKEN_SEPARATION.sub('', match.group('method'))
    return method, 3 + match.group('extras').count(',')


def proof_run_extras(text):
    """The trailing argument names a matched proof run passes after its build, in order.

    The separation the shape allows inside a qualified name is stripped, so
    `proofkit3d . RequireSolid` and `proofkit3d.RequireSolid` are the same argument. That is the
    same normalisation `proof_run_arguments` does to the method, and it is what lets a
    registration be compared name for name against the step list's annotation, which is prose and
    carries no such spacing.
    """
    return [GO_TOKEN_SEPARATION.sub('', part) for part in text.split(',') if part.strip()]


def scan_proof_file(path):
    """Return one proof file's step definitions, registrations and complaints.

    A registration comes back as (line number, Test name, build argument, method, case table,
    extra arguments) so a complaint can say where to go and so the run can be compared with the
    `proof-run` annotation the step list carries for the same function. Complaints here are the
    ones only this file can see: a build constraint in the
    file header, a Test header Go runs but this gate cannot read, a proof run written outside the
    registration shape, a run whose argument count is not the one its method declares, and a call
    to a run method no harness package declares.

    Lines are cut at `\n` and nowhere else, because that is the only line ending Go recognises.
    Python's own line splitting is wider, ending a line at a form feed, a vertical tab and
    several more, so a header comment holding one of them was read as two lines: the tail became
    a build constraint the file does not carry, and every line number after it was off by one.

    The source arrives from `go_source`, which removes a leading byte order mark as Go does, so
    line 1 reads the same to this gate as it does to the compiler and every line number holds.
    That is also where a file Go refuses to read at all is turned into the one complaint such a
    file has: nothing below is scanned, because Go never compiles it and so registers nothing.
    """
    src, refused = go_source(path)
    if refused is not None:
        return set(), [], [refused]
    # The harness read is already done and cached by the time any proof file is scanned: `main`
    # calls the accessor behind its own guard, so a broken harness is exit 2 there rather than a
    # raise from the middle of a scan.
    runs = proof_run_shapes()
    raw = src.split('\n')
    code = strip_go_comments_and_literals(src).split('\n')
    defined = set()
    registrations = []
    complaints = []
    read_runs = set()

    for index, line in enumerate(code):
        match = STEP_DEFINITION.match(line)
        if match:
            defined.add(match.group(1))
        header = TEST_HEADER.match(line)
        if not header:
            named = TEST_FUNCTION.match(line)
            if named and go_runs_test(named.group(1)):
                complaints.append(
                    "  %s:%d heads %s in a shape this gate does not read, so the registration "
                    "under it cannot be checked; %s"
                    % (path, index + 1, named.group(1), TEST_HEADER_SHAPE))
                # The run under such a header is usually well formed, and the header is what did
                # not match, so the run is not also reported as being off the registration shape.
                if (index + 2 < len(code) and runs.shape.match(code[index + 1])
                        and BLOCK_END.match(code[index + 2])):
                    read_runs.add(index + 1)
            continue
        run = runs.shape.match(code[index + 1]) if index + 1 < len(code) else None
        closed = BLOCK_END.match(code[index + 2]) if index + 2 < len(code) else None
        if not run or not closed:
            continue
        method, passed = proof_run_arguments(run)
        declared = runs.arguments[method]
        # A run on the right method with the wrong count is named as that, rather than left to the
        # mention loop below, because "outside the shape" would send the drafter looking at the
        # three lines instead of at the one argument that is missing or spare.
        read_runs.add(index + 1)
        if passed != declared:
            complaints.append(
                "  %s:%d passes %d arguments to %s, which takes %d, so Go cannot compile this "
                "registration; pass exactly the arguments the method declares"
                % (path, index + 2, passed, method, declared))
            continue
        registrations.append((
            index + 2, header.group(1), run.group('build'), method,
            GO_TOKEN_SEPARATION.sub('', run.group('table')), proof_run_extras(run.group('extras'))))

    # A run on a method the harness does not declare is named as that, wherever it is written. It
    # is not the same defect as a run off the shape: the shape complaint sends the drafter to the
    # three lines, and here the three lines may be perfect while the method does not exist. Inside
    # a registration the two coincide, and the undeclared method is the one reported, because it
    # is the one Go fails on.
    for index, line in enumerate(code):
        undeclared = []
        off_shape = False
        for mention in PROOF_RUN_MENTION.finditer(line):
            method = '%s.%s' % (mention.group(1), mention.group(2))
            if method in runs.arguments:
                off_shape = off_shape or index not in read_runs
            elif method not in undeclared:
                undeclared.append(method)
        for method in undeclared:
            complaints.append(
                "  %s:%d calls %s, which no harness package declares, so Go cannot compile "
                "this proof; the run methods are %s"
                % (path, index + 1, method, ', '.join(sorted(runs.arguments))))
        if off_shape:
            complaints.append(
                "  %s:%d runs a proof outside the shape this gate reads, so its build argument "
                "cannot be checked; %s" % (path, index + 1, REGISTRATION_SHAPE))

    # The build constraint is read from the raw source, because the scrubbed copy has already had
    # its comments blanked. A constrained file decides outside itself whether it compiles, so the
    # gate would otherwise credit registrations Go never builds.
    #
    # Which lines those are, and where the walk stops, is `go_header_constraint_lines`, which
    # follows `go/build` rather than approximating it.
    for number in go_header_constraint_lines(raw):
        complaints.append(
            "  %s:%d carries a build constraint, so whether Go ever compiles these "
            "registrations is decided outside the file; a proof file needs no build "
            "constraint, so remove it" % (path, number))

    return defined, registrations, complaints


ProofRegistration = collections.namedtuple('ProofRegistration', 'path line method table extras')


def proof_registrations(proof_dir):
    """Return the proof's step functions, its registered ones, their detail, and every complaint.

    A step function is registered when the Test of the same title builds with it. Requiring the
    titles to agree is what makes the mapping readable from names alone, and it catches a crossed
    pair — a Test building with some other step — which a gate that only looked for a `step<Title>`
    argument would pass.

    `details` maps each registered build to the `ProofRegistration` that accepted it, so the run
    can be held against the step list's `proof-run` annotation for the same function. The first
    writer wins: two Tests of one title is Go's `redeclared in this block`, which `bash
    proof/run.sh` reports before this gate has anything useful to add.
    """
    defined = set()
    registered = set()
    details = {}
    complaints = []
    if not os.path.isdir(proof_dir):
        return defined, registered, details, complaints
    for entry in sorted(os.listdir(proof_dir)):
        if not entry.endswith('.go'):
            continue
        path = os.path.join(proof_dir, entry)
        # A directory may be named `*.go`. Go builds the package around it without complaint, and
        # reading it as a file raises, so it is passed over the way Go passes over it.
        if not os.path.isfile(path):
            continue
        unbuilt = go_ignores_file(entry)
        if unbuilt is not None:
            complaints.append(
                "  %s is never compiled by Go, because %s, so nothing it registers is ever built; "
                "rename it" % (path, unbuilt))
            continue
        found, registrations, file_complaints = scan_proof_file(path)
        defined |= found
        complaints.extend(file_complaints)
        for number, test, build, method, table, extras in registrations:
            if not entry.endswith('_test.go'):
                complaints.append(
                    "  %s:%d registers %s, but `go test` only runs tests in a _test.go file, so "
                    "nothing here ever builds it" % (path, number, build))
                continue
            expected = 'step' + test[len('Test'):]
            if build == expected:
                registered.add(build)
                details.setdefault(
                    build, ProofRegistration(path, number, method, table, extras))
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
    return defined, registered, details, complaints


# The registration file is generated, and the step list is where its inputs are written.
#
# A registration is a pure function of four things — the run method, the case table, the step
# function and the remaining arguments the method declares — and nothing else about the three
# lines is a decision. `.claude/skills/generate-gear/scaffold_proof.py` turns those four into
# `proof/<gear>/zz_registrations_test.go`, so the drafter writes no `Test` function at all.
#
# The four arrive in one directive per `[GO]` step, written as an HTML comment on its own line:
#
#     <!-- proof-run: proofkit3d.RunSolid(solidCases, stepExtrudeTooth, assertExtrudeTooth) -->
#
# An HTML comment rather than a backtick span, because `named_calls` reads only backtick spans:
# written in one, the directive would reach the Fusion-API-reality check and need a
# `check-compile: ignore` escort beside every annotation in every step list. `STEP_NAME_CLAIM`
# scans the raw body, so the annotation's `step<Title>` token doubles as the step's proof-function
# claim.
#
# The arguments are the emitted call's arguments after `t`, which the scaffolder inserts, so an
# annotation lists one fewer than the arity derived from the harness. Two patterns read the
# directive: a loose one that finds every `proof-run:` comment however it is written, and a strict
# one that parses a well-formed one. A directive the loose pattern finds and the strict pattern
# will not parse is a named complaint rather than a silent skip, which is the same decision
# `PROOF_RUN_MENTION` makes about a run written outside the registration shape.
PROOF_RUN_ANNOTATION_LOOSE = re.compile(r'<!--\s*proof-run:(.*?)-->', re.S)

# The strict grammar. The package and the method are checked against the derived run table
# afterwards rather than here, so a run on a method no harness declares is named as that instead
# of arriving as an unparseable directive. Every argument is a plain or package-qualified Go name:
# no literal, no call, no expression built in place, which is the same rule the registration shape
# holds the run's own arguments to.
PROOF_RUN_ANNOTATION = re.compile(
    r'<!--\s*proof-run:\s*(?P<package>%(name)s)\s*\.\s*(?P<method>Run%(part)s*)\s*\('
    r'\s*(?P<arguments>%(qualified)s(?:\s*,\s*%(qualified)s)*)\s*\)\s*-->' % GO_TOKENS)

# One parsed annotation: the qualified run method, the case table, the step function, and the
# remaining argument names in the order the method declares them.
Annotation = collections.namedtuple('Annotation', 'method table build extras text')

PROOF_RUN_ANNOTATION_SHAPE = ('`<!-- proof-run: <package>.<Run…>(<cases>, step<Title>[, …]) -->`')

SCAFFOLD_COMMAND = 'python3 .claude/skills/generate-gear/scaffold_proof.py'


def step_annotations(body):
    """Every `proof-run` annotation one step body carries, and every directive that will not parse.

    Fenced code blocks are blanked first, the way `named_calls` blanks them, so a step quoting an
    annotation in an example does not register anything.
    """
    text = re.sub(r'```.*?```', '', body, flags=re.S)
    found = []
    malformed = []
    for loose in PROOF_RUN_ANNOTATION_LOOSE.finditer(text):
        strict = PROOF_RUN_ANNOTATION.match(text, loose.start())
        if strict is None or strict.end() != loose.end():
            malformed.append(' '.join(loose.group(0).split()))
            continue
        arguments = [argument.strip() for argument in strict.group('arguments').split(',')]
        found.append(Annotation(
            '%s.%s' % (strict.group('package'), strict.group('method')),
            arguments[0], arguments[1] if len(arguments) > 1 else '', arguments[2:],
            ' '.join(strict.group(0).split())))
    return found, malformed


def annotation_findings(step_id, annotation, arities):
    """Everything wrong with one parsed annotation, judged against the derived run table alone.

    These are the findings the step list settles on its own, so the scaffolder and this gate reach
    them from the same code and word them the same way. What a proof file says is not consulted
    here; that comparison is `main`'s, because only it has read the proof.
    """
    problems = []
    if annotation.method not in arities:
        return ["  %s annotates a run on %s, which no harness package declares; the run methods "
                "are %s" % (step_id, annotation.method, ', '.join(sorted(arities)))]
    passed = 2 + len(annotation.extras)
    declared = arities[annotation.method]
    if passed + 1 != declared:
        problems.append("  %s annotates %s with %d arguments, but the method takes %d"
                        % (step_id, annotation.method, passed + 1, declared))
    if not STEP_NAME.fullmatch(annotation.build):
        problems.append(
            "  %s annotates %s with %s as its build argument, but that argument must be a "
            "step<Title> function so a step can claim it"
            % (step_id, annotation.method, annotation.build))
    return problems


def annotation_call(annotation_or_registration, build):
    """A run's arguments after `t`, written the way both the gate and the scaffolder print them."""
    return '%s(%s)' % (annotation_or_registration.method,
                       ', '.join([annotation_or_registration.table, build]
                                 + list(annotation_or_registration.extras)))


def step_list_annotations(steps, arities):
    """Every annotation the step list declares, in step order, with every complaint about them.

    Returns `(list of (step id, annotation, sound), complaints)`, where `sound` says the
    annotation drew no finding of its own and so is worth comparing with a proof or rendering into
    a registration. An unsound one still comes back, because it names a build the proof may
    register and a caller that dropped it would then report that registration as one no step
    annotates.

    This is the whole step-list half of the registration contract, and both readers of it run this
    one function: the gate, which then compares what comes back with the proof, and the
    scaffolder, which renders it. Two implementations of these rules would be two answers to the
    question of what the step list declares, and the file the scaffolder writes has to be the file
    the gate expects.
    """
    found = []
    complaints = []
    owner = {}
    for step_id, tag, body in steps:
        annotations, malformed = step_annotations(body)
        for text in malformed:
            complaints.append("  %s carries a proof-run annotation this gate cannot parse: %s"
                              % (step_id, text))
        if tag == 'GO' and not annotations and not malformed:
            complaints.append(
                "  %s is tagged [GO] but carries no proof-run annotation; write %s in the step "
                "body" % (step_id, PROOF_RUN_ANNOTATION_SHAPE))
        if tag == 'PROSE' and annotations:
            complaints.append(
                "  %s is tagged [PROSE] but carries a proof-run annotation; a step no harness "
                "reaches registers nothing" % step_id)
        for annotation in annotations:
            problems = annotation_findings(step_id, annotation, arities)
            complaints.extend(problems)
            if annotation.build in owner:
                complaints.append("  %s and %s both annotate %s; one step owns a registration"
                                  % (owner[annotation.build], step_id, annotation.build))
                continue
            owner[annotation.build] = step_id
            found.append((step_id, annotation, not problems))
    return found, complaints


def main(argv):
    """Run the gate, turning an unusable provenance hasher into exit 2.

    A `git hash-object` that cannot run is a broken environment, the same class of failure as an
    unreadable harness or a missing API database. It is reported here at exit 2 so a drafter is
    never handed a git problem dressed up as a fault in the step list.
    """
    try:
        return check(argv)
    except ProvenanceError as exc:
        print('check_compile: %s' % exc, file=sys.stderr)
        return 2


def check(argv):
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

    # The harness is an input to this run, not part of the artifact under review, so a harness the
    # gate cannot read is reported here at exit 2 alongside the other broken inputs rather than
    # counted as a finding at exit 1. Reading it up front also makes the exit code deterministic: the
    # accessor caches, so every scan below sees the same table and none of them can raise.
    #
    # `RuntimeError` carries the derivation's own message, which already names the file, the line
    # and what Go says about it, so it is printed as it stands. `OSError` comes from the operating
    # system — `chmod 000` on a harness source raises `PermissionError` from `open()` — and says
    # nothing about why this checker was reading the file, so it is named here.
    try:
        runs = proof_run_shapes()
    except RuntimeError as exc:
        print(exc, file=sys.stderr)
        return 2
    except OSError as exc:
        print('check_compile: the harness sources under %s cannot be read, so the run methods a '
              'registration is checked against cannot be derived: %s'
              % (', '.join(os.path.join('proof', name) for name in PROOF_RUN_PACKAGES), exc),
              file=sys.stderr)
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
    functions, registered, details, registration_problems = proof_registrations(proof_dir)
    problems.extend(registration_problems)
    claimed = set()
    for sid, tag, body in steps:
        named = set(STEP_NAME_CLAIM.findall(body))
        if tag == 'GO' and not named:
            problems.append("  %s is tagged [GO] but names no proof function" % sid)
        for fn in named:
            if fn not in functions:
                problems.append(
                    "  %s names proof function %s, which %s/ does not declare as a function; "
                    "write `func %s(...)`, since a step bound to a variable is not read"
                    % (sid, fn, proof_dir, fn))
            elif fn not in registered:
                problems.append("  %s names proof function %s, which Test%s does not build with; "
                                "%s" % (sid, fn, fn[len('step'):], REGISTRATION_SHAPE))
            claimed.add(fn)
    for fn in sorted(functions - claimed):
        problems.append("  proof function %s is not claimed by any step" % fn)
    for fn in sorted(functions - registered - claimed):
        problems.append("  proof function %s is defined but no Test%s builds with it; %s"
                        % (fn, fn[len('step'):], REGISTRATION_SHAPE))

    # The registration file is generated from the annotations, so the two have to say the same
    # thing. Without this the gate never compared a run method or a case table against anything,
    # and a step list whose annotation moved to another run over a stale generated file passed.
    annotated, annotation_problems = step_list_annotations(steps, runs.arguments)
    problems.extend(annotation_problems)
    for step_id, annotation, sound in annotated:
        registration = details.get(annotation.build)
        # A function with an annotation and no accepted registration is already the existing
        # "which Test<Title> does not build with" complaint, so it is not said twice here. An
        # annotation that drew a finding of its own is not compared either: the finding is the
        # thing to fix, and a disagreement with the proof beside it names the same defect twice.
        if registration is None or not sound:
            continue
        if (registration.method, registration.table, registration.extras) == (
                annotation.method, annotation.table, annotation.extras):
            continue
        problems.append(
            "  %s annotates %s as %s, but %s:%d registers it as %s; regenerate with `%s %s`"
            % (step_id, annotation.build, annotation_call(annotation, annotation.build),
               registration.path, registration.line,
               annotation_call(registration, annotation.build), SCAFFOLD_COMMAND, gear))
    for fn in sorted(set(details) - {annotation.build for _, annotation, _ in annotated}):
        problems.append(
            "  %s:%d registers %s, but no step carries a proof-run annotation for it; "
            "registrations are generated from the step list's annotations"
            % (details[fn].path, details[fn].line, fn))

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
    namings = spec_namings(candidates, gear) if candidates else {}
    for call in candidates:
        wrong_receivers = wrong_watchlist_receivers.get(call, ())
        if (hits[call]
                and all(api_owner_matches_receiver(hits[call], receiver)
                        for receiver in wrong_receivers)):
            continue
        note = fault_note(namings.get(call, []))
        if hits[call] and wrong_receivers:
            owners = sorted({qualified.rsplit('.', 2)[-2] for qualified, _ in hits[call]})
            for receiver in sorted(wrong_receivers, key=lambda value: value or ''):
                if not api_owner_matches_receiver(hits[call], receiver):
                    problems.append(
                        "  the step list names '%s(' on receiver '%s', but the Fusion API "
                        "database declares it on %s%s"
                        % (call, receiver, ', '.join(owners), note))
            continue
        near = fusion_api.similar(call)
        problems.append("  the step list names '%s(', which the Fusion API database does not "
                        "have%s%s"
                        % (call, '' if not near else
                           '; the nearest names it does have are %s' % ', '.join(near), note))

    # 4. inputs have not drifted
    stamped = dict(STAMPED_ROW.findall(src))
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
