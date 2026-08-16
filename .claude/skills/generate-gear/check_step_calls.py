#!/usr/bin/env python3
"""Gate a step-list-driven emit against the step list's own named API calls.

Why this exists: the existing gates (parse, pyright, check_input_read, check_contract) all
inspect *structure* — class names, input ids, reader types, module constants. A generated file
can satisfy every one of them while silently skipping whole steps, leaving an abandoned stub in
place, or degrading a call the step list spelled out. That is not hypothetical: the first
step-list pilot passed all four gates while skipping the circle labels, dropping the angular
dimension, leaving `prev_pt = None  # Would be previous midpoint` in the rib chain, and passing
`.geometry` to every circle centre the step list said to share.

Three checks, all derived from `spec/<gear>/steps.md` itself, so the step list doubles as the
checklist and there is nothing separate to keep in sync:

  1. NAMED-CALL COVERAGE. Every API call the step list names inside a code span must occur as an
     executable call in the generated file. A step the generator skipped outright fails here.

  2. STUB MARKERS. Abandoned work leaves a fingerprint. Any TODO / FIXME / "would be" /
     "placeholder" / "not implemented" comment fails.

  3. SHARED-POINT MISUSE. `addByCenterRadius`/`addByTwoPoints` exist to SHARE an existing
     SketchPoint ([PB-SHARE-XOR-COINCIDENT]). Passing `<something>.geometry` as the shared
     argument creates a fresh point instead, which silently breaks the closed profile.

None of these prove the geometry is right. They prove the generator did what the step list said,
which is the layer that was entirely unguarded.

Usage:
    python3 check_step_calls.py spec/<gear>/steps.md .tmp/<gear>.generated.py

Exit 0 = OK, 1 = BLOCKING.

The step list may exempt a name it mentions but does not require — a call the framework makes on
the generator's behalf, or one named only to forbid it — with a line of the form:

    <!-- check-step-calls: ignore nameOne nameTwo -->
"""
import ast
import re
import sys

# Names shorter than this are almost always prose words followed by a paren.
MIN_NAME_LEN = 6

STUB_PATTERN = re.compile(
    r'(TODO|FIXME|XXX|would be\b|placeholder|not implemented|unimplemented)',
    re.IGNORECASE)

# Creation calls whose whole point is to share an existing SketchPoint.
SHARE_CALL = re.compile(
    r'add(?:ByCenterRadius|ByTwoPoints)\(\s*([A-Za-z_][\w.]*)\s*,', re.S)


def named_calls(steps_src):
    """Extract every API call the step list names inside a single-backtick code span."""
    # Strip fenced blocks FIRST. Their ``` fences desync single-backtick pairing, which
    # silently drops most of the corpus — the bug that made the first draft of this check
    # report a clean pass on a file that was missing calls.
    body = re.sub(r'```.*?```', '', steps_src, flags=re.S)
    names = set()
    for span in re.findall(r'`([^`\n]+)`', body):
        for name in re.findall(r'\b([a-z][A-Za-z0-9_]{%d,})\s*\(' % (MIN_NAME_LEN - 1), span):
            names.add(name)
    for line in re.findall(r'<!--\s*check-step-calls:\s*ignore\s+([^>]*?)-->', steps_src):
        names -= set(line.split())
    return names


def actual_call_names(gen_tree):
    """Return function and method names used by executable Call nodes."""
    names = set()
    for node in ast.walk(gen_tree):
        if not isinstance(node, ast.Call):
            continue
        if isinstance(node.func, ast.Name):
            names.add(node.func.id)
        elif isinstance(node.func, ast.Attribute):
            names.add(node.func.attr)
    return names


def main(argv):
    if len(argv) != 3:
        print('usage: check_step_calls.py <steps.md> <generated.py>', file=sys.stderr)
        return 2
    steps_path, gen_path = argv[1], argv[2]
    steps_src = open(steps_path).read()
    gen_src = open(gen_path).read()

    problems = []

    wanted = named_calls(steps_src)
    try:
        gen_tree = ast.parse(gen_src, filename=gen_path)
    except SyntaxError as err:
        problems.append("  generated candidate is not valid Python: %s" % err)
        actual = set()
    else:
        actual = actual_call_names(gen_tree)

    # Keep the textual scan only to explain whether a missing executable call has a misleading
    # match in a comment or string. The AST result above is the coverage gate.
    textual = {n for n in wanted if n + '(' in gen_src}
    missing = sorted(wanted - actual)
    for name in missing:
        note = " (textual match exists, but it is not an executable call)" if name in textual else ""
        problems.append(
            "  named-call coverage: '%s(' is named in %s but has no executable call in %s%s"
            % (name, steps_path, gen_path, note))

    for lineno, line in enumerate(gen_src.splitlines(), 1):
        hit = STUB_PATTERN.search(line)
        if hit:
            problems.append(
                "  stub marker: %s:%d carries '%s' — %s"
                % (gen_path, lineno, hit.group(1), line.strip()))

    # Scan the whole source, not line by line: these calls routinely wrap, and the argument
    # lands on the following line.
    for match in SHARE_CALL.finditer(gen_src):
        arg = match.group(1)
        if arg.endswith('.geometry'):
            lineno = gen_src.count('\n', 0, match.start()) + 1
            problems.append(
                "  shared-point misuse: %s:%d passes '%s' where the SketchPoint itself "
                "must be shared ([PB-SHARE-XOR-COINCIDENT])" % (gen_path, lineno, arg))

    if problems:
        print('step-call check: BLOCKING (%d)' % len(problems))
        for p in problems:
            print(p)
        return 1

    print('step-call check: OK (%d named calls present, no stubs, no shared-point misuse)'
          % len(wanted))
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv))
