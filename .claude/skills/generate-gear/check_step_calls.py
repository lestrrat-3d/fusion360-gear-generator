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
     reachable executable call in the generated file. Dotted names require a reachable attribute call;
     bare names may be implemented as either a function or a method. A step the generator skipped
     outright fails here.

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
the generator's behalf, or one named only to forbid it. Calls in code spans introduced by a
negative instruction such as "Do not call" or "never use" are also excluded. An explicit
exemption uses a line of the form:

    <!-- check-step-calls: ignore nameOne nameTwo -->
"""
import ast
import re
import sys

from call_parser import call_shapes

NEGATIVE_CALL_CONTEXT = re.compile(
    r'\b(?:do\s+not|must\s+not|never|avoid|forbid(?:den)?|prohibit(?:ed|s)?|'
    r'not\s+(?:a\s+)?substitute)\b',
    re.IGNORECASE)

STUB_PATTERN = re.compile(
    r'(TODO|FIXME|XXX|would be\b|placeholder|not implemented|unimplemented)',
    re.IGNORECASE)

# Creation calls whose whole point is to share an existing SketchPoint.
SHARE_CALL = re.compile(
    r'add(?:ByCenterRadius|ByTwoPoints)\(\s*([A-Za-z_][\w.]*)\s*,', re.S)


def is_negative_call_span(steps_src, span_start, span_end):
    """Return whether a code span is used as a forbidden example."""
    context_start = max(
        steps_src.rfind('.', 0, span_start),
        steps_src.rfind('!', 0, span_start),
        steps_src.rfind('?', 0, span_start),
        steps_src.rfind(';', 0, span_start),
        steps_src.rfind(':', 0, span_start),
        steps_src.rfind('\n', 0, span_start),
    ) + 1
    context_end_candidates = [
        position for position in (
            steps_src.find('.', span_end),
            steps_src.find('!', span_end),
            steps_src.find('?', span_end),
            steps_src.find(':', span_end),
            steps_src.find('\n', span_end),
        ) if position >= 0
    ]
    context_end = min(context_end_candidates, default=len(steps_src))
    context = re.sub(r'[*_]', '', steps_src[context_start:context_end])
    return bool(NEGATIVE_CALL_CONTEXT.search(context))


def named_calls(steps_src):
    """Extract required API calls named inside single-backtick code spans."""
    return {name for name, _ in named_call_shapes(steps_src)}


def named_call_shapes(steps_src):
    """Extract required calls, retaining whether the step names a receiver."""
    # Strip fenced blocks FIRST. Their ``` fences desync single-backtick pairing, which
    # silently drops most of the corpus — the bug that made the first draft of this check
    # report a clean pass on a file that was missing calls.
    body = re.sub(r'```.*?```', '', steps_src, flags=re.S)
    calls = set()
    for match in re.finditer(r'`([^`\n]+)`', body):
        if is_negative_call_span(body, match.start(), match.end()):
            continue
        span = match.group(1)
        calls.update(call_shapes(span))
    for line in re.findall(r'<!--\s*check-step-calls:\s*ignore\s+([^>]*?)-->', steps_src):
        ignored = set(line.split())
        calls = {call for call in calls if call[0] not in ignored}
    return calls


class ReachableCallCollector(ast.NodeVisitor):
    """Collect calls from module code and locally reachable entry-point functions."""

    ENTRY_POINTS = {'configure', 'generate'}

    def __init__(self, tree):
        self.functions = {}
        self.classes = {}
        self.methods = {}
        self.calls = set()
        self.visited = set()
        for node in ast.walk(tree):
            if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)):
                self.functions.setdefault(node.name, []).append(node)
                if node.args.posonlyargs or node.args.args:
                    self.methods.setdefault(node.name, []).append(node)
            elif isinstance(node, ast.ClassDef):
                self.classes.setdefault(node.name, []).append(node)

    def collect(self, tree):
        self.visit_statements(tree.body)
        for name in self.ENTRY_POINTS:
            for node in self.functions.get(name, ()):
                self.visit_function(node)
        return self.calls

    def visit_statements(self, statements):
        falls_through = True
        for statement in statements:
            if not falls_through:
                break
            falls_through = self.visit_statement(statement)
        return falls_through

    def visit_statement(self, statement):
        if isinstance(statement, (ast.FunctionDef, ast.AsyncFunctionDef)):
            return True
        if isinstance(statement, ast.ClassDef):
            for expression in statement.decorator_list:
                self.visit(expression)
            for base in statement.bases:
                self.visit(base)
            self.visit_statements(statement.body)
            return True
        if isinstance(statement, ast.If):
            self.visit(statement.test)
            constant = self.constant_condition(statement.test)
            if constant is True:
                return self.visit_statements(statement.body)
            if constant is False:
                return self.visit_statements(statement.orelse)
            body_falls = self.visit_statements(statement.body)
            else_falls = self.visit_statements(statement.orelse)
            return body_falls or else_falls
        if isinstance(statement, ast.While):
            self.visit(statement.test)
            if self.constant_condition(statement.test) is False:
                return self.visit_statements(statement.orelse)
            self.visit_statements(statement.body)
            self.visit_statements(statement.orelse)
            return True
        if isinstance(statement, (ast.For, ast.AsyncFor)):
            self.visit(statement.target)
            self.visit(statement.iter)
            self.visit_statements(statement.body)
            self.visit_statements(statement.orelse)
            return True
        if isinstance(statement, ast.Try):
            body_falls = self.visit_statements(statement.body)
            for handler in statement.handlers:
                if handler.type:
                    self.visit(handler.type)
                self.visit_statements(handler.body)
            else_falls = self.visit_statements(statement.orelse) if body_falls else True
            finally_falls = self.visit_statements(statement.finalbody)
            return finally_falls and (body_falls and else_falls or bool(statement.handlers))
        if isinstance(statement, (ast.With, ast.AsyncWith)):
            for item in statement.items:
                self.visit(item.context_expr)
                if item.optional_vars:
                    self.visit(item.optional_vars)
            return self.visit_statements(statement.body)
        if isinstance(statement, ast.Return):
            if statement.value:
                self.visit(statement.value)
            return False
        if isinstance(statement, ast.Raise):
            if statement.exc:
                self.visit(statement.exc)
            if statement.cause:
                self.visit(statement.cause)
            return False
        if isinstance(statement, (ast.Break, ast.Continue)):
            return False
        self.visit(statement)
        return True

    @staticmethod
    def constant_condition(expression):
        try:
            return bool(ast.literal_eval(expression))
        except (ValueError, TypeError, SyntaxError):
            return None

    def visit_function(self, node):
        marker = id(node)
        if marker in self.visited:
            return
        self.visited.add(marker)
        for default in (*node.args.defaults, *(default for default in node.args.kw_defaults if default)):
            self.visit(default)
        self.visit_statements(node.body)

    def visit_Call(self, node):
        if isinstance(node.func, ast.Name):
            self.calls.add((node.func.id, False))
            self.visit_local_name(node.func.id)
        elif isinstance(node.func, ast.Attribute):
            self.calls.add((node.func.attr, True))
            self.visit_local_method(node.func.attr)
            self.visit(node.func.value)
        else:
            self.visit(node.func)
        for argument in node.args:
            self.visit(argument)
        for keyword in node.keywords:
            self.visit(keyword.value)

    def visit_local_name(self, name):
        for node in self.functions.get(name, ()):
            self.visit_function(node)
        for node in self.classes.get(name, ()):
            for member in node.body:
                if isinstance(member, (ast.FunctionDef, ast.AsyncFunctionDef)) and member.name == '__init__':
                    self.visit_function(member)

    def visit_local_method(self, name):
        for node in self.methods.get(name, ()):
            self.visit_function(node)


def actual_call_names(gen_tree):
    """Return function and method names used by reachable Call nodes."""
    return {name for name, _ in actual_call_shapes(gen_tree)}


def actual_call_shapes(gen_tree):
    """Return reachable calls, retaining whether each call has an attribute receiver."""
    return ReachableCallCollector(gen_tree).collect(gen_tree)


def main(argv):
    if len(argv) != 3:
        print('usage: check_step_calls.py <steps.md> <generated.py>', file=sys.stderr)
        return 2
    steps_path, gen_path = argv[1], argv[2]
    steps_src = open(steps_path).read()
    gen_src = open(gen_path).read()

    problems = []

    wanted = named_call_shapes(steps_src)
    try:
        gen_tree = ast.parse(gen_src, filename=gen_path)
    except SyntaxError as err:
        problems.append("  generated candidate is not valid Python: %s" % err)
        actual = set()
    else:
        actual = actual_call_shapes(gen_tree)

    # Keep the textual scan only to explain whether a missing reachable call has a misleading
    # match in a comment or string. The AST result above is the coverage gate.
    textual = {name for name, _ in wanted if name + '(' in gen_src}
    missing = sorted(
        (name, has_receiver) for name, has_receiver in wanted
        if not any(actual_name == name and (not has_receiver or actual_receiver)
                   for actual_name, actual_receiver in actual))
    for name, has_receiver in missing:
        note = " (textual match exists, but it is not a reachable executable call)" if name in textual else ""
        call = ('receiver.%s' if has_receiver else '%s') % name
        problems.append(
                "  named-call coverage: '%s(' is named in %s but has no reachable executable %s call in %s%s"
            % (call, steps_path, 'method' if has_receiver else 'function', gen_path, note))

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
