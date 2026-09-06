#!/usr/bin/env python3
"""Gate a generated gear on the type complaints that the shipped gears do not produce.

Why this exists: `pyright_check.py` classifies by rule, and it has to be lenient, because the
Fusion intellisense stubs are incomplete enough that correct code draws a steady stream of
complaints. Everything in its REVIEW bucket is advisory, and the workflow tells authors to ignore
it. Real bugs hide there. A generated spur gear passed all six emit gates while calling
`getParameterValue` on a class that does not define it, and while handing a `BRepBodies` to a
parameter typed `ObjectCollection` — both of which pyright reported, and both of which the
workflow said to ignore.

What it does instead of judging rules: run pyright over the gears already committed in
`lib/geargen/`, and record every complaint they draw. Those files ship and work, so whatever they
trip is stub noise by definition. Any complaint the candidate draws that no shipped gear draws is
new, and new complaints are where real bugs live.

This is a diagnostic with teeth rather than a type checker. It cannot say a finding is a bug, only
that nothing working produces it, which is enough to make a human look. It therefore REPORTS by
default; pass --gate to make findings fail the run, once you trust the baseline covers the API
surface in question.

Diagnostics for calls listed in `fusion_api.UNVERIFIED_CALLS` are explicitly non-gating. The API
checker still reports those calls, and this gate only exempts the matching unavailable member on
its matching Fusion class; other candidate-only diagnostics remain blocking.

Three limits, stated plainly.

It needs at least one shipped gear to compare against, so it does nothing for a repository with no
working code yet.

A bug a shipped gear also has is never reported, because it is part of the baseline.

When the candidate is a copied shipped gear, the exact-content match is omitted from the baseline
so the workflow cannot hide candidate-only complaints by copying the source into .tmp/.

An API the shipped gears never touch has no baseline, so correct code using it reads as new. That
is not hypothetical: the stubs accept `AlignedDimensionOrientation`, which the shipped gears use,
and reject `Horizontal`/`Vertical`, which they do not, so a generated gear using the latter draws
27 findings that are all stub inconsistency. Expect to triage, and expect the noise to fall as the
shipped gears cover more of the API.

A triaged finding is recorded, not re-argued: `accepted_type_noise.json` beside this script holds
the complaint signatures a human has judged to be stub noise, each with its reason, and those
signatures never gate again. This is what lets a gear whose complaints no other gear draws (the
baseline always excludes the gear under check, so single-gear noise cannot self-baseline) stay
green after its finding is triaged. Only add an entry after deciding the complaint is really stub
pessimism; a signature is location-free, so an over-broad entry silences that complaint
everywhere.

Usage:
    python3 check_novel_types.py <candidate.py> [--reference lib/geargen] [--gate]
    python3 check_novel_types.py <candidate.py> --accept N --why "<reason>"

Recording a triaged finding goes through `--accept N --why "<reason>"`, naming the finding by its
printed index, rather than through a hand edit of `accepted_type_noise.json`.

Run from the repo root. Exit 0 = OK, 1 = BLOCKING, 2 = no reference gears to compare against
(also: a refused or unrecordable accept run).
"""
import argparse
import ast
import filecmp
import importlib.util
import json
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))

sys.path.insert(0, HERE)
import fusion_api  # noqa: E402  (sibling module; sys.path is fixed up above)


class AnalysisSetupError(Exception):
    """Raised when a shared analysis result cannot provide diagnostics."""

# Import noise: the candidate lives in .tmp/ and the gears live in a package, so their
# relative imports resolve differently. That difference is about where the file sits,
# not about what it does.
IGNORED_RULES = {'reportMissingImports', 'reportMissingModuleSource'}


ATTRIBUTE_ACCESS = re.compile(
    r'Cannot access attribute "(?P<member>\w+)" for class "(?P<class>[^\"]+)"')
UNVERIFIED_ATTRIBUTES = frozenset(
    (cls.rsplit('.', 1)[-1], member)
    for member, cls, _, _ in fusion_api.UNVERIFIED_CALLS)
QUALIFIED_UNVERIFIED_ATTRIBUTES = frozenset(
    (cls, member) for member, cls, _, _ in fusion_api.UNVERIFIED_CALLS)


def qualified_fusion_type(expression):
    if isinstance(expression, ast.Call) and isinstance(expression.func, ast.Attribute):
        return qualified_fusion_type(expression.func.value)
    parts = []
    while isinstance(expression, ast.Attribute):
        parts.append(expression.attr)
        expression = expression.value
    if isinstance(expression, ast.Name):
        parts.append(expression.id)
        parts.reverse()
        if len(parts) == 3 and parts[:2] in (['adsk', 'core'], ['adsk', 'fusion']):
            return parts[2]
    return None


def verified_fusion_classes(path):
    """Map source lines to Fusion classes proven by current qualified bindings."""
    with open(path) as source:
        tree = ast.parse(source.read(), filename=path)
    by_line = {}

    field_types = {}
    containing_classes = {}

    class Visitor(ast.NodeVisitor):
        def __init__(self):
            self.classes = []

        def visit_ClassDef(self, node):
            self.classes.append(node.name)
            for child in node.body:
                self.visit(child)
            self.classes.pop()

        def visit_Call(self, node):
            containing_classes[id(node)] = self.classes[-1] if self.classes else None
            self.generic_visit(node)

    Visitor().visit(tree)

    for node in ast.walk(tree):
        if not isinstance(node, ast.ClassDef):
            continue
        init = next((child for child in node.body
                     if isinstance(child, (ast.FunctionDef, ast.AsyncFunctionDef))
                     and child.name == '__init__'), None)
        if init is None:
            continue
        bindings = {
            argument.arg: qualified_fusion_type(argument.annotation)
            for argument in (*init.args.posonlyargs, *init.args.args, *init.args.kwonlyargs)
            if qualified_fusion_type(argument.annotation) is not None
        }
        for statement in ast.walk(init):
            if not isinstance(statement, ast.Assign):
                continue
            assigned = qualified_fusion_type(statement.value)
            if isinstance(statement.value, ast.Name):
                assigned = bindings.get(statement.value.id)
            for target in statement.targets:
                if isinstance(target, ast.Name):
                    if assigned is None:
                        bindings.pop(target.id, None)
                    else:
                        bindings[target.id] = assigned
                elif (isinstance(target, ast.Attribute)
                      and isinstance(target.value, ast.Name)
                      and target.value.id == 'self'
                      and assigned is not None):
                    field_types[(node.name, target.attr)] = assigned

    def record(node, bindings):
        start = getattr(node, 'lineno', None)
        end = getattr(node, 'end_lineno', start)
        if start is None:
            return
        for line in range(start, end + 1):
            by_line.setdefault(line, set()).update(bindings.values())

    def walk_statements(statements, bindings):
        for statement in statements:
            record(statement, bindings)
            if isinstance(statement, (ast.FunctionDef, ast.AsyncFunctionDef, ast.ClassDef)):
                continue
            if isinstance(statement, ast.Assign):
                assigned = qualified_fusion_type(statement.value)
                if isinstance(statement.value, ast.Name):
                    assigned = bindings.get(statement.value.id)
                for target in statement.targets:
                    if isinstance(target, ast.Name):
                        if assigned is None:
                            bindings.pop(target.id, None)
                        else:
                            bindings[target.id] = assigned
            elif isinstance(statement, ast.AnnAssign) and isinstance(statement.target, ast.Name):
                assigned = qualified_fusion_type(statement.annotation)
                if assigned is None and statement.value is not None:
                    assigned = qualified_fusion_type(statement.value)
                if assigned is None:
                    bindings.pop(statement.target.id, None)
                else:
                    bindings[statement.target.id] = assigned
            for child in ast.iter_child_nodes(statement):
                if isinstance(child, (ast.If, ast.For, ast.AsyncFor, ast.While, ast.With,
                                      ast.AsyncWith, ast.Try)):
                    for block in (getattr(child, 'body', []), getattr(child, 'orelse', []),
                                  getattr(child, 'finalbody', [])):
                        walk_statements(block, bindings.copy())

    for node in ast.walk(tree):
        if not isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)):
            continue
        bindings = {}
        arguments = (*node.args.posonlyargs, *node.args.args, *node.args.kwonlyargs)
        for argument in arguments:
            bound = qualified_fusion_type(argument.annotation)
            if bound is not None:
                bindings[argument.arg] = bound
        walk_statements(node.body, bindings)

    local_classes = {node.name for node in ast.walk(tree) if isinstance(node, ast.ClassDef)}
    for node in ast.walk(tree):
        if not isinstance(node, ast.Call) or not isinstance(node.func, ast.Attribute):
            continue
        class_name = None
        receiver = node.func.value
        receiver_expression = ast.unparse(receiver)
        watched = fusion_api.unverified_class(node.func.attr, receiver_expression)
        class_name = None if watched is None else watched.rsplit('.', 1)[-1]
        containing_class = containing_classes.get(id(node))
        if (class_name is not None and receiver_expression.startswith('self.')
                and isinstance(receiver, ast.Attribute)
                and isinstance(receiver.value, ast.Name)
                and receiver.value.id == 'self'):
            if field_types.get((containing_class, receiver.attr)) != class_name:
                class_name = None
        if class_name is None:
            continue
        if class_name in local_classes:
            continue
        start = getattr(node, 'lineno', None)
        end = getattr(node, 'end_lineno', start)
        for line in range(start, end + 1):
            by_line.setdefault(line, set()).add(class_name)
    return by_line


def is_unverified_api_diagnostic(diag, verified_by_line=None):
    """Whether a pyright finding names an explicitly unverified Fusion member."""
    if diag.get('rule') != 'reportAttributeAccessIssue':
        return False
    match = ATTRIBUTE_ACCESS.search(diag.get('message', ''))
    if not match:
        return False
    pair = (match.group('class'), match.group('member'))
    if pair in QUALIFIED_UNVERIFIED_ATTRIBUTES:
        return True
    line = diag.get('range', {}).get('start', {}).get('line', -1) + 1
    verified = (verified_by_line or {}).get(line, ())
    return pair in UNVERIFIED_ATTRIBUTES and match.group('class') in verified


def load_pyright_check():
    sys.path.insert(0, HERE)
    spec = importlib.util.spec_from_file_location('pyright_check',
                                                  os.path.join(HERE, 'pyright_check.py'))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def diagnostics(pc, path):
    """Return raw diagnostics for one file through the shared analysis boundary.

    The advisory needs every non-import diagnostic, including findings that the CLI classifies
    as REVIEW. It therefore consumes raw results instead of intercepting ``classify`` or
    suppressing ``SystemExit``. A setup failure is raised so the advisory cannot report success
    for an empty collection.
    """
    result = pc.analyze_paths([path])
    if result.setup_error:
        raise pc.AnalysisSetupError(result.setup_error)
    original = result.diagnostics.get(os.path.abspath(path), [])
    return [diagnostic for diagnostic in original
            if diagnostic.get('rule') not in IGNORED_RULES]


ARGUMENT_TYPE = re.compile(
    r'Argument of type "(?P<got>.+)" cannot be assigned to parameter "(?P<param>\w+)" '
    r'of type "(?P<want>.+)" in function "(?P<func>\w+)"')


def signature(diag):
    """What makes two complaints the same complaint, ignoring where they occur.

    A wrong-argument complaint is keyed on the parameter, the type it wants and the
    function, but NOT on the value passed. Otherwise passing a different member of the
    same enum reads as a brand-new complaint: the shipped gears only ever pass
    AlignedDimensionOrientation, so Horizontal and Vertical would each look novel while
    being exactly the same piece of stub pessimism.
    """
    rule = diag.get('rule')
    message = diag.get('message', '').split('\n')[0]
    m = ARGUMENT_TYPE.search(message)
    if m:
        return (rule, m.group('param'), m.group('want'), m.group('func'))
    return (rule, message)


ACCEPTED_NOISE_PATH = os.path.join(HERE, 'accepted_type_noise.json')


def entry_signature(entry):
    """The signature of one accepted_type_noise.json entry, keyed as signature() keys a diag."""
    if 'param' in entry:
        return (entry['rule'], entry['param'], entry['want'], entry['func'])
    return (entry['rule'], entry['message'])


def accepted_signatures(path=ACCEPTED_NOISE_PATH):
    """Complaint signatures a human has triaged as stub noise, from the checked-in record.

    Each JSON entry mirrors one signature() form: a wrong-argument entry carries
    `rule`/`param`/`want`/`func`, any other entry carries `rule`/`message` (first line).
    The `why` field is for the reader; it is required but not matched on.
    """
    if not os.path.isfile(path):
        return set()
    with open(path, encoding='utf-8') as handle:
        entries = json.load(handle)
    accepted = set()
    for entry in entries:
        if not entry.get('why'):
            raise ValueError('accepted_type_noise.json: every entry needs a "why": %r' % entry)
        accepted.add(entry_signature(entry))
    return accepted


def record_accepted(path, diag, why):
    """Write one triaged complaint into the accepted-noise record; return (status, entry).

    The entry is derived from signature(), so what is recorded is exactly what the check
    matches on later. An existing entry with the same signature has its `why` replaced in
    place, which keeps the file's order stable and makes re-accepting idempotent.
    """
    if not why or not why.strip():
        raise ValueError('an accepted entry needs a non-empty "why"')

    sig = signature(diag)
    if len(sig) == 4:
        rule, param, want, func = sig
        entry = {'rule': rule, 'param': param, 'want': want, 'func': func, 'why': why}
    else:
        rule, message = sig
        entry = {'rule': rule, 'message': message, 'why': why}

    entries = []
    if os.path.isfile(path):
        with open(path, encoding='utf-8') as handle:
            entries = json.load(handle)

    status = 'added'
    for existing in entries:
        if entry_signature(existing) == sig:
            existing['why'] = why
            entry = existing
            status = 'updated'
            break
    else:
        entries.append(entry)

    with open(path, 'w', encoding='utf-8') as handle:
        handle.write(json.dumps(entries, indent=2, ensure_ascii=False) + '\n')
    return status, entry


def novel_sort_key(diag):
    """Order findings so their printed indices mean the same thing on the next run."""
    return (diag['range']['start']['line'], diag.get('rule') or '',
            diag.get('message', '').split('\n')[0])


def reference_gears(reference, candidate):
    """Return shipped gears that are not the source represented by candidate.

    The workflow checks a copied candidate, so comparing only path strings would leave the
    candidate's source file in the baseline. An exact-content match is the source-file identity
    available after the copy step; same-file and real-path checks also cover in-place and symlinked
    candidates.
    """
    candidate_path = os.path.abspath(candidate)
    candidate_realpath = os.path.realpath(candidate_path)
    gears = []
    if not os.path.isdir(reference):
        return gears
    for entry in sorted(os.listdir(reference)):
        full = os.path.join(reference, entry)
        if not entry.endswith('.py') or entry.startswith('_'):
            continue
        full_path = os.path.abspath(full)
        if full_path == candidate_path or os.path.realpath(full_path) == candidate_realpath:
            continue
        try:
            if os.path.samefile(full_path, candidate_path):
                continue
        except (FileNotFoundError, OSError):
            pass
        if os.path.isfile(candidate_path) and filecmp.cmp(full_path, candidate_path, shallow=False):
            continue
        gears.append(full)
    return gears


def evaluate_analysis(result, candidate, gears):
    """Compare one batched analysis result with the candidate and reference paths.

    The returned values are deliberately plain data so the gate runner can feed the same raw
    analysis to both type consumers without invoking Pyright again.
    """
    if result.setup_error:
        raise AnalysisSetupError(result.setup_error)
    candidate_path = os.path.abspath(candidate)

    def path_diagnostics(path):
        return [diagnostic for diagnostic in result.diagnostics.get(os.path.abspath(path), [])
                if diagnostic.get('rule') not in IGNORED_RULES]

    baseline = set()
    for gear in gears:
        for diagnostic in path_diagnostics(gear):
            baseline.add(signature(diagnostic))

    candidate_diagnostics = path_diagnostics(candidate_path)
    accepted = accepted_signatures()
    verified_by_line = verified_fusion_classes(candidate_path)
    accepted_hits = sum(1 for diagnostic in candidate_diagnostics
                        if signature(diagnostic) in accepted)
    novel = sorted((diagnostic for diagnostic in candidate_diagnostics
                    if signature(diagnostic) not in baseline
                    and signature(diagnostic) not in accepted
                    and not is_unverified_api_diagnostic(diagnostic, verified_by_line)),
                   key=novel_sort_key)
    return {
        'baseline_count': len(baseline),
        'candidate_diagnostics': candidate_diagnostics,
        'accepted_hits': accepted_hits,
        'novel': novel,
        'gears': list(gears),
        'candidate': candidate_path,
    }


def render_evaluation(evaluation, *, gate=False):
    """Render a normal novel-type report and return ``(text, exit_code)``."""
    candidate = evaluation['candidate']
    gears = evaluation['gears']
    accepted_hits = evaluation['accepted_hits']
    novel = evaluation['novel']
    lines = []
    if accepted_hits:
        lines.append('novel-type check: %d complaint(s) matched accepted_type_noise.json '
                     '(triaged stub noise; see its "why" entries)' % accepted_hits)
    if novel:
        lines.append('novel-type check: %d complaint(s) no shipped gear produces — triage each'
                     % len(novel))
        for index, diagnostic in enumerate(novel, start=1):
            lines.append('  [%d] %s:%d [%s] %s'
                         % (index, candidate, diagnostic['range']['start']['line'] + 1,
                            diagnostic.get('rule'), diagnostic.get('message', '').split('\n')[0]))
        lines.append('  Record one as triaged stub noise: python3 '
                     '.claude/skills/generate-gear/check_novel_types.py %s '
                     '--accept N --why "<reason>"' % candidate)
        lines.append('  Baseline came from %d shipped gear(s) drawing %d distinct complaint(s).'
                     % (len(gears), evaluation['baseline_count']))
        return '\n'.join(lines), (1 if gate else 0)
    lines.append('novel-type check: OK (nothing the %d shipped gear(s) do not already produce)'
                 % len(gears))
    return '\n'.join(lines), 0


def main():
    ap = argparse.ArgumentParser(add_help=True)
    ap.add_argument('candidate')
    ap.add_argument('--reference', default='lib/geargen',
                    help='directory of shipped gears to take the baseline from')
    ap.add_argument('--gate', action='store_true',
                    help='exit 1 on any finding instead of only reporting them')
    ap.add_argument('--accept', type=int, metavar='N',
                    help='record the Nth reported finding (1-based) as triaged stub noise')
    ap.add_argument('--why', metavar='TEXT',
                    help='the reason the accepted finding is stub noise; required with --accept')
    args = ap.parse_args()

    if args.accept is not None:
        if args.gate:
            ap.error('--accept records one triaged verdict and cannot be combined with --gate')
        if not args.why or not args.why.strip():
            ap.error('--accept needs --why "<reason>": an accepted entry states why it is noise')

    pc = load_pyright_check()

    gears = reference_gears(args.reference, args.candidate)
    if not gears:
        print('check_novel_types: no reference gears under %s, nothing to compare against'
              % args.reference, file=sys.stderr)
        return 2

    try:
        result = pc.analyze_paths(gears + [args.candidate])
        evaluation = evaluate_analysis(result, args.candidate, gears)
    except (pc.AnalysisSetupError, AnalysisSetupError) as error:
        print('ERROR: %s' % error, file=sys.stderr)
        return 2
    novel = evaluation['novel']

    if args.accept is not None:
        if not novel:
            print('nothing to accept: no novel findings', file=sys.stderr)
            return 2
        if not 1 <= args.accept <= len(novel):
            print('--accept %d out of range: %d novel finding(s)' % (args.accept, len(novel)),
                  file=sys.stderr)
            return 2
        chosen = novel[args.accept - 1]
        try:
            status, entry = record_accepted(ACCEPTED_NOISE_PATH, chosen, args.why)
        except ValueError as error:
            print('cannot accept: %s' % error, file=sys.stderr)
            return 2
        print('novel-type check: %s accepted_type_noise.json entry' % status)
        print(json.dumps(entry, indent=2, ensure_ascii=False))
        print('  recorded from %s:%d [%s] %s'
              % (args.candidate, chosen['range']['start']['line'] + 1, chosen.get('rule'),
                 chosen.get('message', '').split('\n')[0]))
        return 0

    report, exit_code = render_evaluation(evaluation, gate=args.gate)
    print(report)
    return exit_code


if __name__ == '__main__':
    sys.exit(main())
