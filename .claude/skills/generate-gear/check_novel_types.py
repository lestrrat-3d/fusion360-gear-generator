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

Usage:
    python3 check_novel_types.py <candidate.py> [--reference lib/geargen]

Run from the repo root. Exit 0 = OK, 1 = BLOCKING, 2 = no reference gears to compare against.
"""
import argparse
import contextlib
import filecmp
import importlib.util
import io
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))

sys.path.insert(0, HERE)
import fusion_api  # noqa: E402  (sibling module; sys.path is fixed up above)

# Import noise: the candidate lives in .tmp/ and the gears live in a package, so their
# relative imports resolve differently. That difference is about where the file sits,
# not about what it does.
IGNORED_RULES = {'reportMissingImports', 'reportMissingModuleSource'}


ATTRIBUTE_ACCESS = re.compile(
    r'Cannot access attribute "(?P<member>\w+)" for class "(?P<class>[^\"]+)"')
UNVERIFIED_ATTRIBUTES = frozenset(
    (cls.rsplit('.', 1)[-1], member)
    for member, cls, _, _ in fusion_api.UNVERIFIED_CALLS)


def is_unverified_api_diagnostic(diag):
    """Whether a pyright finding names an explicitly unverified Fusion member."""
    if diag.get('rule') != 'reportAttributeAccessIssue':
        return False
    match = ATTRIBUTE_ACCESS.search(diag.get('message', ''))
    return bool(match and (match.group('class'), match.group('member'))
                in UNVERIFIED_ATTRIBUTES)


def load_pyright_check():
    sys.path.insert(0, HERE)
    spec = importlib.util.spec_from_file_location('pyright_check',
                                                  os.path.join(HERE, 'pyright_check.py'))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def diagnostics(pc, path):
    """Every diagnostic pyright reports for one file, whatever pyright_check makes of it."""
    found = []
    original = pc.classify

    def spy(diag):
        if diag.get('rule') not in IGNORED_RULES:
            found.append(diag)
        return original(diag)

    pc.classify = spy
    argv = sys.argv
    sys.argv = ['pyright_check.py', path]
    try:
        with contextlib.redirect_stdout(io.StringIO()):
            try:
                pc.main()
            except SystemExit:
                pass
    finally:
        pc.classify = original
        sys.argv = argv
    return found


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


def main():
    ap = argparse.ArgumentParser(add_help=True)
    ap.add_argument('candidate')
    ap.add_argument('--reference', default='lib/geargen',
                    help='directory of shipped gears to take the baseline from')
    ap.add_argument('--gate', action='store_true',
                    help='exit 1 on any finding instead of only reporting them')
    args = ap.parse_args()

    pc = load_pyright_check()

    gears = reference_gears(args.reference, args.candidate)
    if not gears:
        print('check_novel_types: no reference gears under %s, nothing to compare against'
              % args.reference, file=sys.stderr)
        return 2

    baseline = set()
    for gear in gears:
        for diag in diagnostics(pc, gear):
            baseline.add(signature(diag))

    novel = [d for d in diagnostics(pc, args.candidate)
             if signature(d) not in baseline and not is_unverified_api_diagnostic(d)]
    if novel:
        print('novel-type check: %d complaint(s) no shipped gear produces — triage each'
              % len(novel))
        for d in sorted(novel, key=lambda d: d['range']['start']['line']):
            print('  %s:%d [%s] %s' % (args.candidate, d['range']['start']['line'] + 1,
                                       d.get('rule'), d.get('message', '').split('\n')[0]))
        print('  Baseline came from %d shipped gear(s) drawing %d distinct complaint(s).'
              % (len(gears), len(baseline)))
        return 1 if args.gate else 0

    print('novel-type check: OK (nothing the %d shipped gear(s) do not already produce)'
          % len(gears))
    return 0


if __name__ == '__main__':
    sys.exit(main())
