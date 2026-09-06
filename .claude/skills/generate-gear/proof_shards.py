#!/usr/bin/env python3
"""Own the split of the geometry proofs into the parallel CI groups named in `proof/shards.json`.

The whole suite is one `go test ./...`, and on a four-core runner its wall clock is the longest
single package: `bevelgear` alone accounted for 102 of the 112 seconds the job spent running
proofs. Nothing inside `go test` can shorten that, because the harness registers no `t.Parallel`
and a package's top-level tests therefore run one after another. Splitting the work across CI
jobs can, and that is what this script and its manifest exist to do.

The dangerous version of that idea is a hand-maintained list of `-run` expressions in the
workflow. A test added to a package would simply stop being run, and every job would still be
green, because a `-run` expression that selects nothing is a pass. So the assignment is checked
rather than trusted:

    inventory   ask the Go toolchain what packages exist and what top-level names each one holds
    check       every discovered name is assigned to exactly one group, and every assigned name
                still exists; the regex a group emits selects that group's names and no others
    matrix      the group names, as JSON, for the workflow's matrix
    run         execute one group through proof/run.sh

The workflow gets both its matrix and its selections from here, so there is no second answer to
the question of which tests run where. `test_proof_shards.py` holds that to it.

Why the manifest is explicit rather than a rule like "one group per package": a rule that derives
groups from the tree cannot be unbalanced, but it also cannot be balanced. The groups exist to
make the longest job short, which is a fact about measured seconds, not about package names. So
the split is written down, and adding a test is a deliberate edit that says which group pays for
it. A new test with no assignment fails `check`; it is never swept into a catch-all whose runtime
nobody chose.

Usage:
    python3 proof_shards.py inventory [--out PATH]
    python3 proof_shards.py check [--inventory PATH]
    python3 proof_shards.py matrix
    python3 proof_shards.py select <group>
    python3 proof_shards.py run <group> [-- GO_TEST_ARG...]

Run from the repo root, like `check_compile.py`. Exit 0 = the manifest and the tree agree, 1 =
findings, 2 = an input is missing or unreadable. `run` propagates whatever `proof/run.sh` exits
with.
"""
import argparse
import json
import os
import re
import subprocess
import sys

MANIFEST = os.path.join('proof', 'shards.json')
RUNNER = os.path.join('proof', 'run.sh')
GO_MOD = os.path.join('proof', 'go.mod')

# `go test -list` prints benchmarks alongside tests, examples and fuzz targets. A normal
# `go test` run never executes a benchmark, so a benchmark is not work any group owns and
# assigning one would be a promise the run does not keep. Examples and fuzz targets do run
# — an example with an output comment and a fuzz target's seed corpus — so both are inventoried.
RUNNABLE_PREFIXES = ('Test', 'Example', 'Fuzz')
BENCHMARK_PREFIX = 'Benchmark'

# A group name reaches CI as a job name and a matrix value, and reaches this script back as an
# argument. Keep it to what survives all three without quoting.
GROUP_NAME = re.compile(r'\A[a-z0-9]+(?:-[a-z0-9]+)*\Z')

# What a name may hold for the emitted pattern to be checkable here. Go allows a Unicode letter
# after `func Test`, and Go's `-run` matcher is RE2 while the check below is Python's `re`; the
# two agree exactly on an anchored alternation of names drawn from this set, and are not worth
# assuming equal outside it. A name outside it is reported, not silently trusted.
PLAIN_NAME = re.compile(r'\A\w+\Z', re.ASCII)


def read(path):
    with open(path, encoding='utf-8') as handle:
        return handle.read()


def read_json(path):
    """Parse a JSON file, raising ValueError with the path in the message when it will not."""
    with open(path, encoding='utf-8') as handle:
        text = handle.read()
    try:
        return json.loads(text)
    except ValueError as exc:
        raise ValueError('%s is not readable JSON: %s' % (path, exc))


def escape(name):
    """The name as a Go regexp literal.

    Backslash-escaping punctuation is safe in RE2 and backslash-escaping a letter is not, so the
    two cases are separated rather than handed to `re.escape`, which does not draw that line the
    same way. A name Go itself could not have produced from a `func Test…` declaration raises,
    because emitting it would produce a pattern the toolchain rejects and the job would fail with
    a regexp error rather than with the real complaint.
    """
    out = []
    for character in name:
        if character.isalnum() or character == '_':
            out.append(character)
        elif character.isascii() and not character.isspace():
            out.append('\\' + character)
        else:
            raise ValueError('cannot express %r in a Go test pattern' % name)
    return ''.join(out)


def pattern_for(names):
    """The anchored alternation that selects exactly `names` and nothing else.

    Anchored because a bare `TestBore` also selects `TestBoreCut`, and the two live in different
    groups in this repository more than once. Sorted so the manifest and the emitted command are
    stable, which is what makes a workflow log diffable against the last run.

    No group's pattern contains `/`. `go test -run` splits its expression on `/` and applies each
    part to one level of the test tree, so a pattern without one constrains the top-level name and
    leaves every subtest, including the ones a case table creates at run time, to run.
    """
    if not names:
        # `-run ''` matches everything, so an empty group must not produce an empty pattern.
        return '^$'
    return '^(%s)$' % '|'.join(escape(name) for name in sorted(names))


def selects(pattern, names):
    """The subset of `names` a Go `-run` pattern would select, as far as top-level names go.

    Used to prove a group's own pattern does not reach a name assigned elsewhere. Go matches an
    unanchored search against each top-level name, which is what `re.search` does here; the
    patterns are anchored and the names are restricted to `PLAIN_NAME`, so the answer does not
    depend on which of the two regexp engines asks.
    """
    compiled = re.compile(pattern)
    return sorted(name for name in names if compiled.search(name))


def groups_of(manifest, path=MANIFEST):
    """The manifest's groups, after the structural complaints that make the rest meaningless."""
    if not isinstance(manifest, dict):
        raise ValueError('%s must hold a JSON object' % path)
    groups = manifest.get('groups')
    if not isinstance(groups, list) or not groups:
        raise ValueError('%s declares no groups' % path)
    for group in groups:
        if not isinstance(group, dict):
            raise ValueError('%s: every group must be a JSON object' % path)
        if not isinstance(group.get('packages'), dict):
            raise ValueError('%s: group %r declares no packages object'
                             % (path, group.get('name')))
    return groups


def check(manifest, inventory, path=MANIFEST):
    """Every complaint about the manifest, given what the toolchain says the tree holds.

    Pure, so the regression tests can feed it an inventory they wrote by hand. Order is fixed:
    structure, then coverage, then the emitted patterns.
    """
    findings = []
    groups = groups_of(manifest, path)
    packages = inventory.get('packages')
    if not isinstance(packages, dict):
        raise ValueError('the inventory holds no packages object')

    for package, names in sorted(packages.items()):
        for test in sorted(names):
            if not PLAIN_NAME.match(test):
                findings.append(
                    '%s %s holds a character this checker cannot prove a -run pattern for; '
                    'rename it to ASCII letters, digits and underscores' % (package, test))

    seen_names = set()
    owner = {}
    for group in groups:
        name = group.get('name')
        if not isinstance(name, str) or not GROUP_NAME.match(name):
            findings.append('group name %r is not a lowercase hyphenated word' % (name,))
            continue
        if name in seen_names:
            findings.append('group %s is declared twice' % name)
            continue
        seen_names.add(name)

        assigned_here = 0
        for package, names in sorted(group['packages'].items()):
            if package not in packages:
                findings.append('group %s names package %s, which go list does not report'
                                % (name, package))
                continue
            if not isinstance(names, list):
                findings.append('group %s: package %s must map to a list of test names'
                                % (name, package))
                continue
            for test in names:
                if not isinstance(test, str) or not test:
                    findings.append('group %s: package %s lists %r, which is not a test name'
                                    % (name, package, test))
                    continue
                if '/' in test:
                    findings.append(
                        'group %s: %s %s names a subtest; a group owns whole top-level tests'
                        % (name, package, test))
                    continue
                if test not in packages[package]:
                    findings.append('group %s: %s %s is not in the package any more'
                                    % (name, package, test))
                    continue
                key = (package, test)
                if key in owner:
                    findings.append('%s %s is assigned to both %s and %s'
                                    % (package, test, owner[key], name))
                    continue
                owner[key] = name
                assigned_here += 1

        if not assigned_here and not group.get('allow_no_tests'):
            findings.append(
                'group %s selects no test; set "allow_no_tests": true if that is deliberate'
                % name)

    for package, names in sorted(packages.items()):
        if not any(package in group.get('packages', {}) for group in groups):
            findings.append(
                'package %s is in no group, so nothing would compile or run it' % package)
            continue
        for test in sorted(names):
            if (package, test) not in owner:
                findings.append('%s %s is in no group, so no job would run it' % (package, test))

    findings += pattern_findings(groups, packages)
    return findings


def pattern_findings(groups, packages):
    """Complaints about what each group's emitted pattern actually selects.

    A group holding two packages emits one pattern for both, because `go test` takes one `-run`
    for the whole invocation. Two packages in this repository declare the same test name, so a
    pattern that is right for one package can quietly select a name in the other that belongs to a
    different group — the test would then run twice and the split would no longer be a partition.
    """
    findings = []
    for group in groups:
        name = group.get('name')
        assigned = {}
        for package, names in group['packages'].items():
            if package in packages and isinstance(names, list):
                assigned[package] = sorted(test for test in names if isinstance(test, str))
        try:
            pattern = pattern_for(sum(assigned.values(), []))
        except ValueError as exc:
            findings.append('group %s: %s' % (name, exc))
            continue
        if '/' in pattern:
            findings.append('group %s emits %s, which would constrain subtests' % (name, pattern))
            continue
        for package, expected in sorted(assigned.items()):
            actual = selects(pattern, packages[package])
            if sorted(actual) != expected:
                findings.append(
                    'group %s: %s selects %s in %s, but the group is assigned %s'
                    % (name, pattern, sorted(actual) or 'nothing', package,
                       expected or 'nothing'))
    return findings


def module_path(path=GO_MOD):
    """The proof module's import path, from its `go.mod`.

    Read rather than asked of `go list -m`, for the reason `scaffold_proof.py` reads it: it is
    one line of one file, and taking it from there costs nothing and needs no module graph.
    """
    for line in read(path).split('\n'):
        stripped = line.strip()
        if stripped.startswith('module') and stripped[len('module'):].strip():
            return stripped[len('module'):].strip()
    raise ValueError('%s declares no module path' % path)


def module_relative(import_path, module):
    """`./bevelgear` for the import path of a package inside `module`."""
    if import_path == module:
        return '.'
    if import_path.startswith(module + '/'):
        return './' + import_path[len(module) + 1:]
    raise ValueError('%s is not inside %s' % (import_path, module))


def names_from_events(text, module):
    """`{module-relative package: [runnable top-level name]}` from `go test -list -json` output.

    A package with no test files still gets an entry, from its `start` event, because it is work
    a group has to own: nothing else in CI compiles it.

    Every line that is not a JSON object is skipped by shape, which is what lets the runner's own
    preamble share the stream. A `Benchmark` name is dropped: `go test -list` reports benchmarks,
    a normal run never executes one, and assigning it would be a promise no job keeps.
    """
    found = {}
    for line in text.splitlines():
        line = line.strip()
        if not line.startswith('{'):
            continue
        event = json.loads(line)
        package = module_relative(event['Package'], module)
        found.setdefault(package, [])
        if event.get('Action') != 'output':
            continue
        name = event.get('Output', '').strip()
        if not name or not name.startswith(RUNNABLE_PREFIXES):
            continue
        if name.startswith(BENCHMARK_PREFIX) or any(c.isspace() for c in name):
            continue
        found[package].append(name)
    return {package: sorted(names) for package, names in found.items()}


def discover():
    """The inventory, from one `proof/run.sh` invocation.

    Through the runner rather than a bare `go test` so the names come from the same engine
    revisions, module graph and toolchain the groups execute against, and so this fails the same
    way every other job does when a checkout has drifted off the pin.

    `-json` rather than the plain `-list` output because plain `go test` reports the names as
    bare lines and only a later `ok <package>` line says whose they were; nothing in that format
    survives packages being built in parallel. The JSON events name the package on every line.
    """
    completed = subprocess.run(
        ['bash', RUNNER, '--package', './...', '--', '-list', '.*', '-json'],
        stdout=subprocess.PIPE, stderr=None, text=True, check=False)
    if completed.returncode != 0:
        raise RuntimeError('%s exited %d while listing tests' % (RUNNER, completed.returncode))
    return {'packages': names_from_events(completed.stdout, module_path())}


def selection(manifest, name, path=MANIFEST):
    """The `proof/run.sh` arguments that run one group."""
    for group in groups_of(manifest, path):
        if group.get('name') != name:
            continue
        arguments = []
        for package in sorted(group['packages']):
            arguments += ['--package', package]
        assigned = sum((names for names in group['packages'].values()
                        if isinstance(names, list)), [])
        return arguments + ['--', '-run', pattern_for(assigned)]
    raise ValueError('%s declares no group named %s' % (path, name))


def main(argv):
    parser = argparse.ArgumentParser(
        prog='proof_shards.py',
        description='Own the proof suite\'s split into parallel CI groups.')
    parser.add_argument('--manifest', default=MANIFEST,
                        help='read the assignment from PATH instead of %s' % MANIFEST)
    commands = parser.add_subparsers(dest='command', required=True)
    inventory = commands.add_parser('inventory', help='ask the toolchain what the tree holds')
    inventory.add_argument('--out', help='write the inventory to PATH instead of stdout')
    checker = commands.add_parser('check', help='hold the manifest to the tree')
    checker.add_argument('--inventory', help='read the inventory from PATH instead of running go')
    commands.add_parser('matrix', help='print the group names as a JSON list')
    selector = commands.add_parser('select', help='print the run.sh arguments for one group')
    selector.add_argument('group')
    runner = commands.add_parser('run', help='run one group through proof/run.sh')
    runner.add_argument('group')
    runner.add_argument('go_args', nargs='*',
                        help='extra go test arguments, after a bare --')
    arguments = parser.parse_args(argv[1:])

    try:
        manifest = read_json(arguments.manifest)
    except OSError as exc:
        print('shards: %s cannot be read: %s' % (arguments.manifest, exc), file=sys.stderr)
        return 2
    except ValueError as exc:
        print('shards: %s' % exc, file=sys.stderr)
        return 2

    if arguments.command == 'matrix':
        try:
            names = [group.get('name') for group in groups_of(manifest, arguments.manifest)]
        except ValueError as exc:
            print('shards: %s' % exc, file=sys.stderr)
            return 2
        print(json.dumps(names))
        return 0

    if arguments.command in ('select', 'run'):
        try:
            selected = selection(manifest, arguments.group, arguments.manifest)
        except ValueError as exc:
            print('shards: %s' % exc, file=sys.stderr)
            return 2
        if arguments.command == 'select':
            print(' '.join(selected))
            return 0
        if not os.path.isfile(RUNNER):
            print('shards: no runner at %s; run from the repository root' % RUNNER,
                  file=sys.stderr)
            return 2
        extra = [argument for argument in arguments.go_args if argument != '--']
        return subprocess.run(['bash', RUNNER] + selected + extra, check=False).returncode

    if arguments.command == 'inventory':
        try:
            found = discover()
        except (OSError, RuntimeError, ValueError) as exc:
            print('shards: %s' % exc, file=sys.stderr)
            return 2
        text = json.dumps(found, indent=2, sort_keys=True) + '\n'
        if arguments.out:
            with open(arguments.out, 'w', encoding='utf-8') as handle:
                handle.write(text)
            print('shards: wrote %s' % arguments.out)
        else:
            sys.stdout.write(text)
        return 0

    try:
        if arguments.inventory:
            found = read_json(arguments.inventory)
        else:
            found = discover()
        findings = check(manifest, found, arguments.manifest)
    except (OSError, RuntimeError, ValueError) as exc:
        print('shards: %s' % exc, file=sys.stderr)
        return 2
    if findings:
        print('shards: BLOCKING (%d)' % len(findings))
        for finding in findings:
            print(finding)
        return 1
    total = sum(len(names) for names in found['packages'].values())
    print('shards: %d packages and %d top-level tests, each in exactly one of %d groups'
          % (len(found['packages']), total, len(manifest['groups'])))
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv))
