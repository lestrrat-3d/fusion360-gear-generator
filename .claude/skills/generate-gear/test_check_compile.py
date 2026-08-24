#!/usr/bin/env python3
"""Regression tests for the compile-stage gate."""
import ast
import collections
import contextlib
import importlib.util
import inspect
import io
import json
import os
import re
import shutil
import subprocess
import sys
import tempfile
import unicodedata
import unittest
from pathlib import Path
from unittest import mock


COMPILE_CHECKER_PATH = Path(__file__).with_name('check_compile.py')
PROVENANCE_MODULE_PATH = Path(__file__).with_name('provenance.py')
COMPILE_MODULE_SPEC = importlib.util.spec_from_file_location('check_compile', COMPILE_CHECKER_PATH)
COMPILE_CHECKER = importlib.util.module_from_spec(COMPILE_MODULE_SPEC)
COMPILE_MODULE_SPEC.loader.exec_module(COMPILE_CHECKER)


# Every header spelling the gate has an opinion about, with what Go does to it and what the gate
# does about that. The whole class of defect on this boundary is the gate disagreeing with Go
# about a spelling, so a spelling is not settled until both halves are written down and both are
# checked: `test_go_agrees_with_every_recorded_header_verdict` runs the Go toolchain over this
# corpus and fails if a `go` column ever stops being true, and the three gate tests below run the
# checker over the same corpus.
#
# A row carries the bytes above the package clause and nothing else; the file under test is that
# header followed by a body. The `go` column is what Go does with the header:
#
#   ignored             the constraint is honoured and excludes the file — `IgnoredGoFiles`
#   invalid-constraint  the constraint is read but does not parse — `InvalidGoFiles`
#   no-constraint       no constraint is read, whatever else Go thinks of the file
#   unreadable          Go refuses the file's bytes, so `go build` never compiles it and the
#                       constraint question never arises
#
# The `gate` column says what the checker does and, when it refuses, which complaint it makes:
#
#   accept              the file is read and scanned
#   refuse-constraint   "carries a build constraint"
#   refuse-unreadable   "so Go refuses the file", because Go cannot read those bytes
#
# Go and the gate agree everywhere except the `+build` near-misses grouped at the end, which are
# refused deliberately — `PLUS_BUILD_DIRECTIVE` in the checker states why — and the two
# `illegal character` rows, which are the known edge named where they sit.
BOM = b'\xef\xbb\xbf'

GO_HEADER_CASES = (
    # name, header bytes, what Go does, what the gate does
    ('a plain constraint', b'//go:build ignore\n\n', 'ignored', 'refuse-constraint'),
    ('a tab separator', b'//go:build\tignore\n\n', 'ignored', 'refuse-constraint'),
    ('an indented constraint', b'   //go:build ignore\n\n', 'ignored', 'refuse-constraint'),
    ('a tab-indented constraint', b'\t//go:build ignore\n\n', 'ignored', 'refuse-constraint'),
    ('a constraint on the second header line', b'// note\n//go:build ignore\n\n',
     'ignored', 'refuse-constraint'),
    ('no blank line before the package clause', b'//go:build ignore\n', 'ignored',
     'refuse-constraint'),
    ('CRLF line endings', b'//go:build ignore\r\n\r\n', 'ignored', 'refuse-constraint'),
    ('lone CR line endings', b'//go:build ignore\r\r', 'invalid-constraint', 'refuse-constraint'),
    ('a bare directive', b'//go:build\n\n', 'invalid-constraint', 'refuse-constraint'),
    ('a directive followed only by spaces', b'//go:build   \n\n',
     'invalid-constraint', 'refuse-constraint'),
    ('a directive followed only by a tab', b'//go:build\t\n\n', 'invalid-constraint',
     'refuse-constraint'),
    ('a byte order mark above the directive', BOM + b'//go:build ignore\n\n',
     'ignored', 'refuse-constraint'),
    ('a byte order mark above an indented directive', BOM + b'   //go:build ignore\n\n',
     'ignored', 'refuse-constraint'),
    ('a byte order mark above the legacy form', BOM + b'// +build ignore\n\n',
     'ignored', 'refuse-constraint'),
    # Go trims the header line with `unicode.IsSpace`, which counts a non-breaking space.
    ('a non-breaking space before the slashes', b'\xc2\xa0//go:build ignore\n\n',
     'ignored', 'refuse-constraint'),
    ('a closed block comment on the line above', b'/* note */\n//go:build ignore\n\n',
     'ignored', 'refuse-constraint'),
    ('a closed multi-line block comment above', b'/*\nnote\n*/\n//go:build ignore\n\n',
     'ignored', 'refuse-constraint'),
    ('a `/*` inside a line comment above', b'// /* note\n//go:build ignore\n\n',
     'ignored', 'refuse-constraint'),
    ('the legacy form', b'// +build ignore\n\n', 'ignored', 'refuse-constraint'),
    ('the legacy form with no space', b'//+build ignore\n\n', 'ignored', 'refuse-constraint'),
    ('the legacy form with two spaces', b'//  +build ignore\n\n', 'ignored', 'refuse-constraint'),
    ('the legacy form with a tab', b'//\t+build ignore\n\n', 'ignored', 'refuse-constraint'),
    ('the legacy form indented', b'  // +build ignore\n\n', 'ignored', 'refuse-constraint'),
    ('a bare legacy directive', b'// +build\n\n', 'ignored', 'refuse-constraint'),

    ('a space after the slashes', b'// go:build ignore\n\n', 'no-constraint', 'accept'),
    ('prose about the directive', b'// go:build is discussed here, not used\n\n',
     'no-constraint', 'accept'),
    ('two spaces after the slashes', b'//  go:build ignore\n\n', 'no-constraint', 'accept'),
    ('a space inside the directive', b'// go: build ignore\n\n', 'no-constraint', 'accept'),
    ('a `!` straight after the directive', b'//go:build!ignore\n\n', 'no-constraint', 'accept'),
    ('a `/` straight after the directive', b'//go:build/ignore\n\n', 'no-constraint', 'accept'),
    ('a word run on to the directive', b'//go:buildignore\n\n', 'no-constraint', 'accept'),
    ('a directive quoted inside a block comment', b'/*\n\t//go:build ignore\n*/\n',
     'no-constraint', 'accept'),
    ('a block comment closing on the directive line', b'/* note */ //go:build ignore\n\n',
     'no-constraint', 'accept'),
    ('a byte order mark and no constraint', BOM, 'no-constraint', 'accept'),

    # Bytes Go refuses to read. Every one of these makes `go build` fail, so the file is never
    # compiled and nothing it registers is ever built; crediting one is a false pass. The
    # constraint written under them is beside the point — Go never gets far enough to read it —
    # which is why the `go` column says `unreadable` rather than `no-constraint`.
    ('a UTF-16 byte order mark', b'\xff\xfe//go:build ignore\n\n', 'unreadable',
     'refuse-unreadable'),
    ('a big-endian UTF-16 byte order mark', b'\xfe\xff//go:build ignore\n\n', 'unreadable',
     'refuse-unreadable'),
    ('a stray 0xFF in a header comment', b'// no\xffte\n\n', 'unreadable', 'refuse-unreadable'),
    ('a NUL byte', b'\x00//go:build ignore\n\n', 'unreadable', 'refuse-unreadable'),
    ('a NUL byte in a header comment', b'// no\x00te\n\n', 'unreadable', 'refuse-unreadable'),
    ('two byte order marks above the directive', BOM + BOM + b'//go:build ignore\n\n',
     'unreadable', 'refuse-unreadable'),
    ('a byte order mark below the first line', b'// note\n' + BOM + b'//go:build ignore\n\n',
     'unreadable', 'refuse-unreadable'),
    ('a byte order mark inside a header comment', b'// no' + BOM + b'te\n\n', 'unreadable',
     'refuse-unreadable'),

    # The known edge, recorded rather than hidden. Go refuses these two as well, reporting
    # `illegal character U+200B` and `illegal character U+001C`, and the gate still credits them.
    # They are not one of the four byte patterns `go_refused_bytes` knows: both are well-formed
    # UTF-8 holding a character Go's scanner will not start a token with, and that rule is Go's
    # whole lexer rather than a byte pattern a reader can check. Widening the gate to cover it
    # means transcribing that lexer, which is a bigger change than the one these rows document.
    ('a zero width space', b'\xe2\x80\x8b//go:build ignore\n\n', 'unreadable', 'accept'),
    ('a file separator U+001C', b'\x1c//go:build ignore\n\n', 'unreadable', 'accept'),

    # Refused by decision rather than by Go's rule.
    ('a `!` straight after the legacy directive', b'// +build!ignore\n\n',
     'no-constraint', 'refuse-constraint'),
    ('a `/` straight after the legacy directive', b'// +build/ignore\n\n',
     'no-constraint', 'refuse-constraint'),
    ('a word run on to the legacy directive', b'// +buildignore\n\n', 'no-constraint',
     'refuse-constraint'),
    ('the legacy form with no blank line after it', b'// +build ignore\n',
     'no-constraint', 'refuse-constraint'),
)


# What Go says when it refuses a file's bytes. Which message arrives depends on which stage
# catches the file: the reader `go/build` uses for a file's header names a NUL `unexpected NUL in
# input` and a stray mark `illegal byte order mark`, while the compiler, reached only when the
# reader got through, names the same two `invalid NUL character` and `invalid BOM in the middle of
# the file`. Any of them means the file is not compiled, which is the only distinction that
# matters here.
#
# A character Go's scanner will not start a token with is named the same way and is refused just
# as completely, so it belongs in this list although no reader ever looked at the bytes: the
# compiler says `invalid character U+000B` and `invalid character U+00A0 in identifier`, and
# `go vet` says `illegal character U+000B` for the same file.
GO_UNREADABLE_MESSAGES = (
    'illegal UTF-8 encoding',
    'invalid UTF-8 encoding',
    'unexpected NUL in input',
    'invalid NUL character',
    'illegal character NUL',
    'illegal byte order mark',
    'invalid BOM in the middle of the file',
    'illegal character U+',
    'invalid character U+',
)


def go_verdicts(headers, body=b'package p\n\nfunc F() {}\n'):
    """Ask the Go toolchain what it does with each header, in one `go list` run and a build each.

    Every header becomes its own package directory holding the header plus `body`, alongside a
    file that carries the package on its own so a header Go excludes still leaves a package to
    report. `go list -e` classifies a file without compiling it, which is exactly the constraint
    question: `IgnoredGoFiles` means a constraint excluded the file, and a `parsing //go:build
    line` error means one was read and would not parse.

    A file whose bytes Go refuses is settled first and reported `unreadable`, because for such a
    file there is no constraint question: Go never reads one. That verdict comes from `go build`
    rather than from `go list`, because `go list -e` reports only what it hit while scanning a
    file's header, and a second byte order mark is caught later, by the compiler. The builds run
    one package at a time on purpose: `go build ./...` stops at the load errors and never compiles
    the packages whose refusal only the compiler sees.
    """
    root = Path(tempfile.mkdtemp(prefix='go-header-'))
    try:
        (root / 'go.mod').write_text('module headerprobe\n\ngo 1.21\n')
        for index, header in enumerate(headers):
            package = root / ('c%d' % index)
            package.mkdir()
            (package / 'x.go').write_bytes(header + body)
            (package / 'keep.go').write_text('package p\n\nfunc Keep() {}\n')
        unreadable = set()
        for index in range(len(headers)):
            built = subprocess.run(['go', 'build', './c%d' % index],
                                   cwd=root, capture_output=True, text=True)
            if any(message in built.stderr for message in GO_UNREADABLE_MESSAGES):
                unreadable.add('c%d' % index)
        listed = subprocess.run(['go', 'list', '-e', '-json', './...'],
                                cwd=root, capture_output=True, text=True)
        decoder = json.JSONDecoder()
        verdicts = {}
        text = listed.stdout.strip()
        offset = 0
        while offset < len(text):
            info, offset = decoder.raw_decode(text, offset)
            while offset < len(text) and text[offset] in ' \t\r\n':
                offset += 1
            name = info['ImportPath'].rsplit('/', 1)[-1]
            if name in unreadable:
                verdicts[name] = 'unreadable'
            elif 'x.go' in (info.get('IgnoredGoFiles') or []):
                verdicts[name] = 'ignored'
            elif 'parsing //go:build line' in ((info.get('Error') or {}).get('Err') or ''):
                verdicts[name] = 'invalid-constraint'
            else:
                verdicts[name] = 'no-constraint'
        return [verdicts.get('c%d' % index) for index in range(len(headers))]
    finally:
        shutil.rmtree(root, ignore_errors=True)


# Every code point, as one string, so a class can be read over the whole range in one pass.
ALL_CODE_POINTS = ''.join(chr(code) for code in range(sys.maxunicode + 1))


def foreign_goos():
    """A GOOS this machine is not, so a `_GOOS` suffix carrying it is genuinely unsatisfied."""
    for name in sorted(COMPILE_CHECKER.GO_KNOWN_OS):
        if not COMPILE_CHECKER.go_platform_tag_matches(
                name, COMPILE_CHECKER.GOOS, COMPILE_CHECKER.GOARCH):
            return name
    raise AssertionError('every known GOOS matches this machine')


# A vertical tab at each of the five positions inside a registration. Go's scanner skips space,
# tab, carriage return and newline between tokens and nothing else, so every one of these files is
# illegal where the tab sits: `go test` reports `illegal character U+000B` and `[setup failed]`,
# and the package never builds. Python's `\s` matches U+000B, so the gate read all five as
# ordinary separation and credited a registration Go never compiles.
#
# `ProofDeclarationColumnTest` says what Go does with each body and `CheckCompileTest` says what
# the gate does about it, which is the split every rule on this boundary is recorded under.
VERTICAL_TAB = '\x0b'

REGISTRATION_WHITESPACE_CASES = (
    ('after `func` in the Test header',
     'func%sTestOne(t *testing.T) {\n'
     '\tproofkit.Run(t, profileCases, stepOne)\n'
     '}\n\n'
     'func stepOne() {}\n' % VERTICAL_TAB),
    ('after the header\'s opening brace',
     'func TestOne(t *testing.T) {%s\n'
     '\tproofkit.Run(t, profileCases, stepOne)\n'
     '}\n\n'
     'func stepOne() {}\n' % VERTICAL_TAB),
    ('inside the run call',
     'func TestOne(t *testing.T) {\n'
     '\tproofkit.Run(t, profileCases,%sstepOne)\n'
     '}\n\n'
     'func stepOne() {}\n' % VERTICAL_TAB),
    ('after the run\'s closing parenthesis',
     'func TestOne(t *testing.T) {\n'
     '\tproofkit.Run(t, profileCases, stepOne)%s\n'
     '}\n\n'
     'func stepOne() {}\n' % VERTICAL_TAB),
    ('after the closing brace',
     'func TestOne(t *testing.T) {\n'
     '\tproofkit.Run(t, profileCases, stepOne)\n'
     '}%s\n\n'
     'func stepOne() {}\n' % VERTICAL_TAB),
)

# A name carrying a Unicode letter, which Go's identifier rule allows and this gate must read.
# Tightening the patterns to Go is about refusing what Go refuses, and a proof that compiles,
# registers and runs under `go test` has to stay credited.
GO_LETTER_STEP = 'Prüfung'

GO_LETTER_PROOF = (
    'func Test%(title)s(t *testing.T) {\n'
    '\tproofkit.Run(t, profileCases, step%(title)s)\n'
    '}\n\n'
    'func step%(title)s() {}\n' % {'title': GO_LETTER_STEP})

# A name carrying U+00B2, which Python counts as a word character and Go refuses outright:
# `invalid character U+00B2 in identifier` from the compiler, `illegal character U+00B2` from
# `go test`. The step list and the proof agreed with each other about this name because both
# sides read it with Python's `\w`, and the registration was credited.
NON_GO_STEP = 'One\u00b2'

NON_GO_PROOF = (
    'func Test%(title)s(t *testing.T) {\n'
    '\tproofkit.Run(t, profileCases, step%(title)s)\n'
    '}\n\n'
    'func step%(title)s() {}\n' % {'title': NON_GO_STEP})


class CheckCompileTest(unittest.TestCase):
    SOURCE_PATHS = (
        'spec/gear/instructions.md',
        'spec/gear/fusion.md',
        '.claude/skills/generate-gear/PLAYBOOK.md',
    )

    def provenance_rows(self, root, paths=None):
        paths = paths or self.SOURCE_PATHS
        return '\n'.join(
            '| `%s` | `%s` |' % (path, COMPILE_CHECKER.blob_hash(str(root / path)))
            for path in paths)

    def run_checker(self, provenance=None, from_line='**From:** `spec/gear/instructions.md` L1',
                    proof_body=None, proof_filename='proof_test.go', step_body=None,
                    include_fusion=True, auxiliary=False, mutate_auxiliary=False,
                    api_lookup=None, unverified_findings=None, proof_directories=()):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / 'spec' / 'gear').mkdir(parents=True)
            (root / 'proof' / 'gear').mkdir(parents=True)
            (root / '.claude' / 'skills' / 'generate-gear').mkdir(parents=True)
            instruction_text = 'source\nline two\nline three\n'
            if auxiliary:
                instruction_text = 'source\nSee `trace.md` for details.\nline three\n'
                (root / 'spec' / 'gear' / 'trace.md').write_text('trace\n')
            (root / 'spec' / 'gear' / 'instructions.md').write_text(instruction_text)
            if include_fusion:
                (root / 'spec' / 'gear' / 'fusion.md').write_text(
                    'source\nline two\nline three\n')
            (root / '.claude' / 'skills' / 'generate-gear' / 'PLAYBOOK.md').write_text(
                'source\nline two\nline three\n')
            if provenance is None:
                paths = self.SOURCE_PATHS if include_fusion else (
                    self.SOURCE_PATHS[0], self.SOURCE_PATHS[2])
                if auxiliary:
                    paths = (*paths[:-1], 'spec/gear/trace.md', paths[-1])
                provenance = self.provenance_rows(root, paths)
            if proof_body is None:
                proof_body = self.registration('TestOne', 'proofkit.Run', 'stepOne')
            # A header case may turn on bytes no `str` survives a round trip through — a byte
            # order mark, a UTF-16 mark, a NUL — so a fixture may hand over the file as bytes.
            if isinstance(proof_body, bytes):
                (root / 'proof' / 'gear' / proof_filename).write_bytes(proof_body)
            else:
                (root / 'proof' / 'gear' / proof_filename).write_text(proof_body)
            for name in proof_directories:
                (root / 'proof' / 'gear' / name).mkdir()
            table = '\n'.join((
                '| Source | Blob hash |',
                '|---|---|',
                provenance,
            ))
            if step_body is None:
                step_body = (
                    '## S1 `[GO]` One — `stepOne`\n\n'
                    'Build the thing.\n\n'
                    '%s\n\n' % from_line)
            steps = (
                '# Steps\n\n'
                '%s'
                '## Provenance\n\n'
                '%s\n' % (step_body, table))
            (root / 'spec' / 'gear' / 'steps.md').write_text(steps)
            if mutate_auxiliary:
                (root / 'spec' / 'gear' / 'trace.md').write_text('changed\n')
            output = io.StringIO()
            prior = os.getcwd()
            try:
                os.chdir(root)
                lookup_patch = mock.patch.object(
                    COMPILE_CHECKER.fusion_api, 'lookup_many',
                    return_value={} if api_lookup is None else api_lookup)
                findings_patch = mock.patch.object(
                    COMPILE_CHECKER.fusion_api, 'unverified_findings',
                    return_value=[] if unverified_findings is None else unverified_findings)
                with lookup_patch, findings_patch, contextlib.redirect_stdout(output):
                    result = COMPILE_CHECKER.main(['check_compile.py', 'gear'])
            finally:
                os.chdir(prior)
            return result, output.getvalue()

    def registration(self, test, call, build, cases='profileCases', extra=''):
        """The canonical three-line registration, plus the step definition it names.

        Every argument is a knob a test turns to step one way off the shape, so a fixture that
        only differs in the build argument reads as exactly that difference.
        """
        return (
            'func %s(t *testing.T) {\n'
            '\t%s(t, %s, %s%s)\n'
            '}\n\n'
            'func stepOne() {}\n' % (test, call, cases, build, extra))

    def claim_step(self, claim):
        """One `[GO]` step whose title carries `claim`, with citation and body canonical.

        The title is where the committed step lists write the proof function, so a fixture that
        only differs in what is claimed reads as exactly that difference.
        """
        return ('## S1 `[GO]` One — `%s`\n\n'
                'Build the thing.\n\n'
                '**From:** `spec/gear/instructions.md` L1\n\n' % claim)

    def body(self, statements, test='TestTwo'):
        """A proof whose first registration is canonical and whose second Test is `statements`."""
        return (
            'func TestOne(t *testing.T) {\n'
            '\tproofkit.Run(t, profileCases, stepOne)\n'
            '}\n\n'
            'func %s(t *testing.T) {\n'
            '%s\n'
            '}\n\n'
            'func stepOne() {}\n' % (test, statements))

    # The canonical shape, in each of the four run forms.

    def test_canonical_2d_registration_is_accepted(self):
        result, output = self.run_checker()

        self.assertEqual(result, 0, output)
        self.assertIn('compile check: OK', output)

    def test_canonical_3d_registrations_are_accepted(self):
        for call, extra in (('proofkit3d.Run', ', assertOne'),
                            ('proofkit3d.RunSolid', ', assertOne'),
                            ('proofkit3d.RunWithGate', ', proofkit3d.RequireSolid, assertOne')):
            with self.subTest(call=call):
                proof_body = self.registration('TestOne', call, 'stepOne', extra=extra)

                result, output = self.run_checker(proof_body=proof_body)

                self.assertEqual(result, 0, output)
                self.assertIn('compile check: OK', output)

    def test_gofmt_spacing_is_not_required(self):
        """The shape is matched by line, not by byte, so spacing inside the call is free."""
        proof_body = (
            'func TestOne(t *testing.T) {\n'
            '    proofkit . Run( t ,  profileCases ,  stepOne )\n'
            '}\n\n'
            'func stepOne() {}\n')

        result, output = self.run_checker(proof_body=proof_body)

        self.assertEqual(result, 0, output)

    # Only the four run methods that exist are read as registrations.

    def test_run_method_no_package_defines_is_refused(self):
        """`proofkit` defines only `Run`; the two variants live in `proofkit3d` alone.

        A gate that let the namespace and the suffix vary on their own credited a step whose
        registration Go cannot compile, which is the one direction a compile gate must never fail
        in.

        The complaint names the method rather than the shape, because the three lines here are
        well formed and it is the method that does not exist. Go says the same of this pairing:
        `undefined: proofkit.RunSolid`.
        """
        proof_body = self.registration('TestOne', 'proofkit.RunSolid', 'stepOne',
                                       extra=', assertOne')

        result, output = self.run_checker(proof_body=proof_body)

        self.assertEqual(result, 1)
        self.assertIn('proof/gear/proof_test.go:2 calls proofkit.RunSolid, which no harness '
                      'package declares', output)
        self.assertIn(
            'S1 names proof function stepOne, which TestOne does not build with', output)

    def test_call_to_a_harness_method_that_does_not_exist_is_named(self):
        """A run method that does not exist, called outside a registration, was invisible.

        The mention pattern was built from the derived set, so a name outside that set could not
        match it, and the pattern meant to catch a run written outside the accepted shape was
        blind to exactly the case its own comment named. `go vet` reports this file
        `undefined: proofkit3d.RunTypo`.

        The same typo written inside the three-line shape was already caught indirectly, because
        the step then goes unregistered. The hole was the call that is not itself a step's sole
        registration.
        """
        proof_body = (
            'func TestOne(t *testing.T) {\n'
            '\tproofkit.Run(t, profileCases, stepOne)\n'
            '}\n\n'
            'func helper(t *testing.T) {\n'
            '\tproofkit3d.RunTypo(t, profileCases, stepOne)\n'
            '}\n\n'
            'func stepOne() {}\n')

        result, output = self.run_checker(proof_body=proof_body)

        self.assertEqual(result, 1)
        self.assertIn('proof/gear/proof_test.go:6 calls proofkit3d.RunTypo, which no harness '
                      'package declares', output)
        # The three lines are not the defect and are not reported as one.
        self.assertEqual(output.count('proof/gear/proof_test.go:6'), 1, output)

    def test_harness_calls_that_are_not_runs_stay_silent(self):
        """The `Run` prefix is the whole boundary, so every other harness call must pass through.

        These five helpers appear throughout ordinary step bodies. A pattern that reached them
        would turn every proof in the repository into a wall of complaints about calls Go
        compiles.
        """
        proof_body = (
            'func TestOne(t *testing.T) {\n'
            '\tproofkit.Run(t, profileCases, stepOne)\n'
            '}\n\n'
            'func stepOne(t *testing.T) {\n'
            '\tproofkit.Step(t, "one")\n'
            '\tproofkit.Unmodelled(t, "two")\n'
            '\tproofkit.RequireSound(t, s)\n'
            '\tproofkit3d.RequireSolid(t, doc, bodies)\n'
            '\tproofkit3d.BodyReport(t, doc, body)\n'
            '}\n')

        result, output = self.run_checker(proof_body=proof_body)

        self.assertEqual(result, 0, output)
        self.assertIn('compile check: OK', output)

    # Each run takes the arguments its own signature declares, and no others.

    def test_wrong_argument_count_on_a_real_method_is_refused(self):
        """A tail of any length credited calls Go cannot compile, on every one of the four runs.

        `proofkit.Run` takes three arguments, `proofkit3d.Run` and `RunSolid` four, and
        `RunWithGate` five. A shape that accepted three or more read a step as registered by a run
        that never builds, which is the direction a compile gate must never fail in.
        """
        cases = (
            ('proofkit.Run', ', assertOne', 4, 3),
            ('proofkit.Run', ', proofkit3d.RequireSolid, assertOne', 5, 3),
            ('proofkit3d.Run', '', 3, 4),
            ('proofkit3d.Run', ', proofkit3d.RequireSolid, assertOne', 5, 4),
            ('proofkit3d.RunSolid', '', 3, 4),
            ('proofkit3d.RunWithGate', ', assertOne', 4, 5),
            ('proofkit3d.RunWithGate', '', 3, 5),
        )
        for call, extra, passed, declared in cases:
            with self.subTest(call=call, passed=passed):
                proof_body = self.registration('TestOne', call, 'stepOne', extra=extra)

                result, output = self.run_checker(proof_body=proof_body)

                self.assertEqual(result, 1, output)
                self.assertIn(
                    'proof/gear/proof_test.go:2 passes %d arguments to %s, which takes %d, so Go '
                    'cannot compile this registration' % (passed, call, declared), output)
                self.assertIn(
                    'S1 names proof function stepOne, which TestOne does not build with', output)

    def test_declared_argument_count_on_each_method_is_accepted(self):
        """The counts are the signatures in `proof/proofkit` and `proof/proofkit3d`."""
        for call, extra in (('proofkit.Run', ''),
                            ('proofkit3d.Run', ', assertOne'),
                            ('proofkit3d.RunSolid', ', assertOne'),
                            ('proofkit3d.RunWithGate', ', proofkit3d.RequireSolid, assertOne')):
            with self.subTest(call=call):
                proof_body = self.registration('TestOne', call, 'stepOne', extra=extra)

                result, output = self.run_checker(proof_body=proof_body)

                self.assertEqual(result, 0, output)
                self.assertIn('compile check: OK', output)

    def test_wrong_argument_count_is_named_once_at_its_own_line(self):
        """A refused run is named exactly once, so the count complaint is the whole story.

        The wider mention pattern still sees the line, so widening it further is safe; it is
        skipped here only because the count complaint already names the same line.
        """
        proof_body = self.registration('TestOne', 'proofkit.Run', 'stepOne', extra=', assertOne')

        result, output = self.run_checker(proof_body=proof_body)

        self.assertEqual(result, 1)
        self.assertEqual(output.count('proof/gear/proof_test.go:2'), 1, output)

    # The build argument must be the step of the Test's own title.

    def test_misnamed_build_argument_is_blocking(self):
        proof_body = self.body('\tproofkit3d.Run(t, solidCases, buildSolid, assertSolid)')

        result, output = self.run_checker(proof_body=proof_body)

        self.assertEqual(result, 1)
        self.assertIn(
            'proof/gear/proof_test.go:6 registers buildSolid as a proof run\'s build argument, '
            'but that argument must be a step<Title> function so a step can claim it', output)

    def test_crossed_registration_is_blocking(self):
        """A Test that builds with some other step is the failure name-matching exists to catch."""
        proof_body = (
            'func TestOne(t *testing.T) {\n'
            '\tproofkit.Run(t, profileCases, stepTwo)\n'
            '}\n\n'
            'func stepOne() {}\n\n'
            'func stepTwo() {}\n')

        result, output = self.run_checker(proof_body=proof_body)

        self.assertEqual(result, 1)
        self.assertIn(
            'proof/gear/proof_test.go:2 registers stepTwo inside TestOne, but a step is '
            'registered by the Test of its own title, so this build belongs in TestTwo', output)

    def test_step_the_matching_test_does_not_build_is_blocking(self):
        proof_body = (
            'func TestOne(t *testing.T) {\n'
            '\tproofkit.Run(t, profileCases, stepOne)\n'
            '}\n\n'
            'func stepOne() {}\n\n'
            'func stepTwo() {}\n')

        result, output = self.run_checker(proof_body=proof_body)

        self.assertEqual(result, 1)
        self.assertIn('proof function stepTwo is not claimed by any step', output)

    def test_claimed_step_with_no_registration_is_blocking(self):
        proof_body = 'func stepOne() {}\n'

        result, output = self.run_checker(proof_body=proof_body)

        self.assertEqual(result, 1)
        self.assertIn(
            'S1 names proof function stepOne, which TestOne does not build with', output)

    # Everything off the shape is refused rather than read. These are the shapes that cost the
    # brace-matching reader its rounds; here each one is one line of expected output.

    def test_run_in_a_condition_is_refused(self):
        proof_body = self.body(
            '\tif solid {\n'
            '\t\tproofkit3d.Run(t, solidCases, stepTwo, assertTwo)\n'
            '\t}')

        result, output = self.run_checker(proof_body=proof_body)

        self.assertEqual(result, 1)
        self.assertIn('proof/gear/proof_test.go:7 runs a proof outside the shape this gate reads',
                      output)

    def test_run_in_a_loop_is_refused(self):
        proof_body = self.body(
            '\tfor _, c := range []struct{ n int }{{1}} {\n'
            '\t\tproofkit3d.Run(t, solidCases, stepTwo, assertTwo)\n'
            '\t}')

        result, output = self.run_checker(proof_body=proof_body)

        self.assertEqual(result, 1)
        self.assertIn('runs a proof outside the shape this gate reads', output)

    def test_run_in_a_closure_is_refused(self):
        proof_body = self.body(
            '\tt.Run("one", func(t *testing.T) {\n'
            '\t\tproofkit3d.Run(t, solidCases, stepTwo, assertTwo)\n'
            '\t})')

        result, output = self.run_checker(proof_body=proof_body)

        self.assertEqual(result, 1)
        self.assertIn('runs a proof outside the shape this gate reads', output)

    def test_forwarded_argument_list_is_refused(self):
        proof_body = self.body('\tproofkit3d.Run(runArgs(t))')

        result, output = self.run_checker(proof_body=proof_body)

        self.assertEqual(result, 1)
        self.assertIn('proof/gear/proof_test.go:6 runs a proof outside the shape this gate reads, '
                      'so its build argument cannot be checked', output)

    def test_case_table_built_in_place_is_refused(self):
        """The table is a named variable, so the run stays one line the gate can match."""
        proof_body = self.body(
            '\tproofkit.Run(t, cases(\n'
            '\t\tgear{name: "two"},\n'
            '\t), stepTwo)')

        result, output = self.run_checker(proof_body=proof_body)

        self.assertEqual(result, 1)
        self.assertIn('runs a proof outside the shape this gate reads', output)

    def test_extra_statement_in_the_test_body_is_refused(self):
        proof_body = self.body(
            '\tt.Parallel()\n'
            '\tproofkit.Run(t, profileCases, stepTwo)')

        result, output = self.run_checker(proof_body=proof_body)

        self.assertEqual(result, 1)
        self.assertIn('runs a proof outside the shape this gate reads', output)

    def test_run_outside_any_test_is_refused(self):
        proof_body = (
            'func TestOne(t *testing.T) {\n'
            '\tproofkit.Run(t, profileCases, stepOne)\n'
            '}\n\n'
            'func register(t *testing.T) {\n'
            '\tproofkit.Run(t, profileCases, stepTwo)\n'
            '}\n\n'
            'func stepOne() {}\n')

        result, output = self.run_checker(proof_body=proof_body)

        self.assertEqual(result, 1)
        self.assertIn('proof/gear/proof_test.go:6 runs a proof outside the shape this gate reads',
                      output)

    def test_test_name_go_would_not_run_is_refused(self):
        """`go test` runs Test followed by a non-lower rune, and this gate never has to know that.

        It looks for the title it expects instead of classifying the name it finds, so a name Go
        rejects simply fails to be a registration and its run is reported.
        """
        proof_body = (
            'func TestOne(t *testing.T) {\n'
            '\tproofkit.Run(t, profileCases, stepOne)\n'
            '}\n\n'
            'func Testé(t *testing.T) {\n'
            '\tproofkit.Run(t, profileCases, stepTwo)\n'
            '}\n\n'
            'func stepOne() {}\n')

        result, output = self.run_checker(proof_body=proof_body)

        self.assertEqual(result, 1)
        self.assertIn('proof/gear/proof_test.go:6 runs a proof outside the shape this gate reads',
                      output)

    # Comments and literals are blanked first, so a quoted registration is not a real one.

    def test_registration_inside_a_comment_does_not_count(self):
        proof_body = (
            '/*\n'
            'func TestOne(t *testing.T) {\n'
            '\tproofkit.Run(t, profileCases, stepOne)\n'
            '}\n'
            '*/\n\n'
            'func stepOne() {}\n')

        result, output = self.run_checker(proof_body=proof_body)

        self.assertEqual(result, 1)
        self.assertIn(
            'S1 names proof function stepOne, which TestOne does not build with', output)

    def test_registration_inside_a_raw_string_does_not_count(self):
        proof_body = (
            'var sample = `\n'
            'func TestOne(t *testing.T) {\n'
            '\tproofkit.Run(t, profileCases, stepOne)\n'
            '}\n'
            '`\n\n'
            'func stepOne() {}\n')

        result, output = self.run_checker(proof_body=proof_body)

        self.assertEqual(result, 1)
        self.assertIn(
            'S1 names proof function stepOne, which TestOne does not build with', output)

    # Two rules Go owns that no reader of source text can infer, refused rather than copied.

    def test_build_constraint_is_blocking(self):
        proof_body = (
            '//go:build ignore\n\n'
            'package gear_test\n\n'
            'func TestOne(t *testing.T) {\n'
            '\tproofkit.Run(t, profileCases, stepOne)\n'
            '}\n\n'
            'func stepOne() {}\n')

        result, output = self.run_checker(proof_body=proof_body)

        self.assertEqual(result, 1)
        self.assertIn(
            'proof/gear/proof_test.go:1 carries a build constraint, so whether Go ever compiles '
            'these registrations is decided outside the file', output)

    def test_a_byte_order_mark_does_not_hide_a_constraint(self):
        """Go strips one leading mark, so `EF BB BF` above `//go:build ignore` still excludes.

        `go list` reports such a file in `IgnoredGoFiles` with no `TestGoFiles` at all. Reading
        the bytes as they sit left line 1 beginning with U+FEFF, which no `//` match reaches, so
        the gate credited every registration in a file Go never compiles. The mark is removed as
        a character rather than as a line, so the constraint is still reported at line 1.
        """
        proof_body = b'\xef\xbb\xbf//go:build ignore\n\n' + self.PROOF_BODY

        result, output = self.run_checker(proof_body=proof_body)

        self.assertEqual(result, 1)
        self.assertIn('proof/gear/proof_test.go:1 carries a build constraint', output)

    def test_a_byte_order_mark_does_not_shift_the_lines_below_it(self):
        """Removing the mark costs no line, so a constraint on line 2 is reported at line 2."""
        proof_body = b'\xef\xbb\xbf// note\n//go:build ignore\n\n' + self.PROOF_BODY

        result, output = self.run_checker(proof_body=proof_body)

        self.assertEqual(result, 1)
        self.assertIn('proof/gear/proof_test.go:2 carries a build constraint', output)

    def test_a_constraint_below_a_closed_block_comment_is_reported_at_its_own_line(self):
        """Go reads it — the block comment closed, so the directive line is a `//` line again.

        `go list` reports this file in `IgnoredGoFiles`, and the complaint names line 4, which is
        where the directive sits.
        """
        proof_body = b'/*\nnote\n*/\n//go:build ignore\n\n' + self.PROOF_BODY

        result, output = self.run_checker(proof_body=proof_body)

        self.assertEqual(result, 1)
        self.assertIn('proof/gear/proof_test.go:4 carries a build constraint', output)

    def test_build_constraint_text_inside_a_raw_string_is_accepted(self):
        """Go reads a constraint only above the package clause, so lower down it is just text.

        Scanning every raw line refused this file, which `gofmt` and `go test` both accept.
        """
        proof_body = (
            'package gear_test\n\n'
            'var sample = `\n'
            '//go:build ignore\n'
            '`\n\n'
            'func TestOne(t *testing.T) {\n'
            '\tproofkit.Run(t, profileCases, stepOne)\n'
            '}\n\n'
            'func stepOne() {}\n')

        result, output = self.run_checker(proof_body=proof_body)

        self.assertEqual(result, 0, output)
        self.assertIn('compile check: OK', output)

    def test_build_constraint_text_inside_a_header_block_comment_is_accepted(self):
        """Go takes a constraint only from a `//` line comment, so block-comment prose is text.

        A file whose leading `/* */` note quotes a constraint indented inside it builds, vets and
        formats clean, and `go list` reports it unconstrained. Reading every raw header line
        refused it.

        The quote is indented because that is the form Go genuinely accepts. Written at column 1
        inside a leading block comment, `go/build` still ignores it, but `go vet` calls the
        directive misplaced and `go test` fails, so this gate accepting it would agree with no
        real toolchain and that case is deliberately not asserted here.
        """
        proof_body = (
            '/*\n'
            'Package gear_test proves the gear steps.\n'
            '\n'
            'A file that is not built would say\n'
            '\n'
            '\t//go:build ignore\n'
            '\n'
            'above its package clause. This proof is built, so it says no such thing.\n'
            '*/\n'
            'package gear_test\n\n'
            'func TestOne(t *testing.T) {\n'
            '\tproofkit.Run(t, profileCases, stepOne)\n'
            '}\n\n'
            'func stepOne() {}\n')

        result, output = self.run_checker(proof_body=proof_body)

        self.assertEqual(result, 0, output)
        self.assertIn('compile check: OK', output)

    def test_registration_outside_a_test_file_is_blocking(self):
        result, output = self.run_checker(proof_filename='proof.go')

        self.assertEqual(result, 1)
        self.assertIn(
            'proof/gear/proof.go:2 registers stepOne, but `go test` only runs tests in a '
            '_test.go file, so nothing here ever builds it', output)

    def test_each_off_shape_run_is_reported_at_its_own_line(self):
        proof_body = (
            'func TestOne(t *testing.T) {\n'
            '\tproofkit.Run(t, profileCases, stepOne)\n'
            '}\n\n'
            'func TestTwo(t *testing.T) {\n'
            '\tproofkit3d.Run(runArgs(t))\n'
            '}\n\n'
            'func TestThree(t *testing.T) {\n'
            '\tproofkit3d.RunSolid(runArgs(t))\n'
            '}\n\n'
            'func stepOne() {}\n')

        result, output = self.run_checker(proof_body=proof_body)

        self.assertEqual(result, 1)
        self.assertIn('proof/gear/proof_test.go:6 runs a proof', output)
        self.assertIn('proof/gear/proof_test.go:10 runs a proof', output)


    # Only files Go actually compiles are credited.

    def test_file_go_never_compiles_is_named_not_credited(self):
        """A proof file Go drops still parsed as a proof, so its steps were credited unbuilt.

        `_x_test.go` and `.x_test.go` are invisible to `go/build`, and a `_GOOS` or `_GOARCH`
        suffix this machine does not satisfy lands the file in `IgnoredGoFiles`. In all three
        cases `go test` reports no test files, so nothing here registers anything.
        """
        foreign = foreign_goos()
        for filename in ('_x_test.go', '.x_test.go', 'proof_%s_test.go' % foreign,
                         'proof_%s_amd64_test.go' % foreign):
            with self.subTest(filename=filename):
                result, output = self.run_checker(proof_filename=filename)

                self.assertEqual(result, 1, output)
                self.assertIn('proof/gear/%s is never compiled by Go' % filename, output)
                self.assertIn(
                    'S1 names proof function stepOne, which proof/gear/ does not declare as a '
                    'function', output)

    def test_file_go_does_compile_is_still_credited(self):
        """The exclusion must not reach a satisfied suffix or an ordinary trailing word."""
        for filename in ('proof_helper_test.go', 'proof_solid_test.go', 'a_unix_test.go',
                         'a_Windows_test.go', 'windows_test.go',
                         'proof_%s_test.go' % COMPILE_CHECKER.GOOS,
                         'proof_%s_test.go' % COMPILE_CHECKER.GOARCH):
            with self.subTest(filename=filename):
                result, output = self.run_checker(proof_filename=filename)

                self.assertEqual(result, 0, output)
                self.assertIn('compile check: OK', output)

    def test_directory_named_go_does_not_crash_the_gate(self):
        """Go builds a package around a directory called `*.go`; reading it as a file raises."""
        result, output = self.run_checker(proof_directories=('bundle.go', 'nested_test.go'))

        self.assertEqual(result, 0, output)
        self.assertIn('compile check: OK', output)

    # The build-constraint boundary, one row per spelling. `GO_HEADER_CASES` at the top of this
    # file records what Go does with each header and what the gate does about it; these three
    # tests are the two halves of every row and the toolchain run that keeps the `go` column
    # honest.

    PROOF_BODY = (b'package gear_test\n\n'
                  b'func TestOne(t *testing.T) {\n'
                  b'\tproofkit.Run(t, profileCases, stepOne)\n'
                  b'}\n\n'
                  b'func stepOne() {}\n')

    def constrained(self, header):
        return header + self.PROOF_BODY

    def test_every_header_go_reads_as_a_constraint_is_refused(self):
        """Every spelling Go excludes or cannot parse fails the check, plus four by decision.

        Go honours `//go:build` only with nothing between the slashes and the directive and with
        whitespace or the end of the line after it, and honours the legacy `+build` form more
        loosely. The four rows Go builds are refused anyway: a proof file needs no build
        constraint in any form, so a `+build` near-miss costs one message rather than a silent
        credit.
        """
        for name, header, verdict, gate in GO_HEADER_CASES:
            if gate != 'refuse-constraint':
                continue
            with self.subTest(case=name, go=verdict):
                result, output = self.run_checker(proof_body=self.constrained(header))

                self.assertEqual(result, 1, output)
                self.assertIn('carries a build constraint', output)

    def test_every_header_whose_bytes_go_refuses_is_refused(self):
        """A file Go will not read registers nothing, so crediting one is a false pass.

        Four byte patterns do it and each row above names which: a UTF-16 mark, any other bytes
        that are not UTF-8, a NUL, and a byte order mark below the first character. Reading such a
        file as text credited every registration in it with no complaint at all, and for the
        UTF-16 mark it also hid the build constraint underneath, because the two replacement
        characters pushed the line off its `//` start. The complaint wording and the position it
        points at are pinned one pattern at a time below.
        """
        for name, header, verdict, gate in GO_HEADER_CASES:
            if gate != 'refuse-unreadable':
                continue
            with self.subTest(case=name, go=verdict):
                result, output = self.run_checker(proof_body=self.constrained(header))

                self.assertEqual(result, 1, output)
                self.assertIn('so Go refuses the file', output)
                self.assertNotIn('compile check: OK', output)

    def test_every_header_go_builds_as_written_is_accepted(self):
        """Every spelling Go reads no constraint from passes, so the gate refuses no built file.

        These are the near-misses and the disguises: a space after the slashes, a `!` or a `/`
        straight after the directive, and a directive quoted inside a leading block comment.
        `go list` reports all of them in `GoFiles`, with no constraint read, and a gate that
        refused any of them would be failing a proof Go compiles.

        Two rows here are accepted while Go refuses the file, for an illegal character rather than
        for a byte pattern; the corpus comment says why the gate does not reach them.
        """
        for name, header, verdict, gate in GO_HEADER_CASES:
            if gate != 'accept':
                continue
            with self.subTest(case=name, go=verdict):
                result, output = self.run_checker(proof_body=self.constrained(header))

                self.assertEqual(result, 0, output)
                self.assertIn('compile check: OK', output)

    def test_a_directive_inside_an_unterminated_block_comment_is_not_a_constraint(self):
        """The comment never closes, so Go is still inside it at the directive and reads none.

        `go list` puts such a file in `GoFiles` and reports `comment not terminated`: it is
        broken, but not by a build constraint, and saying so would send the drafter to the wrong
        line. It sits outside `GO_HEADER_CASES` because the open comment swallows the
        registrations too, so the check has other things to say about the file.
        """
        proof_body = b'/*\n//go:build ignore\n' + self.PROOF_BODY

        result, output = self.run_checker(proof_body=proof_body)

        self.assertEqual(result, 1)
        self.assertNotIn('carries a build constraint', output)

    @unittest.skipUnless(shutil.which('go'), 'the Go toolchain establishes these verdicts')
    def test_go_agrees_with_every_recorded_header_verdict(self):
        """The `go` column of every row is what the toolchain actually does with that header.

        The gate's rules are transcribed from `go/build`, and a transcription is only worth what
        keeps it true. This runs `go build` and `go list -e -json` over the whole corpus and
        reconciles it row by row, so a Go release that changed the boundary, or a row written from
        memory, fails here rather than in a proof nobody can explain. It is what keeps `unreadable`
        honest as well: that column claims Go refuses the file, and this is the run that shows it.
        """
        headers = [header for _, header, _, _ in GO_HEADER_CASES]

        observed = go_verdicts(headers)

        for (name, _, verdict, _), actual in zip(GO_HEADER_CASES, observed):
            with self.subTest(case=name):
                self.assertEqual(actual, verdict)

    # Bytes Go refuses to read, one pattern at a time: what the complaint says and where it
    # points. The corpus above carries every one of these as a header row, reconciled against the
    # toolchain; what a header row cannot carry is the wording, the position, and a pattern that
    # only appears below the package clause, which is what these add.

    UNREADABLE_TAIL = ('so Go refuses the file: `go test` never compiles it, and nothing it '
                       'registers is ever built; write the file ')

    def assert_refused(self, proof_body, position, reason, remedy):
        result, output = self.run_checker(proof_body=proof_body)

        self.assertEqual(result, 1, output)
        self.assertIn('proof/gear/proof_test.go:%s %s, %s%s'
                      % (position, reason, self.UNREADABLE_TAIL, remedy), output)
        self.assertNotIn('compile check: OK', output)
        return output

    def test_a_utf16_byte_order_mark_is_refused_at_the_first_byte(self):
        """`go build` fails with `illegal UTF-8 encoding (got UTF-16)` at 1:1, and nothing builds.

        Reading the bytes with `errors='replace'` credited the registrations in such a file, and
        the two replacement characters also pushed the header off its `//` start, so the build
        constraint under the mark went unreported as well. Both marks are named, since a file
        saved as UTF-16 opens with whichever one the editor writes.
        """
        for mark in (b'\xff\xfe', b'\xfe\xff'):
            with self.subTest(mark=mark):
                self.assert_refused(
                    mark + b'//go:build ignore\n\n' + self.PROOF_BODY, '1:1',
                    'opens with a UTF-16 byte order mark, which Go reports as `illegal UTF-8 '
                    'encoding (got UTF-16)`', 'as UTF-8 rather than UTF-16')

    def test_bytes_that_are_not_utf8_are_refused_where_they_sit(self):
        """A stray `0xFF` is `illegal UTF-8 encoding` to Go, in a comment and in a literal alike.

        The literal case is here rather than in `GO_HEADER_CASES`, whose rows are headers above
        the package clause: this byte sits below it, and Go still refuses the file.
        """
        self.assert_refused(
            b'// no\xffte\n' + self.PROOF_BODY, '1:6',
            'holds bytes that are not UTF-8, which Go reports as `illegal UTF-8 encoding`',
            'as UTF-8')
        self.assert_refused(
            self.PROOF_BODY + b'\nvar note = "no\xffte"\n', '9:15',
            'holds bytes that are not UTF-8, which Go reports as `illegal UTF-8 encoding`',
            'as UTF-8')

    def test_a_nul_byte_is_refused_wherever_it_sits(self):
        """A NUL decodes as valid UTF-8, so a strict decode alone credits the file.

        Go refuses it either way, reporting `unexpected NUL in input` from the reader that walks a
        file's header and `invalid NUL character` from the compiler for one further down. The
        literal case is here rather than in `GO_HEADER_CASES` because it sits below the package
        clause, which a header row cannot express.
        """
        self.assert_refused(
            b'// no\x00te\n' + self.PROOF_BODY, '1:6',
            'holds a NUL byte, which Go reports as `unexpected NUL in input`',
            'without that byte')
        self.assert_refused(
            self.PROOF_BODY + b'\nvar note = "no\x00te"\n', '9:15',
            'holds a NUL byte, which Go reports as `unexpected NUL in input`',
            'without that byte')

    def test_a_byte_order_mark_below_the_first_character_is_refused(self):
        """Only the very first mark is stripped; any other one is `illegal byte order mark`.

        A mark below the first character decodes as valid UTF-8 too, so this is the second case a
        strict decode does not catch. The scan starts below a stripped leading mark, so a second
        mark is reported at column 4, which is where Go reports it.
        """
        self.assert_refused(
            BOM + BOM + b'//go:build ignore\n\n' + self.PROOF_BODY, '1:4',
            'holds a byte order mark below the first character, which Go reports as `illegal '
            'byte order mark`', 'without that mark')
        self.assert_refused(
            b'// note\n' + BOM + b'//go:build ignore\n\n' + self.PROOF_BODY, '2:1',
            'holds a byte order mark below the first character, which Go reports as `illegal '
            'byte order mark`', 'without that mark')

    def test_the_first_refused_pattern_in_the_file_is_the_one_reported(self):
        """One complaint per file, pointing at the earliest bytes Go stops on."""
        output = self.assert_refused(
            b'// note\x00and\xffmore\n' + self.PROOF_BODY, '1:8',
            'holds a NUL byte, which Go reports as `unexpected NUL in input`',
            'without that byte')

        self.assertNotIn('not UTF-8', output)

    def test_a_leading_byte_order_mark_is_still_stripped_and_the_file_scanned(self):
        """The one mark Go strips stays stripped, and the file under it is read normally.

        `test_a_byte_order_mark_does_not_hide_a_constraint` and the test below it hold the line
        numbers; this holds the acceptance, so refusing every other mark did not take this one
        with it.
        """
        result, output = self.run_checker(proof_body=BOM + self.PROOF_BODY)

        self.assertEqual(result, 0, output)
        self.assertIn('compile check: OK', output)

    def test_no_byte_pattern_raises_out_of_the_gate(self):
        """Every refusal is a complaint, never a traceback.

        `errors='replace'` was put in to stop a UTF-16 mark raising `UnicodeDecodeError` and
        crashing the run, and decoding strictly without catching the failure would bring that
        crash back. The gate has to survive an arbitrary byte anywhere, so this walks a file with
        one byte of every value in it, at three positions, and asks that the read either hands
        back text or hands back a complaint. It goes through `go_source` rather than through the
        whole checker because that is the one place a byte is decoded, and 768 whole runs cost
        more than the suite should pay to say the same thing.
        """
        with tempfile.TemporaryDirectory() as directory:
            path = os.path.join(directory, 'proof_test.go')
            for value in range(256):
                byte = bytes([value])
                for where, data in (('above', byte + self.PROOF_BODY),
                                    ('in a comment', b'// note ' + byte + b'\n' + self.PROOF_BODY),
                                    ('in a literal',
                                     self.PROOF_BODY + b'\nvar note = "x' + byte + b'"\n')):
                    with self.subTest(value=value, where=where):
                        with open(path, 'wb') as handle:
                            handle.write(data)

                        text, refused = COMPILE_CHECKER.go_source(path)

                        self.assertEqual(text is None, refused is not None)

    def test_an_undecodable_file_is_one_complaint_and_no_registration(self):
        """The whole run returns, with the file named and nothing in it credited.

        Reading the same bytes as text returned the file's registrations with no complaint, which
        is the false pass; before that, decoding them strictly crashed the run.
        """
        result, output = self.run_checker(proof_body=b'\xff\xfe' + self.PROOF_BODY)

        self.assertEqual(result, 1, output)
        self.assertIn('compile check: BLOCKING (2)', output)
        self.assertIn('proof/gear/proof_test.go:1:1 opens with a UTF-16 byte order mark', output)
        self.assertIn('S1 names proof function stepOne, which proof/gear/ does not declare as a '
                      'function', output)

    # Lines are cut where Go cuts them, at `\n` and nowhere else.

    def test_form_feed_in_a_header_comment_does_not_invent_a_constraint(self):
        """Go ends a line only at `\\n`, so this header is one comment and the file is built.

        Python's `splitlines` also ends a line at a form feed, which split this comment in two
        and refused the tail as a build constraint the file does not carry.
        """
        proof_body = (
            '// see \x0c//go:build ignore\n'
            'package gear_test\n\n'
            'func TestOne(t *testing.T) {\n'
            '\tproofkit.Run(t, profileCases, stepOne)\n'
            '}\n\n'
            'func stepOne() {}\n')

        result, output = self.run_checker(proof_body=proof_body)

        self.assertEqual(result, 0, output)
        self.assertIn('compile check: OK', output)

    def test_form_feed_does_not_shift_the_header_a_constraint_is_read_from(self):
        """A real constraint stays reported, at its own line, after a form feed above it.

        Splitting the raw source more finely than the blanked copy slid the two out of step, so
        the header slice stopped at the comment and a genuine `//go:build` below it went unseen.
        """
        proof_body = (
            '// note \x0c and more\n'
            '//go:build ignore\n'
            'package gear_test\n\n'
            'func TestOne(t *testing.T) {\n'
            '\tproofkit.Run(t, profileCases, stepOne)\n'
            '}\n\n'
            'func stepOne() {}\n')

        result, output = self.run_checker(proof_body=proof_body)

        self.assertEqual(result, 1, output)
        self.assertIn('proof/gear/proof_test.go:2 carries a build constraint', output)

    def test_the_proof_scanner_cuts_lines_only_at_a_newline(self):
        """Every line the gate numbers is cut at `\n`, the only line ending Go recognises.

        Python's `splitlines` also cuts at a form feed, a vertical tab, U+001C to U+001E, U+0085,
        U+2028 and U+2029. The two tests above show the difference through the gate's output,
        because a `//` comment survives into the copy the header is read from. The scrubbed copy
        the registrations are read from cannot show it, since every one of those characters is
        illegal in Go source outside a comment or a literal and both are blanked before the
        split. It is pinned here anyway: a reader that numbers lines differently from Go cannot
        be trusted to point at the line it names, and every `path:line` complaint rests on that
        arithmetic.
        """
        body = inspect.getsource(COMPILE_CHECKER.scan_proof_file).split('"""')[-1]

        self.assertNotIn('splitlines', body)
        self.assertIn("split('\\n')", body)

    # A step is read from a function declaration, and the refusal says so.

    def test_step_bound_to_a_variable_is_reported_as_not_a_function(self):
        """Go compiles `var stepFoo = func(…)`, so calling the directory undefining it misleads.

        The gate keeps requiring the `func` form, matching the one-function-per-step contract in
        `.claude/skills/compile-gear/prompt.md`. Only the message changed, to name the shape the
        step has to be written in.
        """
        proof_body = (
            'func TestOne(t *testing.T) {\n'
            '\tproofkit.Run(t, profileCases, stepOne)\n'
            '}\n\n'
            'var stepOne = func(t *testing.T) {}\n')

        result, output = self.run_checker(proof_body=proof_body)

        self.assertEqual(result, 1, output)
        self.assertIn(
            'S1 names proof function stepOne, which proof/gear/ does not declare as a function; '
            'write `func stepOne(...)`, since a step bound to a variable is not read', output)
        self.assertNotIn('which proof/gear/ does not define', output)

    # Where a declaration may start on its line, pattern by pattern.
    #
    # Go's layout is free and nothing in this repository holds a proof file to `gofmt`, so every
    # line below can arrive indented and Go still compiles it. Two of the four patterns that read
    # such a line are widened to match Go, and two keep a column-1 anchor because their line is
    # part of the registration shape `.claude/skills/compile-gear/prompt.md` states to the drafter.
    # `ProofDeclarationColumnTest` holds the Go half of each pairing; these hold the gate half, so
    # neither decision can be reversed without a failure here.

    def test_an_indented_step_definition_is_read(self):
        """The step definition is not part of the registration shape, so it follows Go.

        Go compiles an indented `func stepOne`, `go test` runs the registration built on it, and
        the step is genuinely declared as a function. Anchored at column 1 the definition was
        invisible and the step was reported as one the proof directory does not declare as a
        function, which sends the reader to a proof that is already right and says nothing about
        the whitespace that is the real difference.
        """
        for indent in ('\t', '    ', '\r'):
            with self.subTest(indent=repr(indent)):
                proof_body = (
                    'func TestOne(t *testing.T) {\n'
                    '\tproofkit.Run(t, profileCases, stepOne)\n'
                    '}\n\n'
                    '%sfunc stepOne() {}\n' % indent)

                result, output = self.run_checker(proof_body=proof_body)

                self.assertEqual(result, 0, output)
                self.assertIn('compile check: OK', output)

    def test_a_step_definition_behind_whitespace_go_refuses_is_not_read(self):
        """The indentation set is Go's own, so a file Go will not compile defines nothing.

        U+00A0, U+000B and U+000C are not whitespace to Go's scanner: each makes the file illegal
        at that line, `go test` reports `illegal character U+00A0` and the package never builds.
        Reading them would credit a step definition in a proof that never compiles, which is the
        one direction this gate must not fail in, so Python's wider `\\s` is not used here.
        """
        for indent, character in (('\u00a0', 'U+00A0'), ('\x0b', 'U+000B'), ('\x0c', 'U+000C')):
            with self.subTest(character=character):
                proof_body = (
                    'func TestOne(t *testing.T) {\n'
                    '\tproofkit.Run(t, profileCases, stepOne)\n'
                    '}\n\n'
                    '%sfunc stepOne() {}\n' % indent)

                result, output = self.run_checker(proof_body=proof_body)

                self.assertEqual(result, 1, output)
                self.assertIn('S1 names proof function stepOne, which proof/gear/ does not '
                              'declare as a function', output)

    def test_whitespace_go_refuses_inside_a_registration_is_never_credited(self):
        """Every gap in the three lines is Go's separation set, not Python's `\\s`.

        A vertical tab at any of these five positions makes the file illegal where it sits, so
        `go test` reports `[setup failed]` and the registration never builds. The gate read all
        five as separation, printed `compile check: OK` at exit 0 and credited the step.

        What the complaint says depends on which line stopped matching, so the assertion is the
        one thing that must hold at every position: the run is not credited and the gate blocks.
        """
        for position, proof_body in REGISTRATION_WHITESPACE_CASES:
            with self.subTest(position=position):
                result, output = self.run_checker(proof_body=proof_body)

                self.assertEqual(result, 1, output)
                self.assertNotIn('compile check: OK', output)

    def test_a_step_name_go_cannot_have_is_never_credited(self):
        """The step list and the proof agreeing with each other is not the question.

        Both sides read the name with Python's `\\w`, so a U+00B2 suffix written into both matched
        both and the gate credited the step, while `go test` reported `illegal character U+00B2`.
        The name is a Go identifier wherever it is written, so a name Go cannot have is one no
        proof declares and no step claims.

        The complaint names the step rather than the name, because the claim is read whole or not
        at all: nothing here is a step name, so there is no name to quote. The earlier wording
        quoted a truncated identifier the drafter never wrote.
        """
        result, output = self.run_checker(proof_body=NON_GO_PROOF,
                                          step_body=self.claim_step('step' + NON_GO_STEP))

        self.assertEqual(result, 1, output)
        self.assertNotIn('compile check: OK', output)
        self.assertIn('S1 is tagged [GO] but names no proof function', output)

    def test_a_claim_trailing_a_rune_go_refuses_is_not_credited_to_its_prefix(self):
        """A claim is credited whole or not at all, and the right edge is what makes that true.

        With no right-edge assertion the pattern stopped at the first rune outside the Go
        identifier class and handed back the valid prefix, so a step claiming `stepOne²` was
        credited to a proof declaring plain `stepOne` and the gate printed OK at exit 0. The
        drafter wrote a name `go test` refuses, and the gate has to say so.
        """
        result, output = self.run_checker(step_body=self.claim_step('stepOne²'))

        self.assertEqual(result, 1, output)
        self.assertNotIn('compile check: OK', output)
        self.assertIn('S1 is tagged [GO] but names no proof function', output)

    def test_a_claim_behind_a_rune_go_refuses_is_not_credited_to_its_suffix(self):
        """The same hole on the left edge, which a Go-strict lookbehind does not close.

        `left` excludes only the runes a Go identifier carries, and U+00B2 is not one of them, so
        it succeeds in front of `²stepOne` and the valid suffix was credited to a proof declaring
        plain `stepOne`. The edge that closes this is the wider Python class: the neighbouring
        rune is what proves the token is not a Go name, so it has to suppress the claim.
        """
        result, output = self.run_checker(step_body=self.claim_step('²stepOne'))

        self.assertEqual(result, 1, output)
        self.assertNotIn('compile check: OK', output)
        self.assertIn('S1 is tagged [GO] but names no proof function', output)

    def test_a_claim_beside_ordinary_punctuation_is_still_read(self):
        """Suppressing a glued rune must not suppress the punctuation a step list writes.

        A claim is written inside backticks, inside bold, in a parenthesis and mid-sentence, and
        none of those neighbours is a word character. A name that carries a Unicode letter, a
        trailing letter or a trailing digit is one identifier and is read whole.
        """
        for claim, credited in (('`stepOne`', 'stepOne'),
                                ('stepOne, and', 'stepOne'),
                                ('stepOne. Next', 'stepOne'),
                                ('(stepOne)', 'stepOne'),
                                ('**stepOne**', 'stepOne'),
                                ('the `stepOne` step', 'stepOne'),
                                ('stepOnes', 'stepOnes'),
                                ('stepPrüfung', 'stepPrüfung'),
                                ('stepOne2', 'stepOne2')):
            with self.subTest(claim=claim):
                self.assertEqual(COMPILE_CHECKER.STEP_NAME_CLAIM.findall(claim), [credited])

    def test_a_unicode_letter_in_a_step_name_is_still_read(self):
        """Go identifiers include Unicode letters, so tightening must not refuse one.

        This proof compiles, registers and runs under `go test`, which
        `ProofDeclarationColumnTest` checks with the toolchain. A gate that read only ASCII names
        would report a step the proof does declare as one it does not.
        """
        result, output = self.run_checker(proof_body=GO_LETTER_PROOF,
                                          step_body=self.claim_step('step' + GO_LETTER_STEP))

        self.assertEqual(result, 0, output)
        self.assertIn('compile check: OK', output)

    def test_a_mention_is_not_silenced_by_a_neighbouring_character_go_refuses(self):
        """`PROOF_RUN_MENTION` is the only thing that sees a run on a method nothing declares.

        Its left edge was `\\b`, which is defined by Python's `\\w`: a preceding U+00B2 counts to
        Python as part of a word and to Go does not, so the mention went silent and the
        undeclared method was never named. That is this pattern failing in the opposite direction
        from the rest, and the edge is written as Go's own rather than dropped — a preceding Go
        identifier rune still suppresses it, because there the text is one longer name.
        """
        self.assertTrue(COMPILE_CHECKER.PROOF_RUN_MENTION.search('\u00b2proofkit3d.RunTypa('))
        self.assertIsNone(COMPILE_CHECKER.PROOF_RUN_MENTION.search('xproofkit3d.RunTypa('))

        proof_body = self.body('\tvar n = \u00b2proofkit3d.RunTypa(t, profileCases, stepOne)')

        result, output = self.run_checker(proof_body=proof_body)

        self.assertEqual(result, 1, output)
        self.assertIn('calls proofkit3d.RunTypa, which no harness package declares', output)

    def test_a_quoted_step_definition_is_still_not_a_definition(self):
        """Accepting leading whitespace must not turn a `func` in a comment or a string into one.

        `strip_go_comments_and_literals` blanks both to spaces before the scan, so a quoted
        declaration leaves a line of whitespace and nothing to match. Indented or not, it is text.
        """
        for name, quoted in (
                ('a block comment', '/*\n\tfunc stepTwo() {}\n*/\n'),
                ('a line comment', '\t// func stepTwo() {}\n'),
                ('a raw string', 'var sample = `\n\tfunc stepTwo() {}\n`\n')):
            with self.subTest(case=name):
                proof_body = (
                    'func TestOne(t *testing.T) {\n'
                    '\tproofkit.Run(t, profileCases, stepOne)\n'
                    '}\n\n'
                    '%s\n'
                    'func stepOne() {}\n' % quoted)

                result, output = self.run_checker(proof_body=proof_body)

                self.assertEqual(result, 0, output)
                self.assertIn('compile check: OK (1 steps, 1 proof functions', output)

    def test_an_indented_test_header_is_refused_and_named_as_the_header(self):
        """`TEST_HEADER` keeps its anchor, and `TEST_FUNCTION` makes the refusal land on it.

        The three-line shape is a contract, so an indented header is refused although `go test`
        runs it. What the refusal must not do is blame the run beneath, which is well formed; the
        wider diagnostic pattern reads the indented header and names the header instead.
        """
        for indent in ('\t', '    '):
            with self.subTest(indent=repr(indent)):
                proof_body = (
                    '%sfunc TestOne(t *testing.T) {\n'
                    '\tproofkit.Run(t, profileCases, stepOne)\n'
                    '}\n\n'
                    'func stepOne() {}\n' % indent)

                result, output = self.run_checker(proof_body=proof_body)

                self.assertEqual(result, 1, output)
                self.assertIn(
                    'proof/gear/proof_test.go:1 heads TestOne in a shape this gate does not read, '
                    'so the registration under it cannot be checked; a proof Test is headed '
                    '`func Test<Title>(t *testing.T) {`, with the testing package named in full',
                    output)
                self.assertNotIn('runs a proof outside the shape this gate reads', output)

    def test_an_indented_closing_brace_is_refused_as_a_run_off_the_shape(self):
        """`BLOCK_END` keeps its anchor too, and the refusal quotes the shape it belongs to.

        Go compiles an indented `}`, so this is the gate holding its own contract. The complaint
        is the off-shape one, which names all three lines of the registration, and the step is
        reported unregistered rather than quietly credited.
        """
        proof_body = (
            'func TestOne(t *testing.T) {\n'
            '\tproofkit.Run(t, profileCases, stepOne)\n'
            '\t}\n\n'
            'func stepOne() {}\n')

        result, output = self.run_checker(proof_body=proof_body)

        self.assertEqual(result, 1, output)
        self.assertIn('proof/gear/proof_test.go:2 runs a proof outside the shape this gate reads',
                      output)
        self.assertIn('S1 names proof function stepOne, which TestOne does not build with', output)

    def test_the_drafting_contract_states_the_column_the_gate_holds(self):
        """The checker and the contract the drafter is given have to say the same thing.

        `TEST_HEADER` and `BLOCK_END` refuse a line Go compiles, so the rule is this repository's
        and has to be written down where the drafter reads it. Without this, the two patterns could
        keep an anchor no document mentions, which is how the same defect arose on the step side.
        """
        contract = (Path(__file__).parents[3] / '.claude' / 'skills' / 'compile-gear'
                    / 'prompt.md').read_text(encoding='utf-8')

        self.assertIn('The header and the closing `}` each', contract)
        self.assertIn('start at column 1', contract)
        self.assertIn('The step function is not part of', contract)

    def test_the_drafting_contract_states_the_separation_and_names_go_reads(self):
        """The gate reads a proof under Go's rules, so the contract says which those are.

        A file carrying whitespace or a name Go's scanner refuses builds nothing, and what the
        gate says about it names the missing step or the unread run rather than the character.
        The contract is where the drafter finds out what that means.
        """
        contract = (Path(__file__).parents[3] / '.claude' / 'skills' / 'compile-gear'
                    / 'prompt.md').read_text(encoding='utf-8')

        self.assertIn('Between two tokens Go skips', contract)
        self.assertIn('a name is a Unicode letter or `_`', contract)

    def test_each_pattern_answers_the_column_question_as_its_site_says(self):
        """The four decisions, pinned on the patterns themselves.

        The messages above can be reworded; these cannot drift. Two patterns read a declaration
        wherever it starts, and two require column 1 because their line is the drafting contract.
        """
        widened = (
            ('STEP_DEFINITION', COMPILE_CHECKER.STEP_DEFINITION, 'func stepOne() {}'),
            ('TEST_FUNCTION', COMPILE_CHECKER.TEST_FUNCTION, 'func TestOne(t *testing.T) {'),
        )
        for name, pattern, line in widened:
            with self.subTest(pattern=name):
                self.assertTrue(pattern.match(line))
                for indent in (' ', '\t', '\r'):
                    self.assertTrue(pattern.match(indent + line), indent)
                for indent in ('\u00a0', '\x0b', '\x0c'):
                    self.assertIsNone(pattern.match(indent + line), indent)

        anchored = (
            ('TEST_HEADER', COMPILE_CHECKER.TEST_HEADER, 'func TestOne(t *testing.T) {'),
            ('BLOCK_END', COMPILE_CHECKER.BLOCK_END, '}'),
        )
        for name, pattern, line in anchored:
            with self.subTest(pattern=name):
                self.assertTrue(pattern.match(line))
                for indent in (' ', '\t', '\r'):
                    self.assertIsNone(pattern.match(indent + line), indent)

    # A Test header Go runs but this gate cannot read is named as a header problem.

    def test_test_header_go_runs_but_the_gate_cannot_read_names_the_header(self):
        """`Test_Foo`, `Test1x` and an aliased `testing` import all run under `go test`.

        Each was refused with "runs a proof outside the shape this gate reads", which points at a
        run call that is perfectly formed. The header is what did not match, so that is what the
        complaint names.
        """
        for header in ('func Test_Two(t *testing.T) {', 'func Test1x(t *testing.T) {',
                       'func TestTwo(t *tst.T) {', 'func Test(t *testing.T) {'):
            with self.subTest(header=header):
                proof_body = (
                    'func TestOne(t *testing.T) {\n'
                    '\tproofkit.Run(t, profileCases, stepOne)\n'
                    '}\n\n'
                    '%s\n'
                    '\tproofkit.Run(t, profileCases, stepTwo)\n'
                    '}\n\n'
                    'func stepOne() {}\n' % header)

                result, output = self.run_checker(proof_body=proof_body)

                self.assertEqual(result, 1, output)
                self.assertIn(
                    'proof/gear/proof_test.go:5 heads %s in a shape this gate does not read, so '
                    'the registration under it cannot be checked; a proof Test is headed '
                    '`func Test<Title>(t *testing.T) {`, with the testing package named in full'
                    % header.split()[1].split('(')[0], output)
                self.assertNotIn('runs a proof outside the shape this gate reads', output)

    def test_name_go_would_not_run_is_still_reported_as_an_off_shape_run(self):
        """`Testé` is not a test to Go, so its run is a stray call, not a mis-shaped header."""
        proof_body = (
            'func TestOne(t *testing.T) {\n'
            '\tproofkit.Run(t, profileCases, stepOne)\n'
            '}\n\n'
            'func Testé(t *testing.T) {\n'
            '\tproofkit.Run(t, profileCases, stepTwo)\n'
            '}\n\n'
            'func stepOne() {}\n')

        result, output = self.run_checker(proof_body=proof_body)

        self.assertEqual(result, 1, output)
        self.assertIn('proof/gear/proof_test.go:6 runs a proof outside the shape this gate reads',
                      output)
        self.assertNotIn('in a shape this gate does not read', output)


# The proof-side probe. The gate's answers about where a line may start are only worth anything
# next to what Go does with the same file, so each case below is a real proof file handed to the
# real toolchain.
PROOF_PROBE_HARNESS = ('package proofkit\n\n'
                       'import "testing"\n\n'
                       'func Run(t *testing.T, cases []int, build func()) {\n'
                       '\t_ = cases\n'
                       '\tbuild()\n'
                       '}\n')


def go_proof_verdict(body, test='TestOne'):
    """What the Go toolchain does with one proof file body.

    Returns `(compiled, ran)`: whether `go test` builds the package at all, and whether it ran the
    named test. The two are separate questions — a file can compile while its Test never runs,
    which is what a header Go does not recognise as a test looks like.

    The package is shaped like a real proof: an external `gear_test` package importing a `proofkit`
    stub, so `body` is the same text the gate is given.
    """
    root = Path(tempfile.mkdtemp(prefix='go-proof-'))
    try:
        (root / 'go.mod').write_text('module proofprobe\n\ngo 1.21\n')
        (root / 'proofkit').mkdir()
        (root / 'proofkit' / 'proofkit.go').write_text(PROOF_PROBE_HARNESS)
        (root / 'gear').mkdir()
        (root / 'gear' / 'gear.go').write_text('package gear\n')
        (root / 'gear' / 'proof_test.go').write_text(
            'package gear_test\n\n'
            'import (\n\t"testing"\n\n\t"proofprobe/proofkit"\n)\n\n'
            'var profileCases = []int{1}\n\n' + body)
        run = subprocess.run(['go', 'test', '-v', '-run', test, './gear'],
                             cwd=root, capture_output=True, text=True)
        return run.returncode == 0, '--- PASS: %s' % test in run.stdout
    finally:
        shutil.rmtree(root, ignore_errors=True)


class ProofDeclarationColumnTest(unittest.TestCase):
    """Go's half of the column question, for every line of a proof the gate reads by pattern.

    `gofmt` writes all three lines of a registration at column 1, and nothing in this repository
    runs `gofmt`: `proof/run.sh` runs only `go work init/edit` and `go test ./...`, and neither
    `.github/workflows/3d-proof.yml` nor `sketch-bench.yml` has a gofmt, vet or lint step. So an
    indented line is an ordinary proof file rather than a malformed one, and what the gate does
    about each is a decision recorded at its site rather than an accident of a pattern.

    The cases here say what Go does. `CheckCompileTest` says what the gate does about it.
    """

    # name, the proof body, whether Go compiles it, whether `go test` runs TestOne.
    CASES = (
        ('the shape at column 1',
         'func TestOne(t *testing.T) {\n'
         '\tproofkit.Run(t, profileCases, stepOne)\n'
         '}\n\n'
         'func stepOne() {}\n', True, True),
        ('a tab-indented step definition',
         'func TestOne(t *testing.T) {\n'
         '\tproofkit.Run(t, profileCases, stepOne)\n'
         '}\n\n'
         '\tfunc stepOne() {}\n', True, True),
        ('a space-indented step definition',
         'func TestOne(t *testing.T) {\n'
         '\tproofkit.Run(t, profileCases, stepOne)\n'
         '}\n\n'
         '    func stepOne() {}\n', True, True),
        ('a carriage return before the step definition',
         'func TestOne(t *testing.T) {\n'
         '\tproofkit.Run(t, profileCases, stepOne)\n'
         '}\n\n'
         '\rfunc stepOne() {}\n', True, True),
        ('an indented Test header',
         '\tfunc TestOne(t *testing.T) {\n'
         '\tproofkit.Run(t, profileCases, stepOne)\n'
         '}\n\n'
         'func stepOne() {}\n', True, True),
        ('an indented closing brace',
         'func TestOne(t *testing.T) {\n'
         '\tproofkit.Run(t, profileCases, stepOne)\n'
         '\t}\n\n'
         'func stepOne() {}\n', True, True),
        # The whitespace Go does not have. Each of these makes the file illegal where it sits, so
        # nothing in it is declared, registered or run.
        ('a non-breaking space before the step definition',
         'func TestOne(t *testing.T) {\n'
         '\tproofkit.Run(t, profileCases, stepOne)\n'
         '}\n\n'
         '\u00a0func stepOne() {}\n', False, False),
        ('a vertical tab before the step definition',
         'func TestOne(t *testing.T) {\n'
         '\tproofkit.Run(t, profileCases, stepOne)\n'
         '}\n\n'
         '\x0bfunc stepOne() {}\n', False, False),
        ('a form feed before the step definition',
         'func TestOne(t *testing.T) {\n'
         '\tproofkit.Run(t, profileCases, stepOne)\n'
         '}\n\n'
         '\x0cfunc stepOne() {}\n', False, False),
    )

    def test_go_agrees_with_every_recorded_proof_verdict(self):
        """Without this the table above is a claim about Go rather than a reading of it."""
        for name, body, compiled, ran in self.CASES:
            with self.subTest(case=name):
                self.assertEqual(go_proof_verdict(body), (compiled, ran))

    @unittest.skipUnless(shutil.which('go'), 'the Go toolchain establishes these verdicts')
    def test_go_refuses_a_vertical_tab_at_every_position_in_a_registration(self):
        """The Go half of `REGISTRATION_WHITESPACE_CASES`, which the gate half rests on.

        Between two tokens Go skips space, tab, carriage return and newline and nothing else, so
        each of these files is illegal where the vertical tab sits and neither compiles nor runs.
        """
        for position, body in REGISTRATION_WHITESPACE_CASES:
            with self.subTest(position=position):
                self.assertEqual(go_proof_verdict(body), (False, False))

    @unittest.skipUnless(shutil.which('go'), 'the Go toolchain establishes these verdicts')
    def test_go_compiles_and_runs_a_name_carrying_a_unicode_letter(self):
        """The accept direction, read from Go rather than assumed.

        Go's identifiers are a Unicode letter or `_` and then letters, decimal digits or `_`, so
        this proof is an ordinary one that builds and runs. A gate narrowed to ASCII would report
        a step the proof does declare as one it does not.
        """
        self.assertEqual(go_proof_verdict(GO_LETTER_PROOF, 'Test' + GO_LETTER_STEP), (True, True))

    @unittest.skipUnless(shutil.which('go'), 'the Go toolchain establishes these verdicts')
    def test_go_refuses_a_name_carrying_a_character_outside_its_identifier_rule(self):
        """U+00B2 is a word character to Python and not a letter or a digit to Go.

        Written into a step name it makes the proof illegal, which is what the step list agreeing
        with the proof about that name was hiding.
        """
        self.assertEqual(go_proof_verdict(NON_GO_PROOF, 'Test' + NON_GO_STEP), (False, False))


# Every `re` entry point whose first argument is a pattern. `re.escape` is not one of them: it
# takes text and hands back a pattern in which nothing is a class any more.
RE_PATTERN_CALLS = frozenset(
    ('compile', 'match', 'fullmatch', 'search', 'findall', 'finditer', 'sub', 'subn', 'split'))

PYTHON_CHARACTER_CLASS = re.compile(r'\\[bBdDsSwW]')

# The `GO_TOKENS` slots that put a Go identifier into a pattern. A pattern that fills one of these
# is one that reads a name, whatever else it reads.
GO_IDENTIFIER_SLOTS = ('%(part)s', '%(name)s', '%(qualified)s')


def re_pattern_calls(tree):
    """Every `re.*` call in `tree` that takes a pattern, as (method, pattern argument node).

    One enumeration, because two checks read it. `GoPatternClassTest` reads the literals that
    reach the pattern argument; `GoIdentifierEdgeTest` reads the patterns those calls compile and
    probes their edges. A pattern the enumeration misses is invisible to both, so there is one
    place to get it right.
    """
    for node in ast.walk(tree):
        if not isinstance(node, ast.Call):
            continue
        function = node.func
        if not (isinstance(function, ast.Attribute) and function.attr in RE_PATTERN_CALLS
                and isinstance(function.value, ast.Name) and function.value.id == 're'):
            continue
        pattern = node.args[0] if node.args else next(
            (keyword.value for keyword in node.keywords if keyword.arg == 'pattern'), None)
        if pattern is not None:
            yield function.attr, pattern


# One entry of the registry below: the `re` methods the module calls the pattern through, and the
# literal written at its own `re.compile` call, slots and all.
PatternUse = collections.namedtuple('PatternUse', 'methods written')


def go_identifier_patterns(source):
    """Every compiled pattern in `source` that reads a Go identifier, and how it is used.

    Returns {name: PatternUse}: the name the pattern is bound to, against the `re` methods the
    module calls it through and the literal it was compiled from. The name is the assignment
    target, or the function that returns the pattern when it is built rather than assigned, since
    a derived pattern has no other name to report. A pattern whose uses this scan cannot see comes
    back with an empty method set, which is not the same as one used only through `fullmatch`;
    unseen is a reason to probe it, not to trust it.

    A pattern reads a Go identifier when the literal written at its own `re.compile` call fills
    one of the `GO_TOKENS` identifier slots. Only that literal is read, unlike `pattern_literals`
    above: the slots are filled from `GO_TOKENS`, which carries a whole Go identifier in it, so
    following names here would make every pattern in the module look like a name reader.
    """
    tree = ast.parse(source)

    def written_pattern(node):
        """The literal a call compiles, when the call compiles one that reads a Go identifier."""
        for method, pattern in re_pattern_calls(node):
            if method != 'compile':
                continue
            written = ''.join(sub.value for sub in ast.walk(pattern)
                              if isinstance(sub, ast.Constant) and isinstance(sub.value, str))
            if any(slot in written for slot in GO_IDENTIFIER_SLOTS):
                return written
        return None

    found = {}
    for node in ast.walk(tree):
        if isinstance(node, ast.Assign) and len(node.targets) == 1 and isinstance(
                node.targets[0], ast.Name) and isinstance(node.value, ast.Call):
            written = written_pattern(node.value)
            if written is not None:
                found[node.targets[0].id] = [set(), written]
        elif isinstance(node, ast.FunctionDef):
            for sub in ast.walk(node):
                if isinstance(sub, ast.Return) and isinstance(sub.value, ast.Call):
                    written = written_pattern(sub.value)
                    if written is not None:
                        found[node.name] = [set(), written]

    for node in ast.walk(tree):
        if (isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute)
                and node.func.attr in RE_PATTERN_CALLS
                and isinstance(node.func.value, ast.Name)
                and node.func.value.id in found):
            found[node.func.value.id][0].add(node.func.attr)
    return {name: PatternUse(frozenset(methods), written)
            for name, (methods, written) in found.items()}


def pattern_literals(source):
    """Every (line, text) string literal that reaches the pattern argument of an `re.*` call.

    A pattern is rarely one literal written at its call site. It is built by `%` from constants
    defined elsewhere, so a name is followed to what it was assigned, wherever in the module that
    happened, and a name assigned in two places is followed to both. The looseness is the point:
    a check meant to fail closed must not leave a hiding place one indirection away.

    Only the pattern argument is read. The subject and the replacement are ordinary text, and a
    character class written in either is a run of characters rather than a rule.

    Adjacent literals are one constant by the time the parser is done, so a pattern written over
    two lines arrives here as a single string reported at the line it starts on.
    """
    tree = ast.parse(source)
    bindings = collections.defaultdict(list)
    for node in ast.walk(tree):
        if isinstance(node, ast.Assign):
            for target in node.targets:
                if isinstance(target, ast.Name):
                    bindings[target.id].append(node.value)

    found = set()

    def visit(node, seen):
        for sub in ast.walk(node):
            if isinstance(sub, ast.Constant) and isinstance(sub.value, str):
                found.add((sub.lineno, sub.value))
            elif isinstance(sub, ast.Name) and sub.id not in seen:
                for value in bindings.get(sub.id, ()):
                    visit(value, seen | {sub.id})

    for _, pattern in re_pattern_calls(tree):
        visit(pattern, frozenset())
    return found


# Invented modules, not code from this repository. An illustration of a defect that is a live
# string in the tree becomes a false target for whoever greps for it next, so these are written
# to be wrong on purpose and exist nowhere else.
LOOSE_PATTERN_MODULE = (
    'import re\n'
    '\n'
    'HEADER = re.compile(r"^\\s*func\\s+(\\w+)")\n')

HIDDEN_LOOSE_PATTERN_MODULE = (
    'import re\n'
    '\n'
    'GAP = r"\\s*"\n'
    'HEADER = re.compile(r"^%sfunc" % GAP)\n')

STRICT_PATTERN_MODULE = (
    'import re\n'
    '\n'
    'GAP = "[ \\t\\r]*"\n'
    'HEADER = re.compile(r"^%sfunc%s[A-Z]" % (GAP, GAP))\n')

CLASS_IN_THE_SUBJECT_MODULE = (
    'import re\n'
    '\n'
    'NOTE = r"the \\s class is discussed here"\n'
    'FOUND = re.findall(r"[ \\t]+", NOTE)\n')


class GoPatternClassTest(unittest.TestCase):
    """No pattern that reads Go source may spell a Python character class.

    Go's scanner skips space, tab, carriage return and newline between tokens and nothing else,
    and its identifiers are a Unicode letter or `_` then letters, decimal digits or `_`. Python's
    `\\s` and `\\w` are both wider, so every pattern written with them read a file Go refuses as a
    sound one and credited what it found. Fixing the patterns is the smaller half of that; this is
    the half that keeps them fixed.

    It fails closed. A literal reaching an `re.*` pattern argument is held to Go's rules unless it
    is named below, so a new pattern carrying a class fails here until someone classifies it, and
    classifying it means writing down why the text it matches is not Go.
    """

    # Patterns whose subject is Markdown. Go never reads a step list, a spec or a provenance
    # table, so Python's classes are the right ones there and the wide reading is the intended
    # one: prose is what these match.
    MARKDOWN_PATTERNS = {
        r'[\w./-]+\.(?:md|go|py|json|sh)':
            'PATH_REF, a file path as a step list writes it',
        r'(?<![\w./-])[\w./-]+\.md\b':
            'DOCUMENT_REF, a Markdown document a spec names in prose',
        r'`(%s):(\d+)(?:\s*[-\u2013]\s*(\d+))?`':
            'INLINE_CITATION, the older `path:line` citation form in a From block',
        r'\bL(\d+)(?:\s*[-\u2013]\s*(\d+))?\b':
            'LINE_RANGE, an `L12-L20` range in a From block',
        r'^##\s+(\S+)\s+`\[(GO|PROSE)\]`\s+(.*)$':
            'steps_of, a step heading in the step list',
        r'^\*\*From:\*\*(.*?)(?=\n\s*\n|\Z)':
            'from_block, the From block under a step heading',
        r'<!--\s*check-compile:\s*ignore\s+([^>]*?)-->':
            'named_calls and named_call_shapes, an HTML comment waiving a named call',
        r'(?<![\w./-])(?:proof|\.tmp)/[\w./-]+':
            'proof_paths, a proof path named in the step-list summary',
        r'\|\s*`([\w./-]+)`\s*\|\s*`([0-9a-f]{40})`\s*\|':
            'stamped, a row of the provenance table',
        r'^##\s':
            'SECTION_END, the next `## ` heading after a step list\'s provenance section',
    }

    # The two classes that say what a Go identifier rune is. They are written as `\w` with what
    # Go refuses taken out, so the Python classes in them are the subtraction itself rather than a
    # rule about Go being approximated. `GoIdentifierClassTest` holds both against Go's own tables.
    GO_CLASS_DEFINITIONS = {
        r'[^\W\d%s]': 'GO_IDENTIFIER_START, a word character that is neither a digit nor a '
                      'numeral Go refuses',
        r'[^\W%s]': 'GO_IDENTIFIER_PART, a word character that is not a numeral Go refuses',
    }

    # The two edges of a token that has to stand alone. They are the one place on this path where
    # the wider Python class is the correct rule rather than an approximation of Go's, and the
    # reason is what the match is for. Every other pattern here matches to raise a complaint,
    # where a wider read is a louder report; `STEP_NAME_CLAIM` matches to grant a step credit for
    # a proof function, where a wider read is a false OK. A rune Go refuses glued to the token
    # proves the token as written is not a Go name at all, so it must suppress the whole match
    # rather than leave the valid prefix or suffix credited — and a Go-strict edge cannot do that,
    # because such a rune is not a Go identifier rune and every Go-strict edge succeeds beside it.
    # `GoIdentifierEdgeTest` holds the behaviour these two exist for.
    GO_CLAIM_EDGES = {
        r'(?<!\w)': 'no_word_left, the left edge of a claim that must stand alone',
        r'(?!\w)': 'no_word_right, the right edge of a claim that must stand alone',
    }

    def allowed(self):
        allowed = dict(self.MARKDOWN_PATTERNS)
        allowed.update(self.GO_CLASS_DEFINITIONS)
        allowed.update(self.GO_CLAIM_EDGES)
        return allowed

    def findings(self, source):
        """Every pattern literal in `source` that carries a Python class and is not allowlisted."""
        allowed = self.allowed()
        return sorted(
            (line, ''.join(sorted(set(PYTHON_CHARACTER_CLASS.findall(text)))), text)
            for line, text in pattern_literals(source)
            if PYTHON_CHARACTER_CLASS.search(text) and text not in allowed)

    def checker_source(self):
        # provenance.py is a sibling module check_compile.py imports its Markdown-matching
        # patterns from (DOCUMENT_REF, STAMPED_ROW); this scan follows them there so an
        # allowlist entry for either still points at a pattern that is actually written down,
        # just no longer inside check_compile.py itself.
        return COMPILE_CHECKER_PATH.read_text() + '\n' + PROVENANCE_MODULE_PATH.read_text()

    def test_no_pattern_that_reads_go_source_spells_a_python_class(self):
        """The gate itself, under its own rule."""
        self.assertEqual(self.findings(self.checker_source()), [])

    def test_every_allowlisted_pattern_is_still_written_in_the_checker(self):
        """An allowlist entry for a pattern nobody kept is a waiver over nothing.

        Left standing it would silently cover a future pattern that happened to be spelled the
        same way, so an entry that no longer matches anything is a failure here.
        """
        carrying = {text for _, text in pattern_literals(self.checker_source())
                    if PYTHON_CHARACTER_CLASS.search(text)}
        for pattern, reason in sorted(self.allowed().items()):
            with self.subTest(reason=reason):
                self.assertIn(pattern, carrying)

    def test_a_python_class_in_a_new_pattern_is_a_finding(self):
        """The property the whole check exists for, on an invented pattern.

        Nothing was added to the allowlist for it, and that is what makes it fail: the default
        classification is Go-strict.
        """
        findings = self.findings(LOOSE_PATTERN_MODULE)

        self.assertEqual(len(findings), 1, findings)
        self.assertEqual(findings[0][1], r'\s\w')

    def test_a_python_class_reached_through_a_constant_is_a_finding(self):
        """A pattern is usually assembled from constants, so the check follows them.

        Reading only the literal at the call site would leave every class one `%` away from
        invisible, which is exactly how these patterns are written.
        """
        findings = self.findings(HIDDEN_LOOSE_PATTERN_MODULE)

        self.assertEqual(len(findings), 1, findings)
        self.assertEqual(findings[0][1], r'\s')

    def test_a_go_strict_pattern_is_not_a_finding(self):
        """The control. A check that failed on everything would prove nothing about the eleven."""
        self.assertEqual(self.findings(STRICT_PATTERN_MODULE), [])

    def test_a_class_written_in_the_subject_is_not_a_finding(self):
        """Only the pattern argument is a rule. A class in the searched text is characters."""
        self.assertEqual(self.findings(CLASS_IN_THE_SUBJECT_MODULE), [])


# An invented pattern, not one from this repository, and nothing in the tree is spelled this way.
# It reads a name with neither edge asserted, which is what the probe below has to be able to see.
OPEN_EDGED_PATTERN = re.compile(r'(gate[A-Z]\w*)')

# One canonical sample per pattern that reads a Go identifier: the line it is read from, with a
# `%s` where the name sits, and the name. The probe glues a rune Go refuses to one side of the
# name and then the other, so the sample has to put the name where the pattern expects it and
# nothing else about the line may be off the shape.
GO_IDENTIFIER_SAMPLES = {
    'STEP_DEFINITION': ('func %s() {}', 'stepOne'),
    'TEST_HEADER': ('func %s(t *testing.T) {', 'TestOne'),
    'TEST_FUNCTION': ('\tfunc %s(t *testing.T) {', 'TestOne'),
    'GO_RUN_DECLARATION': ('func %s(t *testing.T, cases []Case, build func()) {', 'RunSolid'),
    'STEP_NAME_CLAIM': ('The step builds the profile with `%s`.', 'stepOne'),
    'PROOF_RUN_MENTION': ('\tproofkit3d.%s(t, profileCases, stepOne)', 'RunSolid'),
    'proof_run_shape': ('\tproofkit.Run(t, profileCases, %s)', 'stepOne'),
}

# The patterns that need no edge at all, with the reason each one does not. `fullmatch` makes the
# whole subject the name, so there is no neighbour to glue anything to. The claim is checked
# against the scan rather than taken on trust: an entry here whose pattern is used through
# anything else is a pattern nobody is probing.
GO_IDENTIFIER_WHOLE_SUBJECT = {
    'STEP_NAME': 'the build argument a registration passed, tested whole against the step '
                 'naming rule',
}


class GoIdentifierEdgeTest(unittest.TestCase):
    """A pattern that reads a Go identifier must refuse a name with a refused rune glued to it.

    `GoPatternClassTest` above reads the character classes a pattern spells, and the defect this
    class exists for spells none: `STEP_NAME_CLAIM` had no right-edge assertion at all, so the
    match stopped at the first rune outside the identifier class and handed back the valid prefix.
    A missing assertion is nothing for a scan of written classes to see, so this is the
    behavioural half of the same rule.

    Every pattern in the registry gets its canonical sample with one refused rune glued to the
    left of the name, and again to the right, and must match neither. Which patterns those are is
    enumerated from the checker rather than listed by hand, so a new pattern that reads a name
    fails here until someone gives it a sample or writes down why the whole subject is the name.

    Suppressing rather than truncating is what a credit-granting match needs, and truncation is
    what the other patterns are already safe from by accident: each is closed by a required
    character such as `(` or `.` that a refused rune displaces. Their edges are checked here too,
    because that safety is a property of how they happen to be written and nothing was holding it.
    """

    # The rune the probe glues on. `GoIdentifierClassTest` holds the checker's classes against
    # Go's own tables and `go_proof_verdict` shows the toolchain refusing a proof carrying this
    # one, so what makes it the right probe is checked in three places rather than asserted here.
    GLUED = '²'

    def registry(self):
        return go_identifier_patterns(COMPILE_CHECKER_PATH.read_text())

    def pattern_for(self, name):
        """The compiled pattern a registry entry names.

        Most are module constants. The registration shape is not: it is derived from the harness
        sources, so it is reached through the accessor that builds it.
        """
        found = getattr(COMPILE_CHECKER, name, None)
        if isinstance(found, re.Pattern):
            return found
        if name == 'proof_run_shape':
            return COMPILE_CHECKER.proof_run_shapes().shape
        self.fail('%s is in the registry but this test cannot reach the pattern it compiles'
                  % name)

    def edges(self, pattern, template, name):
        """Whether the pattern still matches with the refused rune glued to each side of `name`."""
        return tuple(bool(pattern.search(template % glued))
                     for glued in (self.GLUED + name, name + self.GLUED))

    def test_every_pattern_that_reads_a_go_identifier_is_probed_or_classified(self):
        """The registry is the checker's, so a new name reader cannot arrive unexamined."""
        registry = self.registry()

        self.assertEqual(sorted(registry),
                         sorted(set(GO_IDENTIFIER_SAMPLES) | set(GO_IDENTIFIER_WHOLE_SUBJECT)))
        for name, reason in sorted(GO_IDENTIFIER_WHOLE_SUBJECT.items()):
            with self.subTest(name=name, reason=reason):
                self.assertEqual(registry[name].methods, frozenset(('fullmatch',)))

    def test_only_the_claim_pattern_is_written_from_the_credit_side_edges(self):
        """`GO_TOKENS` says the wider pair belongs to the one match that grants credit.

        That sentence is a rule about which pattern may take which edge, and a rule nothing checks
        is one the next pattern can quietly break, so the count is held here.
        """
        written = sorted(name for name, use in self.registry().items()
                         if '%(no_word_left)s' in use.written
                         or '%(no_word_right)s' in use.written)

        self.assertEqual(written, ['STEP_NAME_CLAIM'])

    def test_a_refused_rune_glued_to_a_name_suppresses_the_match(self):
        """The property itself, on both edges of every pattern that reads a name.

        The sample matching is asserted first, because a probe whose sample never matched would
        report both edges closed on a pattern it had not exercised at all.
        """
        for name, (template, identifier) in sorted(GO_IDENTIFIER_SAMPLES.items()):
            with self.subTest(pattern=name):
                pattern = self.pattern_for(name)

                match = pattern.search(template % identifier)

                self.assertIsNotNone(match, template % identifier)
                self.assertIn(identifier, match.groups())
                self.assertEqual(self.edges(pattern, template, identifier), (False, False))

    def test_the_probe_sees_a_pattern_with_open_edges(self):
        """The control. A probe that reported every pattern closed would prove nothing.

        The invented pattern above asserts neither edge, which is the shape the defect had, and
        the probe has to report both sides open.
        """
        self.assertEqual(self.edges(OPEN_EDGED_PATTERN, 'builds `%s` here', 'gateOne'),
                         (True, True))

    def test_the_glued_rune_is_a_word_character_go_refuses(self):
        """The probe rests on Python and Go disagreeing about this rune, so state both halves."""
        self.assertTrue(re.fullmatch(r'\w', self.GLUED))
        self.assertIsNone(
            re.compile(COMPILE_CHECKER.GO_IDENTIFIER_PART).fullmatch(self.GLUED))


GO_UNICODE_TABLE_PROBE = (
    'package main\n'
    '\n'
    'import (\n'
    '\t"fmt"\n'
    '\t"unicode"\n'
    ')\n'
    '\n'
    '// The two tables Go builds its identifier rule from, as ranges, under the Unicode\n'
    '// release the toolchain was built against.\n'
    'func main() {\n'
    '\tfmt.Printf("version %s\\n", unicode.Version)\n'
    '\temit("letter", unicode.IsLetter)\n'
    '\temit("digit", unicode.IsDigit)\n'
    '}\n'
    '\n'
    'func emit(label string, member func(rune) bool) {\n'
    '\tstart := rune(-1)\n'
    '\tfor r := rune(0); r <= 0x110000; r++ {\n'
    '\t\tif r < 0x110000 && member(r) {\n'
    '\t\t\tif start < 0 {\n'
    '\t\t\t\tstart = r\n'
    '\t\t\t}\n'
    '\t\t\tcontinue\n'
    '\t\t}\n'
    '\t\tif start >= 0 {\n'
    '\t\t\tfmt.Printf("%s %d %d\\n", label, start, r-1)\n'
    '\t\t\tstart = -1\n'
    '\t\t}\n'
    '\t}\n'
    '}\n')


def go_unicode_tables():
    """Go's Unicode release, and `unicode.IsLetter` and `unicode.IsDigit` as code point sets."""
    root = Path(tempfile.mkdtemp(prefix='go-unicode-'))
    try:
        (root / 'go.mod').write_text('module unicodeprobe\n\ngo 1.21\n')
        (root / 'main.go').write_text(GO_UNICODE_TABLE_PROBE)
        run = subprocess.run(['go', 'run', '.'], cwd=root, capture_output=True, text=True)
        if run.returncode != 0:
            raise AssertionError(run.stderr)
        tables = {'letter': set(), 'digit': set()}
        version = None
        for line in run.stdout.split('\n'):
            if not line:
                continue
            fields = line.split()
            if fields[0] == 'version':
                version = fields[1]
                continue
            label, first, last = fields
            tables[label].update(range(int(first), int(last) + 1))
        return version, tables['letter'], tables['digit']
    finally:
        shutil.rmtree(root, ignore_errors=True)


class GoIdentifierClassTest(unittest.TestCase):
    """`GO_IDENTIFIER_START` and `GO_IDENTIFIER_PART` are Go's rule and not an approximation of it.

    The classes are derived from Python's Unicode tables rather than typed out, so what has to be
    checked is the derivation: that Python's letters and decimal digits are the same sets Go
    builds its identifier rule from, and that the two classes read exactly those. A Python or a Go
    release that moved either boundary fails here rather than in a proof nobody can explain.
    """

    def characters_matching(self, pattern):
        """Every code point one class matches, in a single pass over the whole range."""
        return {ord(character) for character in re.findall(pattern, ALL_CODE_POINTS)}

    def test_the_classes_read_every_rune_gos_rule_allows_and_no_other(self):
        letters = {code for code in range(sys.maxunicode + 1) if chr(code).isalpha()}
        digits = {code for code in range(sys.maxunicode + 1) if chr(code).isdecimal()}

        self.assertEqual(self.characters_matching(COMPILE_CHECKER.GO_IDENTIFIER_START),
                         letters | {ord('_')})
        self.assertEqual(self.characters_matching(COMPILE_CHECKER.GO_IDENTIFIER_PART),
                         letters | digits | {ord('_')})

    def test_the_characters_the_defect_turned_on_are_named_one_by_one(self):
        """The set equality above is the rule; these are the runes the eleven sites read.

        U+00B2, U+2160 and U+00BD are word characters to Python and neither letters nor decimal
        digits to Go, which reports `invalid character U+00B2 in identifier`. The letters beside
        them are the accept direction: Go identifiers are Unicode, and narrowing to ASCII would
        refuse a proof that builds.
        """
        start = re.compile(COMPILE_CHECKER.GO_IDENTIFIER_START)
        part = re.compile(COMPILE_CHECKER.GO_IDENTIFIER_PART)
        for character in ('a', 'Z', '_', 'ü', 'Ω', '漢'):
            with self.subTest(character=character):
                self.assertTrue(start.fullmatch(character))
                self.assertTrue(part.fullmatch(character))
        for character in ('\u00b2', '\u2160', '\u00bd'):
            with self.subTest(character=repr(character)):
                self.assertIsNone(start.fullmatch(character))
                self.assertIsNone(part.fullmatch(character))
        for character in ('7', '\u0663'):
            with self.subTest(character=repr(character)):
                self.assertIsNone(start.fullmatch(character))
                self.assertTrue(part.fullmatch(character))

    @unittest.skipUnless(shutil.which('go'), 'the Go toolchain establishes these tables')
    def test_go_agrees_about_which_runes_are_letters_and_digits(self):
        """The derivation rests on Python's tables being Go's, so Go is asked rather than trusted.

        Go's spec writes an identifier as a Unicode letter or `_` then letters, Unicode decimal
        digits or `_`, and the compiler reads those two tables through `unicode.IsLetter` and
        `unicode.IsDigit`.

        The two sides are separate Unicode releases that happen to agree, so the failure message
        names both: a Python and a Go built against different releases genuinely disagree about a
        handful of runes, and which of the two moved is the first thing to know.
        """
        go_version, go_letters, go_digits = go_unicode_tables()
        releases = ('Go reads Unicode %s and this Python reads Unicode %s'
                    % (go_version, unicodedata.unidata_version))

        self.assertEqual({code for code in range(sys.maxunicode + 1) if chr(code).isalpha()},
                         go_letters, releases)
        self.assertEqual({code for code in range(sys.maxunicode + 1) if chr(code).isdecimal()},
                         go_digits, releases)


class GoFilenameRuleTest(unittest.TestCase):
    """`go_ignores_file` against a fixed linux/amd64, so every case is the one Go was asked.

    Running the suite elsewhere would change which suffixes are satisfied, so the platform is
    passed in rather than taken from the machine. The integration tests above use the live one.
    """

    IGNORED = (
        ('_x_test.go', 'invisible'),
        ('.x_test.go', 'invisible'),
        ('x_windows_test.go', 'suffix'),
        ('x_arm64_test.go', 'suffix'),
        ('x_windows_amd64_test.go', 'suffix'),
    )

    COMPILED = (
        'proof_helper_test.go',
        'proof_solid_test.go',
        'a_unix_test.go',
        'a_Windows_test.go',
        'windows_test.go',
        'proof_linux_test.go',
        'proof_amd64_test.go',
        'proof_test.go',
        'proof.go',
    )

    def test_names_go_never_compiles(self):
        for name, reason in self.IGNORED:
            with self.subTest(name=name):
                self.assertIn(
                    reason,
                    COMPILE_CHECKER.go_ignores_file(name, goos='linux', goarch='amd64') or '')

    def test_names_go_does_compile(self):
        for name in self.COMPILED:
            with self.subTest(name=name):
                self.assertIsNone(
                    COMPILE_CHECKER.go_ignores_file(name, goos='linux', goarch='amd64'))

    def test_go_aliases_a_few_platforms(self):
        """`android` takes `linux` files, `illumos` takes `solaris`, and `ios` takes `darwin`."""
        for goos, tag in (('android', 'linux'), ('illumos', 'solaris'), ('ios', 'darwin')):
            with self.subTest(goos=goos):
                self.assertIsNone(COMPILE_CHECKER.go_ignores_file(
                    'proof_%s_test.go' % tag, goos=goos, goarch='amd64'))

    def test_this_machine_is_named_the_way_go_names_it(self):
        """The live platform must be a name Go knows, or every suffix check is nonsense."""
        self.assertIn(COMPILE_CHECKER.GOOS, COMPILE_CHECKER.GO_KNOWN_OS)
        self.assertIn(COMPILE_CHECKER.GOARCH, COMPILE_CHECKER.GO_KNOWN_ARCH)


class ProofRunArityDerivationTest(unittest.TestCase):
    """The run table is read from the harness sources rather than typed out beside them."""

    def test_derived_table_matches_the_harness_signatures(self):
        """The tripwire for the derivation, and the one place a run signature is pinned.

        Deriving the counts means a hand-copy cannot drift, but a derivation that quietly read
        nothing would look just as healthy. This pins what `proof/proofkit` and `proof/proofkit3d`
        declare today, so adding a run method or changing an arity fails here and gets reviewed.
        """
        self.assertEqual(COMPILE_CHECKER.proof_run_shapes().arguments, {
            'proofkit.Run': 3,
            'proofkit3d.Run': 4,
            'proofkit3d.RunSolid': 4,
            'proofkit3d.RunWithGate': 5,
        })

    def test_parameter_count_is_of_parameters_not_commas_in_the_source(self):
        """Go lets parameters share a type, and a nested comma belongs to the type it sits in."""
        cases = (
            ('', 0),
            ('t *testing.T', 1),
            ('t *testing.T, cases []Case, build Build', 3),
            ('t *testing.T, a, b Case', 3),
            ('t *testing.T, build func(*testing.T, []Case) error', 2),
            ('t *testing.T, table map[string]Case, seed [2]float64', 3),
        )
        for parameters, expected in cases:
            with self.subTest(parameters=parameters):
                self.assertEqual(COMPILE_CHECKER.go_parameter_count(parameters), expected)

    def test_arities_are_read_from_code_only_never_a_comment_or_a_literal(self):
        """The declaration is read wherever it starts on its line, but only in code.

        The derivation allows leading whitespace, because Go compiles and exports an indented
        package-level declaration and nothing here runs gofmt. That width must not reach the two
        places a `func` is text rather than a declaration, and both are written here: the doc
        comment holds a whole indented signature, and the raw string holds one at column 1.
        `strip_go_comments_and_literals` blanks each to spaces before the scan, so neither is
        derived, and `RunTwice` and `RunQuoted` appearing in the table would say that broke.
        """
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'harness.go'
            path.write_text(
                'package harness\n\n'
                '// RunTwice would be written\n'
                '//\n'
                '//\tfunc RunTwice(t *testing.T, a, b, c, d Case) {}\n'
                '//\n'
                '// but it is not.\n'
                'const usage = `\n'
                'func RunQuoted(t *testing.T, a, b, c, d, e Case) {}\n'
                '`\n\n'
                'func Run(t *testing.T, cases []Case, build Build) {}\n\n'
                '\tfunc RunSolid(t *testing.T, cases []Case, build Build, assert Assert) {}\n\n'
                'func RunWithGate(t *testing.T, cases []Case, build Build, gate Gate,\n'
                '\tassert Assert) {\n'
                '}\n\n'
                'func helper(t *testing.T) {}\n')

            table = COMPILE_CHECKER.go_run_arities('harness', str(path))

        self.assertEqual(table,
                         {'harness.Run': 3, 'harness.RunSolid': 4, 'harness.RunWithGate': 5})

    def test_both_run_patterns_are_built_from_the_derived_table(self):
        """Every derived method is read as a registration; no other pairing is read as one.

        The mention pattern stays wider on purpose, so a call on a method no package declares —
        `proofkit.RunSolid` — is still seen and complained about rather than passing unnoticed.
        """
        runs = COMPILE_CHECKER.proof_run_shapes()
        packages = sorted({method.split('.')[0] for method in runs.arguments})
        names = sorted({method.split('.')[1] for method in runs.arguments})
        for package in packages:
            for name in names:
                method = '%s.%s' % (package, name)
                declared = COMPILE_CHECKER.proof_run_shapes().arguments.get(method)
                arguments = ['t', 'profileCases', 'stepOne'] + ['assertOne'] * 2
                line = '\t%s(%s)' % (method, ', '.join(arguments[:declared or 3]))
                with self.subTest(method=method):
                    self.assertTrue(COMPILE_CHECKER.PROOF_RUN_MENTION.search(line))
                    match = runs.shape.match(line)
                    if declared is None:
                        self.assertIsNone(match)
                        continue
                    self.assertIsNotNone(match)
                    self.assertEqual(COMPILE_CHECKER.proof_run_arguments(match),
                                     (method, declared))


def harness_file(package, name='RunExtra', indent=b''):
    """One harness source declaring a single run method, for the named package.

    `indent` is the whitespace in front of the declaration. Go's layout is free and nothing in
    this repository holds the harness to `gofmt`, so an indented declaration is an ordinary file
    the derivation has to read, not a malformed one.
    """
    return b'package %s\n\n%sfunc %s(a, b, c int) {}\n' % (package.encode(), indent,
                                                          name.encode())


def go_harness_verdict(name, source):
    """What Go does with one file dropped into a package, and whether its method can be called.

    Returns `(placement, callable)`. `placement` is where the file lands — `compiled` for
    `GoFiles`, `ignored` for `IgnoredGoFiles`, `unreadable` when Go refuses its bytes — and
    `callable` is whether another package can build against the method it declares, which is the
    question the derived run table is answering.

    The call lives in a second package on purpose. A method Go does not compile is not a
    compilation error in the harness; it is an `undefined` at every use, which is what a proof
    registration on it would be.

    The probe package is named `proofkit` so the fixture bytes are the same ones the gate is given.
    """
    root = Path(tempfile.mkdtemp(prefix='go-harness-'))
    try:
        (root / 'go.mod').write_text('module harnessprobe\n\ngo 1.21\n')
        (root / 'proofkit').mkdir()
        (root / 'proofkit' / 'proofkit.go').write_bytes(harness_file('proofkit', 'Run'))
        (root / 'proofkit' / name).write_bytes(source)
        (root / 'user').mkdir()
        (root / 'user' / 'user.go').write_text(
            'package user\n\n'
            'import "harnessprobe/proofkit"\n\n'
            'func call() { proofkit.RunExtra(1, 2, 3) }\n')
        built = subprocess.run(['go', 'build', './proofkit'],
                               cwd=root, capture_output=True, text=True)
        listed = subprocess.run(['go', 'list', '-e', '-json', './proofkit'],
                                cwd=root, capture_output=True, text=True)
        info = json.JSONDecoder().raw_decode(listed.stdout.strip())[0]
        if any(message in built.stderr for message in GO_UNREADABLE_MESSAGES):
            placement = 'unreadable'
        elif name in (info.get('IgnoredGoFiles') or []):
            placement = 'ignored'
        else:
            placement = 'compiled'
        used = subprocess.run(['go', 'build', './user'], cwd=root, capture_output=True, text=True)
        return placement, used.returncode == 0
    finally:
        shutil.rmtree(root, ignore_errors=True)


class HarnessSourceRuleTest(unittest.TestCase):
    """The harness sources are read under the Go rules the proof files are already held to.

    Every run method the gate accepts is derived from `proof/proofkit/` and `proof/proofkit3d/`,
    so the derivation is where a Go rule missing from the read does its damage: a file read here
    but not compiled by Go invents a method that does not exist, and every registration on it is
    then credited although `go vet` calls it `undefined`.

    Which rule is a skip and which is a named error differs, and `HARNESS_FILE_CASES` records what
    Go does with each input alongside what the gate does about it.
    """

    # name, file bytes below the package clause, what Go does, what the gate does.
    #
    #   compiled/ignored/unreadable  where Go puts the file, and `callable` below says whether the
    #                                method it declares can be used from another package at all
    #   read                         the gate derives the method from it
    #   skipped                      the gate passes over the file exactly as Go does
    #   error                        the gate raises a named error rather than deriving anything
    @staticmethod
    def cases():
        goos = COMPILE_CHECKER.GOOS
        body = harness_file('proofkit')
        return (
            ('an ordinary name', 'runmore.go', body, 'compiled', 'read'),
            ('a satisfied GOOS suffix', 'run_%s.go' % goos, body, 'compiled', 'read'),
            ('an unsatisfied GOOS suffix', 'run_%s.go' % foreign_goos(), body, 'ignored',
             'skipped'),
            ('an unsatisfiable constraint', 'runconstrained.go',
             b'//go:build never\n\n' + body, 'ignored', 'error'),
            # The row the loud option exists for. Go compiles this file here, so a gate that
            # skipped it would drop a method that does exist and report every sound registration
            # on it as a call the harness does not declare.
            ('a satisfied constraint', 'runconstrained.go',
             ('//go:build %s\n\n' % goos).encode() + body, 'compiled', 'error'),
            ('a legacy constraint', 'runconstrained.go',
             b'// +build never\n\n' + body, 'ignored', 'error'),
            ('a NUL byte', 'runnul.go', body[:-1] + b'\x00\n', 'unreadable', 'error'),
            # Go's layout is free, and `proof/run.sh` and both workflows run no gofmt, vet or
            # lint, so nothing stops an indented declaration reaching the harness. Anchoring the
            # derivation at column 1 lost the method while Go still compiled and exported it, and
            # every registration on it was then reported as a call no harness package declares.
            ('a tab-indented declaration', 'runindenttab.go',
             harness_file('proofkit', indent=b'\t'), 'compiled', 'read'),
            ('a space-indented declaration', 'runindentspaces.go',
             harness_file('proofkit', indent=b'    '), 'compiled', 'read'),
        )

    def derive(self, name=None, source=None):
        """The run table derived from a harness tree holding one extra file under `proofkit`."""
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            for package in COMPILE_CHECKER.PROOF_RUN_PACKAGES:
                (root / 'proof' / package).mkdir(parents=True)
                (root / 'proof' / package / ('%s.go' % package)).write_bytes(
                    harness_file(package, 'Run'))
            if name is not None:
                (root / 'proof' / 'proofkit' / name).write_bytes(source)
            return COMPILE_CHECKER.derive_proof_run_arguments(root=str(root))

    def test_the_fixture_tree_derives_what_it_declares(self):
        """The control. Without it a derivation that read nothing would pass every case below."""
        self.assertEqual(self.derive(), {'proofkit.Run': 3, 'proofkit3d.Run': 3})

    def test_a_harness_file_go_compiles_is_read(self):
        for name, filename, source, _, gate in self.cases():
            if gate != 'read':
                continue
            with self.subTest(case=name):
                self.assertEqual(self.derive(filename, source).get('proofkit.RunExtra'), 3)

    def test_a_harness_file_go_excludes_by_name_is_not_read(self):
        """Go decides this from the name alone, and the gate already holds proof files to it.

        A `proof/proofkit/run_windows.go` declaring `RunWindows` was read on linux and its method
        accepted in a registration, while `go list` reported the file in `IgnoredGoFiles` and
        `go vet` reported `undefined: proofkit.RunWindows`.

        Skipping is the right answer here rather than an error: the name is the whole reason, Go
        does the same, and the method really does not exist on this machine, so a proof that calls
        it is told so at the call.
        """
        for name, filename, source, _, gate in self.cases():
            if gate != 'skipped':
                continue
            with self.subTest(case=name):
                table = self.derive(filename, source)

                self.assertNotIn('proofkit.RunExtra', table)
                self.assertIn('proofkit.Run', table)

    def test_a_harness_file_carrying_a_build_constraint_is_a_named_error(self):
        """A constraint is settled outside the file, so the harness carries none in any spelling.

        `//go:build never` excludes the file and `//go:build linux` on a linux builder does not,
        and nothing in the file says which of the two the reader is holding. Skipping both would
        drop a method Go does build and turn every sound registration on it into a complaint about
        the proof, which is the wrong file entirely. So both are refused, loudly, naming the
        harness file and line.
        """
        for name, filename, source, _, gate in self.cases():
            if gate != 'error':
                continue
            if b'//go:build' not in source and b'+build' not in source:
                continue
            with self.subTest(case=name):
                with self.assertRaises(RuntimeError) as raised:
                    self.derive(filename, source)

                message = str(raised.exception)
                self.assertIn('%s:1 carries a build constraint' % filename, message)
                self.assertIn('a harness source must carry no build constraint', message)

    def test_a_harness_file_go_will_not_read_is_a_named_error(self):
        """The byte refusals reach the harness too, and name the harness file when they bite.

        A `proof/proofkit/runnul.go` holding a NUL byte was read with a plain `read()` and its
        method accepted, while `go vet ./proofkit/` refused the file outright. Reading it through
        `go_source`, as the proof files are read, turns it into an error that names the file, the
        byte and what Go says about it.
        """
        for name, filename, source, _, gate in self.cases():
            if gate != 'error' or b'\x00' not in source:
                continue
            with self.subTest(case=name):
                with self.assertRaises(RuntimeError) as raised:
                    self.derive(filename, source)

                message = str(raised.exception)
                self.assertIn(filename, message)
                self.assertIn('is one Go will not read', message)
                self.assertIn('holds a NUL byte', message)

    def test_no_harness_file_is_credited_with_a_method_go_does_not_have(self):
        """The whole class, in one line: read or refused, never quietly wrong.

        A file Go does not compile must never contribute a method, whether the gate skips it or
        errors on it.
        """
        for name, filename, source, placement, _ in self.cases():
            if placement == 'compiled':
                continue
            with self.subTest(case=name):
                try:
                    table = self.derive(filename, source)
                except RuntimeError:
                    continue
                self.assertNotIn('proofkit.RunExtra', table)

    def test_a_run_declaration_behind_whitespace_go_refuses_is_not_derived(self):
        """The gap between `func` and the name is Go's separation set, like the indent before it.

        One vertical tab there made `go vet` report `invalid character U+000B` while the gate
        still derived `RunExtra`, and every registration built on the method was then credited
        against a harness Go cannot compile.
        """
        for character, name in (('\x0b', 'U+000B'), ('\x0c', 'U+000C'), ('\u00a0', 'U+00A0')):
            with self.subTest(character=name):
                source = (b'package proofkit\n\nfunc%sRunExtra(a, b, c int) {}\n'
                          % character.encode())

                self.assertNotIn('proofkit.RunExtra', self.derive('rungap.go', source))

    @unittest.skipUnless(shutil.which('go'), 'the Go toolchain establishes these verdicts')
    def test_go_refuses_a_harness_declaration_behind_whitespace_it_does_not_have(self):
        """The Go half of the case above: the file is unreadable and the method uncallable."""
        for character, name in (('\x0b', 'U+000B'), ('\x0c', 'U+000C'), ('\u00a0', 'U+00A0')):
            with self.subTest(character=name):
                source = (b'package proofkit\n\nfunc%sRunExtra(a, b, c int) {}\n'
                          % character.encode())

                self.assertEqual(go_harness_verdict('rungap.go', source), ('unreadable', False))

    @unittest.skipUnless(shutil.which('go'), 'the Go toolchain establishes these verdicts')
    def test_go_agrees_with_the_recorded_harness_verdicts(self):
        """The `go` column of every row is what the toolchain actually does with that input.

        A row written from memory, or a Go release that moved the boundary, fails here rather
        than in a harness read nobody can explain. `callable` is checked with it, because that is
        the consequence the rows are really about: a method from a file Go does not compile is an
        `undefined` at every use.
        """
        for name, filename, source, placement, _ in self.cases():
            with self.subTest(case=name):
                self.assertEqual(go_harness_verdict(filename, source),
                                 (placement, placement == 'compiled'))

    @unittest.skipUnless(shutil.which('go'), 'the Go toolchain establishes this API')
    def test_no_exported_harness_helper_but_a_run_method_is_named_run(self):
        """`PROOF_RUN_MENTION` reads any `Run`-prefixed name, and this is what makes that safe.

        The boundary is a fact about the harness, not a rule Go enforces: were a helper called
        `RunReport` added beside `RequireSound`, every call to it in a step body would become a
        complaint about a method the harness does declare. `go doc` is asked rather than the
        source, so the answer is the package's real exported API.
        """
        root = Path(__file__).parents[3] / 'proof'
        for package in COMPILE_CHECKER.PROOF_RUN_PACKAGES:
            with self.subTest(package=package):
                listed = subprocess.run(['go', 'doc', './%s' % package],
                                        cwd=root, capture_output=True, text=True)
                self.assertEqual(listed.returncode, 0, listed.stderr)
                exported = set(re.findall(r'^(?:func|type|var|const)\s+(\w+)',
                                          listed.stdout, re.M))

                self.assertEqual(
                    {name for name in exported if name.startswith('Run')},
                    {method.split('.', 1)[1]
                     for method in COMPILE_CHECKER.proof_run_shapes().arguments
                     if method.startswith('%s.' % package)})


class BrokenHarnessExitCodeTest(unittest.TestCase):
    """A harness this gate cannot read is a broken input: exit 2, named, and never a traceback.

    The run table used to be derived at import, before `main` existed, so every raise from the
    derivation killed the command with an uncaught traceback and exit 1. Exit 1 is this checker's
    code for findings in the artifact under review, and the harness is not that artifact: sending
    a reader to look for findings in a proof that has none, with a Python stack instead of a
    message, is the wrong file and the wrong shape of answer. Exit 2 is the documented code for a
    missing or broken input, and `main` already uses it for a usage error, a missing step list, a
    step list with no steps and an unavailable API database.

    Each case is run as a real subprocess, because the defect was in what the command does at
    import and only a fresh process shows that.
    """

    SKILL_DIR = Path(__file__).parent

    STEP_LIST = '## S1 `[PROSE]` Title\n\n**From:** `spec/gear/instructions.md` L1\n'

    def harness_tree(self):
        """A tree the checker reads as a repository, with a sound harness and one step list.

        The skill directory is linked rather than copied, so the checker under test is this one
        and not a stale copy. `check_compile.py` takes the repository root from its own path
        without resolving links, so the link is what puts the harness below in its reach.
        """
        root = Path(tempfile.mkdtemp(prefix='harness-exit-'))
        self.addCleanup(shutil.rmtree, str(root), True)
        (root / '.claude' / 'skills').mkdir(parents=True)
        (root / '.claude' / 'skills' / 'generate-gear').symlink_to(self.SKILL_DIR)
        for package in COMPILE_CHECKER.PROOF_RUN_PACKAGES:
            (root / 'proof' / package).mkdir(parents=True)
            (root / 'proof' / package / ('%s.go' % package)).write_bytes(
                harness_file(package, 'Run'))
        (root / 'spec' / 'gear').mkdir(parents=True)
        (root / 'spec' / 'gear' / 'steps.md').write_text(self.STEP_LIST)
        return root

    def check(self, root, *arguments):
        return subprocess.run(
            [sys.executable,
             str(root / '.claude' / 'skills' / 'generate-gear' / 'check_compile.py')]
            + list(arguments),
            cwd=str(root), capture_output=True, text=True)

    def assertNoTraceback(self, result):
        self.assertNotIn('Traceback', result.stderr)
        self.assertNotIn('Traceback', result.stdout)

    def test_a_sound_harness_is_read_and_the_run_goes_on_to_the_artifact(self):
        """The control. Without it every case below would pass on a checker that always exits 2.

        The exit code is not pinned, because what the run finds past the harness depends on
        whether the Fusion API database is installed. What is pinned is that the harness was read:
        the run got somewhere the harness never named, and it did not stop at the derivation.
        """
        result = self.check(self.harness_tree(), 'gear')

        self.assertNoTraceback(result)
        self.assertNotEqual(result.returncode, 0)
        self.assertNotIn('harness', result.stderr)
        self.assertNotIn('run methods', result.stderr)

    def test_a_harness_carrying_a_build_constraint_exits_two(self):
        """Whether Go compiles a constrained file is settled outside it, so this is an input fault.

        `//go:build linux` is the case that makes the point: on a linux builder Go does compile
        the file, so skipping it would drop a method that exists and turn every sound registration
        on it into a complaint about the proof.
        """
        root = self.harness_tree()
        (root / 'proof' / 'proofkit' / 'runconstrained.go').write_bytes(
            b'//go:build linux\n\n' + harness_file('proofkit'))

        result = self.check(root, 'gear')

        self.assertNoTraceback(result)
        self.assertEqual(result.returncode, 2, result.stderr)
        self.assertIn('runconstrained.go:1 carries a build constraint', result.stderr)

    def test_a_harness_go_will_not_read_exits_two(self):
        root = self.harness_tree()
        (root / 'proof' / 'proofkit' / 'runnul.go').write_bytes(
            harness_file('proofkit')[:-1] + b'\x00\n')

        result = self.check(root, 'gear')

        self.assertNoTraceback(result)
        self.assertEqual(result.returncode, 2, result.stderr)
        self.assertIn('is one Go will not read', result.stderr)
        self.assertIn('holds a NUL byte', result.stderr)

    @unittest.skipIf(getattr(os, 'geteuid', lambda: 1)() == 0,
                     'root reads a mode 000 file, so there is nothing to refuse')
    def test_a_harness_this_process_cannot_open_exits_two(self):
        """The `OSError` path, and the reason catching `RuntimeError` alone leaves a hole.

        Every other refusal here is the gate's own `RuntimeError`. A harness source at mode 000
        raises `PermissionError` from `open()` instead, which is an `OSError`, and the operating
        system's message says nothing about why this checker was reading the file, so `main` names
        the harness itself.
        """
        root = self.harness_tree()
        unreadable = root / 'proof' / 'proofkit' / 'proofkit.go'
        unreadable.chmod(0o000)
        self.addCleanup(unreadable.chmod, 0o644)

        result = self.check(root, 'gear')

        self.assertNoTraceback(result)
        self.assertEqual(result.returncode, 2, result.stderr)
        self.assertIn('the harness sources under proof/proofkit, proof/proofkit3d cannot be read',
                      result.stderr)
        self.assertIn('Permission denied', result.stderr)

    def test_a_missing_harness_exits_two(self):
        root = self.harness_tree()
        for package in COMPILE_CHECKER.PROOF_RUN_PACKAGES:
            shutil.rmtree(str(root / 'proof' / package))

        result = self.check(root, 'gear')

        self.assertNoTraceback(result)
        self.assertEqual(result.returncode, 2, result.stderr)
        self.assertIn('no proof run methods found under proof/proofkit, proof/proofkit3d',
                      result.stderr)

    def test_the_usage_message_survives_a_broken_harness(self):
        """Reading the harness at import ran it before the arguments were looked at.

        With a harness the gate refuses, `check_compile.py` with no arguments printed a Python
        stack instead of the one line that says how to call it.
        """
        root = self.harness_tree()
        (root / 'proof' / 'proofkit' / 'runconstrained.go').write_bytes(
            b'//go:build linux\n\n' + harness_file('proofkit'))

        result = self.check(root)

        self.assertNoTraceback(result)
        self.assertEqual(result.returncode, 2, result.stderr)
        self.assertIn('usage: check_compile.py <gear>', result.stderr)
        self.assertNotIn('build constraint', result.stderr)


class CommittedStepListProofPathsTest(unittest.TestCase):
    """The proof-path gate must have something to check on the real step lists.

    `proof_paths` reads only the text above `## Provenance`, so a recompile that writes the
    summary sentence below that heading, or that names bare file names instead of
    `proof/<gear>/...`, leaves the gate scanning an empty slice and passing vacuously. The
    synthetic-string tests elsewhere cannot see that, because they supply the sentence the
    committed file is supposed to carry.
    """

    ROOT = Path(__file__).parents[3]

    def step_lists(self):
        paths = sorted((self.ROOT / 'spec').glob('*/steps.md'))
        self.assertTrue(paths, 'no compiled step list found under spec/')
        return paths

    def test_every_committed_step_list_names_its_proof_files(self):
        for path in self.step_lists():
            with self.subTest(steps=str(path.relative_to(self.ROOT))):
                named = COMPILE_CHECKER.proof_paths(path.read_text(encoding='utf-8'))

                self.assertTrue(
                    named,
                    '%s names no proof path above ## Provenance, so the gate checks nothing'
                    % path.relative_to(self.ROOT))
                gear = path.parent.name
                for proof_path in named:
                    self.assertTrue(
                        proof_path.startswith('proof/%s/' % gear),
                        '%s points at %s, not the committed proof/%s/'
                        % (path.relative_to(self.ROOT), proof_path, gear))

    def test_every_named_proof_path_exists_and_is_committed(self):
        prior = os.getcwd()
        try:
            os.chdir(self.ROOT)
            for path in self.step_lists():
                for proof_path in COMPILE_CHECKER.proof_paths(path.read_text(encoding='utf-8')):
                    with self.subTest(proof=proof_path):
                        self.assertTrue(
                            os.path.isfile(proof_path),
                            '%s does not exist' % proof_path)
                        self.assertTrue(
                            COMPILE_CHECKER.proof_path_is_tracked_or_committed(proof_path),
                            '%s is not tracked or committed' % proof_path)
        finally:
            os.chdir(prior)


if __name__ == '__main__':
    unittest.main()
