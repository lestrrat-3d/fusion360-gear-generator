#!/usr/bin/env python3
"""Regression tests for the compile-stage gate."""
import contextlib
import importlib.util
import inspect
import io
import os
import tempfile
import unittest
from pathlib import Path
from unittest import mock


COMPILE_CHECKER_PATH = Path(__file__).with_name('check_compile.py')
COMPILE_MODULE_SPEC = importlib.util.spec_from_file_location('check_compile', COMPILE_CHECKER_PATH)
COMPILE_CHECKER = importlib.util.module_from_spec(COMPILE_MODULE_SPEC)
COMPILE_MODULE_SPEC.loader.exec_module(COMPILE_CHECKER)


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
        """
        proof_body = self.registration('TestOne', 'proofkit.RunSolid', 'stepOne',
                                       extra=', assertOne')

        result, output = self.run_checker(proof_body=proof_body)

        self.assertEqual(result, 1)
        self.assertIn('proof/gear/proof_test.go:2 runs a proof outside the shape this gate reads',
                      output)
        self.assertIn(
            'S1 names proof function stepOne, which TestOne does not build with', output)

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

    def foreign_goos(self):
        """A GOOS this machine is not, so a `_GOOS` suffix here is genuinely unsatisfied."""
        for name in sorted(COMPILE_CHECKER.GO_KNOWN_OS):
            if not COMPILE_CHECKER.go_platform_tag_matches(
                    name, COMPILE_CHECKER.GOOS, COMPILE_CHECKER.GOARCH):
                return name
        raise AssertionError('every known GOOS matches this machine')

    def test_file_go_never_compiles_is_named_not_credited(self):
        """A proof file Go drops still parsed as a proof, so its steps were credited unbuilt.

        `_x_test.go` and `.x_test.go` are invisible to `go/build`, and a `_GOOS` or `_GOARCH`
        suffix this machine does not satisfy lands the file in `IgnoredGoFiles`. In all three
        cases `go test` reports no test files, so nothing here registers anything.
        """
        foreign = self.foreign_goos()
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

    # The two `//` constraint forms differ in what may sit before the directive.

    def constrained(self, header):
        return (
            '%s\n\n'
            'package gear_test\n\n'
            'func TestOne(t *testing.T) {\n'
            '\tproofkit.Run(t, profileCases, stepOne)\n'
            '}\n\n'
            'func stepOne() {}\n' % header)

    def test_every_spelling_go_reads_as_a_constraint_is_refused(self):
        """`//go:build` takes no space after the slashes; the legacy form takes any."""
        for header in ('//go:build ignore', '  //go:build ignore', '\t//go:build ignore',
                       '// +build ignore', '//+build ignore', '//  +build ignore',
                       '//\t+build ignore', '  // +build ignore'):
            with self.subTest(header=header):
                result, output = self.run_checker(proof_body=self.constrained(header))

                self.assertEqual(result, 1, output)
                self.assertIn('carries a build constraint', output)

    def test_comment_go_does_not_read_as_a_constraint_is_accepted(self):
        """`// go:build …` is prose to Go: the file lands in `GoFiles` and its tests run.

        Allowing whitespace between the slashes and `go:build` refused a header comment that
        merely discusses the directive, in a file Go builds.
        """
        for header in ('// go:build ignore', '// go:build is discussed here, not used',
                       '//  go:build ignore', '// go: build ignore'):
            with self.subTest(header=header):
                result, output = self.run_checker(proof_body=self.constrained(header))

                self.assertEqual(result, 0, output)
                self.assertIn('compile check: OK', output)

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
        `.claude/skills/compile-gear/SKILL.md`. Only the message changed, to name the shape the
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
        self.assertEqual(COMPILE_CHECKER.PROOF_RUN_ARGUMENTS, {
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

    def test_arities_are_read_from_source_text_comments_excluded(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'harness.go'
            path.write_text(
                'package harness\n\n'
                '// RunTwice would be written\n'
                '//\n'
                '//\tfunc RunTwice(t *testing.T, a, b, c, d Case) {}\n'
                '//\n'
                '// but it is not.\n'
                'func Run(t *testing.T, cases []Case, build Build) {}\n\n'
                'func RunWithGate(t *testing.T, cases []Case, build Build, gate Gate,\n'
                '\tassert Assert) {\n'
                '}\n\n'
                'func helper(t *testing.T) {}\n')

            table = COMPILE_CHECKER.go_run_arities('harness', str(path))

        self.assertEqual(table, {'harness.Run': 3, 'harness.RunWithGate': 5})

    def test_both_run_patterns_are_built_from_the_derived_table(self):
        """Every derived method is read as a registration; no other pairing is read as one.

        The mention pattern stays wider on purpose, so a call on a method no package declares —
        `proofkit.RunSolid` — is still seen and complained about rather than passing unnoticed.
        """
        packages = sorted({method.split('.')[0] for method in COMPILE_CHECKER.PROOF_RUN_ARGUMENTS})
        names = sorted({method.split('.')[1] for method in COMPILE_CHECKER.PROOF_RUN_ARGUMENTS})
        for package in packages:
            for name in names:
                method = '%s.%s' % (package, name)
                declared = COMPILE_CHECKER.PROOF_RUN_ARGUMENTS.get(method)
                arguments = ['t', 'profileCases', 'stepOne'] + ['assertOne'] * 2
                line = '\t%s(%s)' % (method, ', '.join(arguments[:declared or 3]))
                with self.subTest(method=method):
                    self.assertTrue(COMPILE_CHECKER.PROOF_RUN_MENTION.search(line))
                    match = COMPILE_CHECKER.PROOF_RUN.match(line)
                    if declared is None:
                        self.assertIsNone(match)
                        continue
                    self.assertIsNotNone(match)
                    self.assertEqual(COMPILE_CHECKER.proof_run_arguments(match),
                                     (method, declared))


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
