"""Regression tests for proof_shards.py, the owner of proof/shards.json.

The manifest's whole job is to fail when the tree moves under it, so most of what follows feeds
`check` an inventory that differs from the manifest in one specific way and asserts the complaint
comes back. The inventories are written here rather than taken from the toolchain: a fixture can
hold a fuzz target, a renamed test and a name full of regexp punctuation, and the real tree holds
none of those.
"""
import importlib.util
import json
import re
import unittest
from pathlib import Path

HERE = Path(__file__).parent
ROOT = HERE.parents[2]


def load(name):
    spec = importlib.util.spec_from_file_location(name, HERE / (name + '.py'))
    assert spec is not None and spec.loader is not None, name
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


shards = load('proof_shards')


def manifest(*groups):
    return {'groups': [dict(group) for group in groups]}


def group(name, packages, **rest):
    body = {'name': name, 'packages': packages}
    body.update(rest)
    return body


def inventory(packages):
    return {'packages': packages}


class PatternTest(unittest.TestCase):
    def test_the_pattern_is_anchored_and_sorted(self):
        self.assertEqual('^(TestA|TestB)$', shards.pattern_for(['TestB', 'TestA']))

    def test_an_anchored_pattern_does_not_select_a_longer_name(self):
        pattern = shards.pattern_for(['TestBore'])
        self.assertEqual(['TestBore'], shards.selects(pattern, ['TestBore', 'TestBoreCut']))

    def test_an_empty_group_selects_nothing_rather_than_everything(self):
        # `-run ''` matches every test, so the empty case must not produce an empty pattern.
        pattern = shards.pattern_for([])
        self.assertEqual('^$', pattern)
        self.assertEqual([], shards.selects(pattern, ['TestA', 'TestB']))

    def test_no_pattern_constrains_a_subtest(self):
        # A `/` in a -run expression applies the next part to the next level of the test tree,
        # which would drop the subtests a case table creates at run time.
        self.assertNotIn('/', shards.pattern_for(['TestA', 'TestB']))

    def test_regexp_punctuation_in_a_name_is_escaped(self):
        pattern = shards.pattern_for(['Test.A'])
        self.assertEqual(r'^(Test\.A)$', pattern)
        self.assertEqual(['Test.A'], shards.selects(pattern, ['Test.A', 'TestXA']))

    def test_a_letter_outside_ascii_stays_a_literal(self):
        # RE2 takes a literal letter of any script; backslash-escaping one is what it refuses.
        self.assertEqual('^(Testé)$', shards.pattern_for(['Testé']))

    def test_a_name_no_go_pattern_can_express_is_refused(self):
        # No `func Test…` declares a name holding a space, and emitting one would produce a
        # pattern go test rejects, which reports a regexp error instead of the real complaint.
        with self.assertRaises(ValueError):
            shards.pattern_for(['Test One'])


class CoverageTest(unittest.TestCase):
    """Every discovered name belongs to exactly one group, and every assigned name exists."""

    WHOLE = manifest(
        group('alpha', {'./alpha': ['TestOne', 'TestTwo']}),
        group('beta', {'./beta': ['TestThree']}),
    )
    TREE = inventory({'./alpha': ['TestOne', 'TestTwo'], './beta': ['TestThree']})

    def test_a_manifest_that_matches_the_tree_has_no_findings(self):
        self.assertEqual([], shards.check(self.WHOLE, self.TREE))

    def test_an_added_test_is_reported(self):
        tree = inventory({'./alpha': ['TestOne', 'TestTwo', 'TestNew'], './beta': ['TestThree']})
        findings = shards.check(self.WHOLE, tree)
        self.assertEqual(1, len(findings), findings)
        self.assertIn('TestNew', findings[0])
        self.assertIn('no job would run it', findings[0])

    def test_an_omitted_test_is_reported(self):
        broken = manifest(
            group('alpha', {'./alpha': ['TestOne']}),
            group('beta', {'./beta': ['TestThree']}),
        )
        findings = shards.check(broken, self.TREE)
        self.assertEqual(1, len(findings), findings)
        self.assertIn('TestTwo', findings[0])

    def test_a_test_in_two_groups_is_reported(self):
        broken = manifest(
            group('alpha', {'./alpha': ['TestOne', 'TestTwo']}),
            group('beta', {'./alpha': ['TestTwo'], './beta': ['TestThree']}),
        )
        findings = shards.check(broken, self.TREE)
        self.assertTrue(any('assigned to both alpha and beta' in f for f in findings), findings)

    def test_a_renamed_test_is_reported_from_both_sides(self):
        tree = inventory({'./alpha': ['TestOne', 'TestRenamed'], './beta': ['TestThree']})
        findings = shards.check(self.WHOLE, tree)
        self.assertTrue(any('TestTwo is not in the package any more' in f for f in findings),
                        findings)
        self.assertTrue(any('TestRenamed' in f and 'no job would run it' in f for f in findings),
                        findings)

    def test_a_package_in_no_group_is_reported(self):
        tree = inventory({'./alpha': ['TestOne', 'TestTwo'], './beta': ['TestThree'],
                          './gamma': []})
        findings = shards.check(self.WHOLE, tree)
        self.assertEqual(1, len(findings), findings)
        self.assertIn('./gamma is in no group', findings[0])

    def test_a_package_with_no_tests_still_has_to_be_assigned(self):
        # It has to compile somewhere; no other job in the workflow builds the proof module.
        tree = inventory({'./alpha': ['TestOne', 'TestTwo'], './beta': ['TestThree'],
                          './involute': []})
        whole = manifest(
            group('alpha', {'./alpha': ['TestOne', 'TestTwo'], './involute': []}),
            group('beta', {'./beta': ['TestThree']}),
        )
        self.assertEqual([], shards.check(whole, tree))

    def test_a_package_the_toolchain_does_not_report_is_reported(self):
        broken = manifest(
            group('alpha', {'./alpha': ['TestOne', 'TestTwo'], './ghost': []}),
            group('beta', {'./beta': ['TestThree']}),
        )
        findings = shards.check(broken, self.TREE)
        self.assertTrue(any('./ghost' in f and 'go list does not report' in f
                            for f in findings), findings)

    def test_a_group_that_selects_nothing_is_reported(self):
        broken = manifest(
            group('alpha', {'./alpha': ['TestOne', 'TestTwo']}),
            group('beta', {'./beta': ['TestThree']}),
            group('empty', {}),
        )
        findings = shards.check(broken, self.TREE)
        self.assertTrue(any('group empty selects no test' in f for f in findings), findings)

    def test_a_group_may_declare_that_it_runs_no_test(self):
        allowed = manifest(
            group('alpha', {'./alpha': ['TestOne', 'TestTwo']}),
            group('beta', {'./beta': ['TestThree']}),
            group('compile-only', {'./gamma': []}, allow_no_tests=True),
        )
        tree = inventory({'./alpha': ['TestOne', 'TestTwo'], './beta': ['TestThree'],
                          './gamma': []})
        self.assertEqual([], shards.check(allowed, tree))

    def test_a_subtest_name_is_refused(self):
        broken = manifest(
            group('alpha', {'./alpha': ['TestOne', 'TestTwo', 'TestOne/case']}),
            group('beta', {'./beta': ['TestThree']}),
        )
        findings = shards.check(broken, self.TREE)
        self.assertTrue(any('names a subtest' in f for f in findings), findings)

    def test_a_group_name_that_would_not_survive_a_matrix_is_refused(self):
        broken = manifest(
            group('Alpha Group', {'./alpha': ['TestOne', 'TestTwo']}),
            group('beta', {'./beta': ['TestThree']}),
        )
        findings = shards.check(broken, self.TREE)
        self.assertTrue(any('not a lowercase hyphenated word' in f for f in findings), findings)

    def test_a_group_declared_twice_is_refused(self):
        broken = manifest(
            group('alpha', {'./alpha': ['TestOne', 'TestTwo']}),
            group('alpha', {'./beta': ['TestThree']}),
        )
        findings = shards.check(broken, self.TREE)
        self.assertTrue(any('declared twice' in f for f in findings), findings)

    def test_a_name_the_checker_cannot_prove_a_pattern_for_is_reported(self):
        tree = inventory({'./alpha': ['TestOne', 'TestTwo', 'Testé'],
                          './beta': ['TestThree']})
        findings = shards.check(self.WHOLE, tree)
        self.assertTrue(any('cannot prove a -run pattern' in f for f in findings), findings)

    def test_an_empty_manifest_is_an_input_error_not_a_finding(self):
        with self.assertRaises(ValueError):
            shards.check({'groups': []}, self.TREE)


class OverMatchTest(unittest.TestCase):
    """One `-run` covers every package in the invocation, so a shared name can leak between them.

    `TestBoreCut` is declared in both `bevelgear` and `spurgear` in this repository, and
    `TestRenderExample` in both `bevelgear` and `cycloidal`, so this is the live hazard rather
    than a theoretical one.
    """

    TREE = inventory({'./alpha': ['TestShared', 'TestOne'], './beta': ['TestShared']})

    def test_a_group_holding_two_packages_that_share_a_name_is_reported(self):
        broken = manifest(
            group('one', {'./alpha': ['TestShared', 'TestOne'], './beta': []},
                  allow_no_tests=False),
            group('two', {'./beta': ['TestShared']}),
        )
        findings = shards.check(broken, self.TREE)
        self.assertTrue(any('group one' in f and 'selects' in f and './beta' in f
                            for f in findings), findings)

    def test_the_same_name_in_two_packages_is_fine_when_the_groups_are_disjoint(self):
        whole = manifest(
            group('one', {'./alpha': ['TestShared', 'TestOne']}),
            group('two', {'./beta': ['TestShared']}),
        )
        self.assertEqual([], shards.check(whole, self.TREE))


class InventoryParsingTest(unittest.TestCase):
    """`go test -list -json` output, including the shapes today's tree does not contain."""

    MODULE = 'example.com/proof'

    def events(self, *rows):
        return '\n'.join(json.dumps(row) for row in rows)

    def test_examples_and_fuzz_targets_are_inventoried_and_benchmarks_are_not(self):
        text = self.events(
            {'Action': 'start', 'Package': self.MODULE + '/alpha'},
            {'Action': 'output', 'Package': self.MODULE + '/alpha', 'Output': 'TestOne\n'},
            {'Action': 'output', 'Package': self.MODULE + '/alpha', 'Output': 'ExampleGear\n'},
            {'Action': 'output', 'Package': self.MODULE + '/alpha', 'Output': 'FuzzProfile\n'},
            {'Action': 'output', 'Package': self.MODULE + '/alpha', 'Output': 'BenchmarkLoft\n'},
        )
        found = shards.names_from_events(text, self.MODULE)
        self.assertEqual({'./alpha': ['ExampleGear', 'FuzzProfile', 'TestOne']}, found)

    def test_a_package_with_no_test_files_still_appears(self):
        text = self.events(
            {'Action': 'start', 'Package': self.MODULE + '/involute'},
            {'Action': 'output', 'Package': self.MODULE + '/involute',
             'Output': '?   \t%s/involute\t[no test files]\n' % self.MODULE},
            {'Action': 'skip', 'Package': self.MODULE + '/involute', 'Elapsed': 0},
        )
        self.assertEqual({'./involute': []}, shards.names_from_events(text, self.MODULE))

    def test_the_runners_own_output_is_skipped_by_shape(self):
        text = '\n'.join([
            'verified sketch revision: 34765bc10360',
            'running: go test -list .* -json ./...',
            json.dumps({'Action': 'output', 'Package': self.MODULE + '/alpha',
                        'Output': 'TestOne\n'}),
            'ok  \t%s/alpha\t0.004s' % self.MODULE,
        ])
        self.assertEqual({'./alpha': ['TestOne']}, shards.names_from_events(text, self.MODULE))

    def test_a_package_outside_the_module_is_refused(self):
        text = json.dumps({'Action': 'start', 'Package': 'example.org/other'})
        with self.assertRaises(ValueError):
            shards.names_from_events(text, self.MODULE)


class SelectionTest(unittest.TestCase):
    WHOLE = manifest(
        group('alpha', {'./alpha': ['TestTwo', 'TestOne'], './involute': []}),
        group('beta', {'./beta': ['TestThree']}),
    )

    def test_a_group_selects_its_packages_and_its_names(self):
        self.assertEqual(
            ['--package', './alpha', '--package', './involute',
             '--', '-run', '^(TestOne|TestTwo)$'],
            shards.selection(self.WHOLE, 'alpha'))

    def test_an_unknown_group_is_refused(self):
        with self.assertRaises(ValueError):
            shards.selection(self.WHOLE, 'gamma')


class RealManifestTest(unittest.TestCase):
    """The checked-in manifest, held to what can be checked without a Go toolchain."""

    def setUp(self):
        self.path = ROOT / 'proof' / 'shards.json'
        self.manifest = json.loads(self.path.read_text())
        self.groups = shards.groups_of(self.manifest, str(self.path))

    def test_every_named_package_is_a_directory_in_the_proof_module(self):
        for entry in self.groups:
            for package in entry['packages']:
                self.assertTrue((ROOT / 'proof' / package[len('./'):]).is_dir(),
                                '%s names %s, which is not a directory'
                                % (entry['name'], package))

    def test_every_group_emits_an_anchored_pattern_with_no_subtest_part(self):
        for entry in self.groups:
            names = sum(entry['packages'].values(), [])
            pattern = shards.pattern_for(names)
            self.assertTrue(pattern.startswith('^(') or pattern == '^$', pattern)
            self.assertNotIn('/', pattern)

    def test_no_test_is_assigned_twice(self):
        seen = set()
        for entry in self.groups:
            for package, names in entry['packages'].items():
                for name in names:
                    self.assertNotIn((package, name), seen)
                    seen.add((package, name))

    def test_every_declared_test_exists_in_the_package_source(self):
        """A cheap stand-in for the inventory job, so a rename fails the checker suite too.

        Reading `func TestX(` out of the package's `_test.go` files is not what CI trusts —
        `proof_shards.py check` asks the toolchain — but it runs without Go and catches the
        common case, which is a test renamed or deleted without an edit here.
        """
        for entry in self.groups:
            for package, names in entry['packages'].items():
                directory = ROOT / 'proof' / package[len('./'):]
                declared = set()
                for source in directory.glob('*_test.go'):
                    declared |= set(re.findall(r'(?m)^func ((?:Test|Example|Fuzz)\w*)\(',
                                               source.read_text()))
                for name in names:
                    self.assertIn(name, declared,
                                  '%s: %s %s is not declared in the package'
                                  % (entry['name'], package, name))


class WorkflowShardWiringTest(unittest.TestCase):
    """CI must take its matrix and its selections from the manifest, never from this YAML.

    A hand-kept `-run` list in the workflow fails open: a test that stops being selected leaves
    every job green. So the workflow is held to calling the owner, and to keeping the aggregate
    that turns seven green legs into the one check a branch rule can ask for.
    """

    def setUp(self):
        self.workflow = (ROOT / '.github' / 'workflows' / '3d-proof.yml').read_text()
        self.manifest = json.loads((ROOT / 'proof' / 'shards.json').read_text())
        self.names = [entry['name'] for entry in self.manifest['groups']]
        # The gates matrix names gears, and several groups are named after the gear they hold,
        # so the "no group is named here" check has to look at the proof jobs alone. Comment
        # lines come out with it: this file's comments discuss `-run` and the manifest by name,
        # and a comment cannot select a test.
        proof_layer = self.workflow.split('\n  proof-groups:\n', 1)[1].split('\n  gates:', 1)[0]
        self.proof_layer = '\n'.join(line for line in proof_layer.splitlines()
                                     if not line.lstrip().startswith('#'))
        self.commands = '\n'.join(line for line in self.workflow.splitlines()
                                  if not line.lstrip().startswith('#'))

    def refute(self, needle, haystack, message):
        """assertNotIn without printing the whole workflow when it fails."""
        self.assertTrue(needle not in haystack, message)

    def test_the_matrix_comes_from_the_manifest(self):
        self.assertIn('proof_shards.py matrix', self.workflow)
        self.assertIn('fromJSON(needs.proof-groups.outputs.groups)', self.workflow)

    def test_each_group_runs_through_the_owner(self):
        self.assertIn("proof_shards.py run '${{ matrix.group }}'", self.workflow)

    def test_the_inventory_is_checked_against_the_toolchain(self):
        self.assertIn('proof_shards.py check', self.workflow)

    def test_the_workflow_names_no_group_and_no_run_expression(self):
        self.assertTrue(self.proof_layer, 'the proof jobs could not be located in the workflow')
        for name in self.names:
            self.refute(name, self.proof_layer,
                        'the proof jobs name group %s; the manifest owns that list' % name)
        self.refute('-run ', self.commands, 'the workflow spells out a -run expression')
        self.refute('--package', self.commands, 'the workflow spells out a package selection')

    def test_every_leg_reports_even_when_one_fails(self):
        self.assertIn('fail-fast: false', self.workflow)

    def test_the_aggregate_keeps_the_name_a_branch_rule_would_ask_for(self):
        self.assertIn('name: Geometry proofs', self.workflow)
        self.assertIn('needs: [proof-groups, proof-inventory, proof-run]', self.workflow)
        # if: always(), or a skipped upstream job would silently skip the aggregate too.
        self.assertIn('if: always()', self.workflow)
        for job in ('proof-groups', 'proof-inventory', 'proof-run'):
            self.assertIn('needs.%s.result' % job, self.workflow)

    def test_every_job_that_runs_go_verifies_the_engine_revisions(self):
        legs = self.workflow.count("PROOF_VERIFY_REVISIONS: '1'")
        self.assertEqual(2, legs,
                         'both the inventory job and the group job must verify the pins')
        self.assertNotIn('PROOF_VERIFY_REVISIONS: 0', self.workflow)

    def test_the_runner_fixtures_still_run(self):
        self.assertIn('bash proof/run_test.sh', self.workflow)

    def test_each_group_caches_its_own_test_results(self):
        """One shared cache entry can hold only one group's results, so the key names the group.

        Go's test cache matches on the test binary and its arguments, and a group's -run
        expression is one of those arguments. A cache entry is immutable, so a key every group
        shares is written once, by whichever job finishes first, and no other group's results
        ever reach it. That was measured: every group executed on every run.
        """
        self.assertIn('proof-build-${{ matrix.group }}-', self.commands)
        self.assertIn('path: ~/.cache/go-build', self.commands)

    def test_a_manifest_edit_rolls_the_group_cache_key(self):
        """Editing a group's tests changes its -run expression, so its cached results expire.

        Without shards.json in the key the stale entry would never be replaced, and the group
        would re-execute on every run from then on.
        """
        build_key = [line for line in self.commands.splitlines()
                     if 'proof-build-${{ matrix.group }}-' in line and 'key:' in line]
        self.assertTrue(build_key, 'the group build cache declares no key')
        self.assertIn('shards.json', build_key[0])

    def build_cache_steps(self):
        """The key line and restore-key lines of every build-and-test cache step, by job."""
        steps = {}
        lines = self.commands.splitlines()
        for index, line in enumerate(lines):
            if 'key:' not in line or 'proof-build-' not in line:
                continue
            job = line.split('proof-build-', 1)[1].split('-', 1)[0]
            restores = []
            for follow in lines[index + 1:]:
                if follow.strip() == 'restore-keys: |':
                    continue
                if not follow.startswith('            proof-build-'):
                    break
                restores.append(follow.strip())
            steps[job] = (line, restores)
        return steps

    def test_a_proof_source_edit_rolls_the_build_cache_key(self):
        """A proof edit that touches no module file must still roll the key.

        actions/cache never rewrites an entry: a restore that hits the primary key skips the
        save, so an entry is frozen at whatever the run that created it stored. A key built from
        the module files and the manifest alone therefore goes stale the moment a proof source
        changes, and the job re-executes on every run from then on while saving nothing.
        """
        steps = self.build_cache_steps()
        self.assertTrue(steps, 'no build-and-test cache step declares a key')
        for job, (key, _) in steps.items():
            self.assertIn("hashFiles('gears/proof/**/*.go')", key,
                          'the %s build cache key ignores the proof source' % job)

    def test_the_build_cache_falls_back_to_its_older_key_shapes(self):
        """The fallbacks must recover a sibling entry and one saved before the source component
        existed, or adding that component costs every job a full rebuild once."""
        module_hash = ("${{ hashFiles('gears/proof/go.sum', 'gears/proof/go.mod', "
                       "'gears/proof/shards.json') }}")
        steps = self.build_cache_steps()
        self.assertTrue(steps, 'no build-and-test cache step declares a key')
        for job, (_, restores) in steps.items():
            self.assertEqual(2, len(restores),
                             'the %s build cache declares %d fallbacks, want 2'
                             % (job, len(restores)))
            self.assertTrue(restores[0].endswith(module_hash + '-'),
                            'the %s build cache has no same-module-graph fallback' % job)
            self.assertTrue(restores[1].endswith('${{ runner.os }}-'),
                            'the %s build cache drops the pre-source key shape' % job)
            self.assertTrue(restores[0].startswith(restores[1]),
                            'the %s build cache fallbacks are not longest-prefix-first' % job)

    def test_the_module_cache_is_shared_and_the_build_cache_is_not(self):
        # go.sum decides the module cache's content, so every group writes the same bytes and
        # one entry is right. Nothing group-specific may appear in its key.
        module_key = [line for line in self.commands.splitlines()
                      if 'proof-mod-' in line and 'key:' in line]
        self.assertTrue(module_key, 'the module cache declares no key')
        for line in module_key:
            self.assertNotIn('matrix.group', line)
            self.assertNotIn('shards.json', line)

    def test_setup_go_does_no_caching_of_its_own(self):
        # setup-go derives one key from go.sum for every job that enables it, which is the
        # sharing this design exists to avoid. The cache steps above own it instead.
        self.refute('cache-dependency-path', self.commands,
                    'a job still lets setup-go derive its own cache key')


if __name__ == '__main__':
    unittest.main()
