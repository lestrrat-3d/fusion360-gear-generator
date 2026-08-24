#!/usr/bin/env python3
"""Regressions for the cited-rules playbook extractor."""
import contextlib
import importlib.util
import io
import re
import tempfile
import unittest
from pathlib import Path


EXTRACTOR_PATH = Path(__file__).with_name('extract_playbook.py')
MODULE_SPEC = importlib.util.spec_from_file_location('extract_playbook', EXTRACTOR_PATH)
EXTRACTOR = importlib.util.module_from_spec(MODULE_SPEC)
MODULE_SPEC.loader.exec_module(EXTRACTOR)

GENERATE_GEAR = EXTRACTOR_PATH.resolve().parent
REPO_ROOT = GENERATE_GEAR.parent.parent.parent

# One small playbook standing in for the real one: a preamble, two `##` sections
# of bullet rules (one nest included), a heading-defined anchor with a `###`
# subsection under it, and a trailing section to prove a block stops.
FIXTURE_PLAYBOOK = """# Fixture Playbook

Preamble text that every extract carries.

## Alpha

Intro prose for alpha.

- **[PB-ONE] Rule one opener.** First body line of rule one.
  Continuation line of rule one.
- a plain bullet that anchors nothing
- **[PB-TWO] Rule two opener.** Body of rule two.

- **[PB-PARENT] Parent rule opener.** Body of the parent rule.
  - **[PB-CHILD-A] Child A opener.** Body of child A.
  - **[PB-CHILD-B] Child B opener.** Body of child B.
- **[PB-AFTER] After rule opener.** Body of the after rule.

## Beta

Body of beta.

## Gamma the heading anchor ([PB-GAMMA])

Body of gamma.

### Gamma subsection

Body of the gamma subsection.

## Delta

Body of delta.
"""


class ExtractorTestCase(unittest.TestCase):
    """Drive `main` against a fixture playbook in a temporary directory."""

    def run_main(self, cites, playbook=FIXTURE_PLAYBOOK, core_sections=(),
                 write_steps=True):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            steps = root / 'steps.md'
            if write_steps:
                steps.write_text(cites, encoding='utf-8')
            book = root / 'PLAYBOOK.md'
            book.write_text(playbook, encoding='utf-8')
            out_path = root / 'out' / 'extract.md'
            argv = [
                'extract_playbook.py', 'testgear',
                '--steps', str(steps),
                '--playbook', str(book),
                '--out', str(out_path),
            ]
            stdout, stderr = io.StringIO(), io.StringIO()
            with contextlib.redirect_stdout(stdout), contextlib.redirect_stderr(stderr):
                code = EXTRACTOR.main(argv, core_sections=core_sections)
            text = out_path.read_text(encoding='utf-8') if out_path.exists() else ''
        return code, text, stdout.getvalue(), stderr.getvalue()

    def test_bullet_anchor_stops_at_the_next_top_level_bullet(self):
        code, text, _, err = self.run_main('step cites [PB-ONE]\n')
        self.assertEqual(code, 0, err)
        self.assertIn('First body line of rule one.', text)
        self.assertIn('Continuation line of rule one.', text)
        self.assertNotIn('a plain bullet that anchors nothing', text)
        self.assertNotIn('Body of rule two.', text)

    def test_parent_anchor_carries_its_nested_anchors(self):
        code, text, _, err = self.run_main('step cites [PB-PARENT]\n')
        self.assertEqual(code, 0, err)
        self.assertIn('Body of the parent rule.', text)
        self.assertIn('Body of child A.', text)
        self.assertIn('Body of child B.', text)
        self.assertNotIn('Body of the after rule.', text)

    def test_nested_anchor_alone_extracts_only_its_sub_block(self):
        code, text, _, err = self.run_main('step cites [PB-CHILD-A]\n')
        self.assertEqual(code, 0, err)
        self.assertIn('Body of child A.', text)
        self.assertNotIn('Body of child B.', text)
        self.assertNotIn('Body of the parent rule.', text)

    def test_heading_anchor_takes_its_whole_section(self):
        code, text, _, err = self.run_main('step cites [PB-GAMMA]\n')
        self.assertEqual(code, 0, err)
        self.assertIn('Body of gamma.', text)
        self.assertIn('Body of the gamma subsection.', text)
        self.assertNotIn('Body of delta.', text)

    def test_core_sections_survive_a_steps_file_with_no_cites(self):
        code, text, _, err = self.run_main(
            'a step that cites nothing at all\n', core_sections=('## Beta',))
        self.assertEqual(code, 0, err)
        self.assertIn('Preamble text that every extract carries.', text)
        self.assertIn('## Beta', text)
        self.assertIn('Body of beta.', text)
        self.assertNotIn('Body of delta.', text)

    def test_non_playbook_cites_pass_and_are_named_in_the_header(self):
        code, text, _, err = self.run_main('step cites [SPUR-F-X] and [PB-ONE]\n')
        self.assertEqual(code, 0, err)
        self.assertIn('[SPUR-F-X]', text)
        self.assertIn('this stage does not read', text)
        self.assertIn('First body line of rule one.', text)

    def test_undefined_playbook_anchor_exits_1_and_names_it(self):
        code, text, out, err = self.run_main('step cites [PB-NOWHERE]\n')
        self.assertEqual(code, 1)
        self.assertEqual(text, '')
        self.assertEqual(out, '')
        self.assertIn('PB-NOWHERE', err)

    def test_missing_steps_file_exits_2(self):
        code, text, out, err = self.run_main('', write_steps=False)
        self.assertEqual(code, 2)
        self.assertEqual(text, '')
        self.assertEqual(out, '')
        self.assertIn('steps file', err)

    def test_repeated_and_contained_blocks_are_written_once(self):
        code, text, _, err = self.run_main(
            'cites [PB-ONE] then [PB-ONE] again, plus [PB-PARENT] and [PB-CHILD-A]\n')
        self.assertEqual(code, 0, err)
        self.assertEqual(text.count('First body line of rule one.'), 1)
        self.assertEqual(text.count('Body of child A.'), 1)
        self.assertEqual(text.count('Body of the parent rule.'), 1)

    def test_playbook_order_is_preserved_with_one_heading_trail(self):
        code, text, _, err = self.run_main(
            'cites [PB-GAMMA] first, then [PB-TWO], then [PB-ONE]\n')
        self.assertEqual(code, 0, err)
        one = text.index('First body line of rule one.')
        two = text.index('Body of rule two.')
        gamma = text.index('Body of gamma.')
        self.assertLess(one, two)
        self.assertLess(two, gamma)
        # both bullet blocks live under Alpha, so the trail is written once
        self.assertEqual(text.count('## Alpha'), 1)
        self.assertIn('## Gamma the heading anchor ([PB-GAMMA])', text)

    def test_missing_core_section_exits_1_and_names_it(self):
        code, text, out, err = self.run_main(
            'cites [PB-ONE]\n', core_sections=('## Alpha', '## No Such Section'))
        self.assertEqual(code, 1)
        self.assertEqual(text, '')
        self.assertEqual(out, '')
        self.assertIn('## No Such Section', err)
        self.assertNotIn('## Alpha', err)


class CommittedRepoTest(unittest.TestCase):
    """Run the extractor over the real spurgear step list and playbook."""

    def test_spurgear_extract_covers_every_cited_playbook_anchor(self):
        playbook = GENERATE_GEAR / 'PLAYBOOK.md'
        steps = REPO_ROOT / 'spec' / 'spurgear' / 'steps.md'
        book_lines = playbook.read_text(encoding='utf-8').splitlines()

        with tempfile.TemporaryDirectory() as directory:
            out_path = Path(directory) / 'spurgear.playbook-extract.md'
            stdout, stderr = io.StringIO(), io.StringIO()
            with contextlib.redirect_stdout(stdout), contextlib.redirect_stderr(stderr):
                code = EXTRACTOR.main(
                    ['extract_playbook.py', 'spurgear', '--out', str(out_path)])
            self.assertEqual(code, 0, stderr.getvalue())
            text = out_path.read_text(encoding='utf-8')

        cited = {
            anchor for anchor in
            EXTRACTOR.collect_cites(steps.read_text(encoding='utf-8').splitlines())
            if anchor.startswith(EXTRACTOR.PLAYBOOK_PREFIX)
        }
        self.assertGreater(len(cited), 15, 'the spur step list should cite many playbook rules')

        defs = EXTRACTOR.index_definitions(book_lines)
        for anchor in sorted(cited):
            with self.subTest(anchor=anchor):
                self.assertIn(anchor, defs, 'cited playbook anchor has no definition')
                self.assertIn(book_lines[defs[anchor][0]], text,
                              'the extract dropped the line defining this rule')

        self.assertLess(
            len(text.encode('utf-8')),
            len(playbook.read_bytes()) // 2,
            'the extract should be well under half the playbook')

    def test_definition_index_agrees_with_check_anchors(self):
        """Every anchor `check_anchors.py` sees defined in the playbook is indexed."""
        playbook = GENERATE_GEAR / 'PLAYBOOK.md'
        text = playbook.read_text(encoding='utf-8')
        gate_defs = set()
        for line in text.splitlines():
            gate_defs.update(m.group(1) for m in EXTRACTOR.BOLD_DEF_RE.finditer(line))
            if line.lstrip().startswith('#'):
                gate_defs.update(m.group(1) for m in EXTRACTOR.CITE_RE.finditer(line))
        self.assertEqual(gate_defs, set(EXTRACTOR.index_definitions(text.splitlines())))

    def test_core_sections_exist_in_the_committed_playbook(self):
        text = (GENERATE_GEAR / 'PLAYBOOK.md').read_text(encoding='utf-8')
        headings = {line.rstrip() for line in text.splitlines()
                    if re.match(r'#{1,6}\s', line)}
        for heading in EXTRACTOR.CORE_SECTIONS:
            with self.subTest(heading=heading):
                self.assertIn(heading, headings)


if __name__ == '__main__':
    unittest.main()
