#!/usr/bin/env python3
"""Regressions for the standard-drafting-prompt renderer."""
import contextlib
import importlib.util
import io
import re
import tempfile
import unittest
from pathlib import Path


RENDERER_PATH = Path(__file__).with_name('render_prompt.py')
MODULE_SPEC = importlib.util.spec_from_file_location('render_prompt', RENDERER_PATH)
RENDERER = importlib.util.module_from_spec(MODULE_SPEC)
MODULE_SPEC.loader.exec_module(RENDERER)

SKILLS_ROOT = RENDERER_PATH.resolve().parent.parent


class RenderUnitTest(unittest.TestCase):
    """Drive `main` against a temporary skills tree."""

    def run_main(self, args, templates=None):
        templates = templates or {}
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            for skill, text in templates.items():
                skill_dir = root / skill
                skill_dir.mkdir(parents=True, exist_ok=True)
                (skill_dir / 'prompt.md').write_text(text, encoding='utf-8')
            out, err = io.StringIO(), io.StringIO()
            with contextlib.redirect_stdout(out), contextlib.redirect_stderr(err):
                code = RENDERER.main(['render_prompt.py'] + list(args), skills_root=root)
        return code, out.getvalue(), err.getvalue()

    def test_renders_and_substitutes(self):
        code, out, err = self.run_main(
            ['compile-gear', 'spurgear'], {'compile-gear': 'a {{gear}} b'})
        self.assertEqual(code, 0, err)
        self.assertEqual(out, 'a spurgear b')

    def test_default_gear_is_spurgear(self):
        code, out, err = self.run_main(
            ['compile-gear'], {'compile-gear': 'a {{gear}} b'})
        self.assertEqual(code, 0, err)
        self.assertIn('spurgear', out)

    def test_unknown_skill_exits_2(self):
        code, out, err = self.run_main(
            ['frobnicate-gear'], {'compile-gear': '{{gear}}'})
        self.assertEqual(code, 2)
        self.assertEqual(out, '')
        for skill in RENDERER.KNOWN_SKILLS:
            self.assertIn(skill, err)

    def test_missing_template_exits_2(self):
        code, out, err = self.run_main(['emit-gear'], {'compile-gear': '{{gear}}'})
        self.assertEqual(code, 2)
        self.assertEqual(out, '')
        self.assertIn('emit-gear/prompt.md', err.replace('\\', '/'))

    def test_invalid_gear_name_exits_2(self):
        for gear in ('SpurGear', 'spur gear', '../etc', '<gear>', '{{gear}}', ''):
            with self.subTest(gear=gear):
                code, out, err = self.run_main(
                    ['compile-gear', gear], {'compile-gear': 'a {{gear}} b'})
                self.assertEqual(code, 2)
                self.assertEqual(out, '')
                self.assertIn('render_prompt:', err)

    def test_unknown_placeholder_exits_2(self):
        code, out, err = self.run_main(
            ['compile-gear'], {'compile-gear': '{{gear}} has {{teeth}}'})
        self.assertEqual(code, 2)
        self.assertEqual(out, '')
        self.assertIn('teeth', err)

    def test_template_without_gear_placeholder_exits_2(self):
        code, out, err = self.run_main(
            ['compile-gear'], {'compile-gear': 'hard-coded spurgear\n'})
        self.assertEqual(code, 2)
        self.assertEqual(out, '')
        self.assertIn('gear', err)

    def test_stray_braces_exit_2(self):
        code, out, err = self.run_main(
            ['compile-gear'], {'compile-gear': 'a {{gear}} and {{gear}\n'})
        self.assertEqual(code, 2)
        self.assertEqual(out, '')

    def test_angle_bracket_literals_untouched(self):
        template = 'ask about <Class>.<member>, write <file>, log <unix-epoch-seconds> <n>\n{{gear}}\n'
        code, out, err = self.run_main(['compile-gear', 'bevel'], {'compile-gear': template})
        self.assertEqual(code, 0, err)
        self.assertEqual(out, template.replace('{{gear}}', 'bevel'))

    def test_output_preserves_template_bytes(self):
        template = '  leading space\n\n\ttab line\ntrailing spaces   \n{{gear}} end'
        code, out, err = self.run_main(['emit-gear', 'helical'], {'emit-gear': template})
        self.assertEqual(code, 0, err)
        self.assertEqual(out, template.replace('{{gear}}', 'helical'))


class CommittedTemplatesTest(unittest.TestCase):
    """Check the templates and SKILL.md files that ship in the repo."""

    def render(self, skill, gear):
        out, err = io.StringIO(), io.StringIO()
        with contextlib.redirect_stdout(out), contextlib.redirect_stderr(err):
            code = RENDERER.main(['render_prompt.py', skill, gear])
        return code, out.getvalue(), err.getvalue()

    def test_all_templates_render(self):
        for skill in RENDERER.KNOWN_SKILLS:
            with self.subTest(skill=skill):
                code, out, err = self.render(skill, 'testgear')
                self.assertEqual(code, 0, err)
                self.assertIn('testgear', out)
                self.assertNotIn('{{', out)
                self.assertNotIn('}}', out)

    def test_templates_declare_only_gear(self):
        for skill in RENDERER.KNOWN_SKILLS:
            with self.subTest(skill=skill):
                text = (SKILLS_ROOT / skill / 'prompt.md').read_text(encoding='utf-8')
                names = RENDERER.PLACEHOLDER_RE.findall(text)
                self.assertIn('gear', names)
                self.assertEqual(set(names), {'gear'})

    def test_emit_prompt_gate_scripts_exist(self):
        text = (SKILLS_ROOT / 'emit-gear' / 'prompt.md').read_text(encoding='utf-8')
        names = {
            name for name in re.findall(r'\w+\.py', text)
            if name.startswith('check_') or name in ('pyright_check.py', 'run_gates.py')
        }
        self.assertTrue(names, 'emit prompt should name the gate scripts')
        for name in sorted(names):
            with self.subTest(script=name):
                self.assertTrue(
                    (SKILLS_ROOT / 'generate-gear' / name).is_file(),
                    '{} named by the emit prompt does not exist'.format(name))

    def test_skill_md_references_renderer_not_a_copy(self):
        for skill in RENDERER.KNOWN_SKILLS:
            with self.subTest(skill=skill):
                skill_md = (SKILLS_ROOT / skill / 'SKILL.md').read_text(encoding='utf-8')
                self.assertIn('render_prompt.py {}'.format(skill), skill_md)
                template = (SKILLS_ROOT / skill / 'prompt.md').read_text(encoding='utf-8')
                first_line = template.splitlines()[0].replace('{{gear}}', '<gear>')
                self.assertNotIn(
                    first_line, skill_md,
                    'SKILL.md re-inlines the prompt instead of pointing at prompt.md')


if __name__ == '__main__':
    unittest.main()
