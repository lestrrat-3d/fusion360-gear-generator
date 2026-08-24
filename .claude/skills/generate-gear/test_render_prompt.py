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


class RenderCase(unittest.TestCase):
    """Drive `main` against a temporary skills tree."""

    def run_main(self, args, templates=None, files=None):
        """Render `args` against a temp skills tree, with optional extra files.

        `files` maps a name under the temp root to its content: `str` is written as UTF-8,
        `bytes` verbatim. `{root}` in an argument is filled in with the temp root, so a
        case can point `--failure-file` at one of those files.
        """
        templates = templates or {}
        files = files or {}
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            for skill, text in templates.items():
                skill_dir = root / skill
                skill_dir.mkdir(parents=True, exist_ok=True)
                (skill_dir / 'prompt.md').write_text(text, encoding='utf-8')
            for name, content in files.items():
                target = root / name
                target.parent.mkdir(parents=True, exist_ok=True)
                if isinstance(content, bytes):
                    target.write_bytes(content)
                else:
                    target.write_text(content, encoding='utf-8')
            args = [str(arg).replace('{root}', str(root)) for arg in args]
            out, err = io.StringIO(), io.StringIO()
            with contextlib.redirect_stdout(out), contextlib.redirect_stderr(err):
                code = RENDERER.main(['render_prompt.py'] + list(args), skills_root=root)
        return code, out.getvalue(), err.getvalue()


class RenderUnitTest(RenderCase):
    """Rendering, substitution and the refusals that predate `--failure-file`."""

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


class FailureFileTest(RenderCase):
    """`--failure-file` appends a stored gate report, or refuses."""

    TEMPLATE = 'draft {{gear}} now\n'
    REPORT = (
        'GATE parse FAIL\n'
        '```\n'
        'File ".tmp/spurgear.generated.py", line 3\n'
        '```\n'
        'trailing }} brace and a { single one\n'
    )

    def render_with_report(self, skill='emit-gear', gear='spurgear', report=None):
        report = self.REPORT if report is None else report
        return self.run_main(
            [skill, gear, '--failure-file', '{root}/gates.txt'],
            {skill: self.TEMPLATE},
            {'gates.txt': report})

    def test_failure_file_appends_verbatim(self):
        code, out, err = self.render_with_report()
        self.assertEqual(code, 0, err)
        self.assertTrue(out.startswith('draft spurgear now\n'), out)
        self.assertEqual(out.count(RENDERER.BEGIN_MARKER), 1)
        self.assertEqual(out.count(RENDERER.END_MARKER), 1)
        body = out.split(RENDERER.BEGIN_MARKER + '\n', 1)[1]
        body = body.split('\n' + RENDERER.END_MARKER, 1)[0]
        self.assertEqual(body + '\n', self.REPORT)

    def test_failure_file_content_not_substituted(self):
        code, out, err = self.render_with_report(report='saw {{gear}} in the report\n')
        self.assertEqual(code, 0, err)
        self.assertIn('draft spurgear now', out)
        self.assertIn('saw {{gear}} in the report', out)

    def test_failure_file_missing_exits_2(self):
        code, out, err = self.run_main(
            ['emit-gear', 'spurgear', '--failure-file', '{root}/nope.txt'],
            {'emit-gear': self.TEMPLATE})
        self.assertEqual(code, 2)
        self.assertEqual(out, '')
        self.assertIn('nope.txt', err.replace('\\', '/'))

    def test_failure_file_empty_exits_2(self):
        for report in ('', '   \n\n\t\n'):
            with self.subTest(report=report):
                code, out, err = self.render_with_report(report=report)
                self.assertEqual(code, 2)
                self.assertEqual(out, '')
                self.assertIn('empty', err)

    def test_failure_file_refused_for_generate_gear(self):
        code, out, err = self.render_with_report(skill='generate-gear')
        self.assertEqual(code, 2)
        self.assertEqual(out, '')
        for skill in RENDERER.FAILURE_FEEDBACK_SKILLS:
            self.assertIn(skill, err)

    def test_failure_file_flag_needs_value(self):
        cases = {
            'last arg': ['emit-gear', 'spurgear', '--failure-file'],
            'given twice': [
                'emit-gear', 'spurgear',
                '--failure-file', '{root}/gates.txt',
                '--failure-file', '{root}/gates.txt'],
        }
        for name, args in cases.items():
            with self.subTest(case=name):
                code, out, err = self.run_main(
                    args, {'emit-gear': self.TEMPLATE}, {'gates.txt': self.REPORT})
                self.assertEqual(code, 2)
                self.assertEqual(out, '')
                self.assertIn('--failure-file', err)

    def test_failure_file_not_utf8_exits_2(self):
        code, out, err = self.render_with_report(report=b'\xff\xfe GATE parse FAIL\n')
        self.assertEqual(code, 2)
        self.assertEqual(out, '')
        self.assertIn('UTF-8', err)

    def test_without_flag_output_unchanged(self):
        with_flag = self.render_with_report()
        without = self.run_main(['emit-gear', 'spurgear'], {'emit-gear': self.TEMPLATE})
        self.assertEqual(without, (0, 'draft spurgear now\n', ''))
        self.assertTrue(with_flag[1].startswith(without[1]), with_flag[1])
        self.assertEqual(
            with_flag[1][len(without[1]):],
            RENDERER.failure_block(self.REPORT))

    def test_trailing_newline_normalized(self):
        code, out, err = self.render_with_report(report='GATE parse FAIL')
        self.assertEqual(code, 0, err)
        self.assertIn('\nGATE parse FAIL\n' + RENDERER.END_MARKER + '\n', out)
        self.assertTrue(out.endswith(RENDERER.END_MARKER + '\n'), out)

    def test_compile_gear_is_allowed(self):
        code, out, err = self.render_with_report(skill='compile-gear')
        self.assertEqual(code, 0, err)
        self.assertIn(RENDERER.BEGIN_MARKER, out)


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
