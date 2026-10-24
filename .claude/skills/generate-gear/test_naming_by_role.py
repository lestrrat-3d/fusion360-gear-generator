#!/usr/bin/env python3
"""Every object a gear add-in creates in Fusion is named by its role (see CLAUDE.md, "Naming").

The scan reads lib/geargen/*.py and flags a sketch, construction plane, axis or point,
timeline feature or new component that is created without a name. A name is a
`<variable>.name = ...` assignment or a `name_feature(<variable>, ...)` call.
"""
import re
import unittest
from pathlib import Path

GEARGEN = Path(__file__).resolve().parents[3] / 'lib' / 'geargen'

KINDS = (r'sketches|constructionPlanes|constructionAxes|constructionPoints|\w+Features|'
         r'occurrences')
CREATOR = re.compile(
    r'(?m)^[ \t]*(?:(\w+)\s*=\s*)?[^\n=#]*?\b(' + KINDS + r')\.(add|addNewComponent)\(')

# Gears still built by the compile+emit pipeline and not yet regenerated with the rule. Each entry is the number of unnamed creations the scan finds today. The
# test fails when a count changes, so an entry is removed when its gear is regenerated and
# can never grow.
PENDING_REGENERATION = {'cycloidal.py': 17}


def unnamed(path):
    """Returns (line, kind, variable) for each creation in path that is never named."""
    text = path.read_text()
    found = []
    for m in CREATOR.finditer(text):
        var, kind = m.group(1), m.group(2)
        named = var and (
            re.search(r'\b' + re.escape(var) + r'(\.component)?\.name\s*=', text) or
            re.search(r'name_feature\(\s*' + re.escape(var) + r'\b', text))
        if not named:
            found.append((text.count('\n', 0, m.start()) + 1, kind, var))
    return found


class NamingByRoleTest(unittest.TestCase):
    def test_every_created_object_is_named(self):
        files = sorted(GEARGEN.glob('*.py'))
        self.assertTrue(files)
        for path in files:
            if path.name in PENDING_REGENERATION:
                continue
            with self.subTest(gear=path.name):
                self.assertEqual(
                    [], unnamed(path),
                    f'{path.name} creates objects without a name: name each by its role '
                    '(CLAUDE.md, "Naming"); for a ported gear change its fragment under '
                    'port/<gear>/fragments/ and run gearport gen')

    def test_pending_regeneration_list_is_exact(self):
        for name, count in PENDING_REGENERATION.items():
            with self.subTest(gear=name):
                self.assertEqual(
                    count, len(unnamed(GEARGEN / name)),
                    f'{name} changed: update PENDING_REGENERATION, and remove the entry once '
                    'the gear is regenerated with every object named')


if __name__ == '__main__':
    unittest.main()
