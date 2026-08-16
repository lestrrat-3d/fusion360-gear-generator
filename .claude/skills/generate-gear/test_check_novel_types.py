#!/usr/bin/env python3
"""Regression tests for the generated-candidate novel-type baseline."""
import importlib.util
import tempfile
import unittest
from pathlib import Path


CHECKER = Path(__file__).with_name('check_novel_types.py')
SPEC = importlib.util.spec_from_file_location('check_novel_types', CHECKER)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class ReferenceGearTests(unittest.TestCase):
    def test_copied_candidate_excludes_its_source_from_baseline(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            reference = root / 'reference'
            reference.mkdir()
            source = reference / 'spurgear.py'
            source.write_text('source gear\n')
            (reference / 'othergear.py').write_text('other gear\n')
            candidate = root / 'spurgear.generated.py'
            candidate.write_text(source.read_text())

            self.assertEqual(
                MODULE.reference_gears(str(reference), str(candidate)),
                [str(reference / 'othergear.py')],
            )

    def test_in_place_candidate_excludes_itself(self):
        with tempfile.TemporaryDirectory() as directory:
            reference = Path(directory)
            candidate = reference / 'spurgear.py'
            candidate.write_text('source gear\n')
            other = reference / 'othergear.py'
            other.write_text('other gear\n')

            self.assertEqual(
                MODULE.reference_gears(str(reference), str(candidate)),
                [str(other)],
            )


if __name__ == '__main__':
    unittest.main()
