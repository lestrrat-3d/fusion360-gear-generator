#!/usr/bin/env python3
"""A failed build leaves no user parameter behind.

The bound sketch dimensions and the extents of the gear's features live in its component and reference the gear's
user parameters. A parameter with dependents is not deletable, so the cleaner deletes the occurrence before the
parameters.
"""
import importlib
import unittest
from types import SimpleNamespace
from unittest import mock

from fakefusion import FusionTestCase


class FakeParameter:
    def __init__(self, name, design):
        self.name, self.design = name, design

    @property
    def isDeletable(self):
        # Referenced by the component's dimensions and features while the occurrence exists.
        return not self.design.occurrenceAlive

    def deleteMe(self):
        if not self.isDeletable:
            return True  # The reference says deleteMe can answer True without deleting.
        self.design.userParameters.remove(self)
        return True


class ComponentCleanerTest(FusionTestCase):
    def test_the_parameters_go_with_the_occurrence(self):
        base = importlib.import_module('addin.lib.geargen.base')
        design = SimpleNamespace(occurrenceAlive=True, userParameters=[])
        design.userParameters.extend(FakeParameter(n, design) for n in ('Spur_1_Module', 'Spur_1_Teeth', 'Other'))
        occurrence = SimpleNamespace(deleteMe=lambda: setattr(design, 'occurrenceAlive', False))
        with mock.patch.object(base, 'get_design', return_value=design):
            base.ComponentCleaner('Spur_1', occurrence).deleteAll()
        self.assertFalse(design.occurrenceAlive)
        self.assertEqual(['Other'], [p.name for p in design.userParameters])


if __name__ == '__main__':
    unittest.main()
