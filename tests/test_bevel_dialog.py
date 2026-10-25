#!/usr/bin/env python3
"""The bevel dialog's Tooth Type drop-down shows and hides the spiral-only inputs.

A fake command-input collection stands in for Fusion's. The inputs a spiral tooth needs, Mean Spiral Angle, Hand of Spiral
and Cutter Radius, show when the Tooth Type is Spiral and hide when it is Straight, and the inputs both types use stay.
"""
import importlib
import importlib.util
import sys
import types
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest import mock

ROOT = Path(__file__).resolve().parents[1]


class FakeInputs:
    def __init__(self, tooth):
        self.items = {}
        for input_id in ('spiralAngle', 'spiralHand', 'cutterRadius', 'toeExtension', 'faceWidth'):
            self.items[input_id] = SimpleNamespace(isVisible=True)
        self.items['toothType'] = SimpleNamespace(isVisible=True, selectedItem=SimpleNamespace(name=tooth))

    def itemById(self, input_id):
        return self.items.get(input_id)


class BevelDialogTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.saved = {n: m for n, m in sys.modules.items() if n == 'adsk' or n.startswith(('adsk.', 'addin'))}
        adsk = types.ModuleType('adsk')
        adsk.__path__ = []
        for name in ('core', 'fusion'):
            stub = mock.MagicMock(name=f'adsk.{name}')
            sys.modules[f'adsk.{name}'] = stub
            setattr(adsk, name, stub)
        sys.modules['adsk'] = adsk
        spec = importlib.util.spec_from_file_location(
            'addin', ROOT / '__init__.py', submodule_search_locations=[str(ROOT)])
        sys.modules['addin'] = importlib.util.module_from_spec(spec)
        cls.bevel = importlib.import_module('addin.lib.geargen.bevelgear')

    @classmethod
    def tearDownClass(cls):
        for name in [n for n in sys.modules if n == 'adsk' or n.startswith(('adsk.', 'addin'))]:
            del sys.modules[name]
        sys.modules.update(cls.saved)

    def test_straight_teeth_hide_the_spiral_inputs(self):
        inputs = FakeInputs('Straight')
        self.bevel.BevelGearCommandInputsConfigurator._updateSpiralInputVisibility(inputs)
        for input_id in ('spiralAngle', 'spiralHand', 'cutterRadius'):
            self.assertFalse(inputs.items[input_id].isVisible, input_id)
        for input_id in ('toeExtension', 'faceWidth', 'toothType'):
            self.assertTrue(inputs.items[input_id].isVisible, input_id)
        self.assertFalse(self.bevel._is_spiral(inputs))

    def test_spiral_teeth_show_the_spiral_inputs(self):
        inputs = FakeInputs('Spiral')
        inputs.items['spiralAngle'].isVisible = False
        self.bevel.BevelGearCommandInputsConfigurator._updateSpiralInputVisibility(inputs)
        for input_id in ('spiralAngle', 'spiralHand', 'cutterRadius'):
            self.assertTrue(inputs.items[input_id].isVisible, input_id)
        self.assertTrue(self.bevel._is_spiral(inputs))

    def test_a_dialog_with_no_tooth_type_builds_straight_teeth(self):
        self.assertFalse(self.bevel._is_spiral(SimpleNamespace(itemById=lambda input_id: None)))


if __name__ == '__main__':
    unittest.main()
