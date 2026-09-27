"""Focused checks for the spur exact-value source and generated setup."""
import copy
import os
import sys
import unittest

HERE = os.path.dirname(__file__)
ROOT = os.path.abspath(os.path.join(HERE, '..', '..', '..'))
sys.path.insert(0, HERE)

import exact_values


class ExactValuesTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.data, cls.constants = exact_values.load(ROOT, 'spurgear')

    def test_spur_values_preserve_existing_dialog_and_parameters(self):
        inputs = self.data['inputs']
        self.assertEqual([self.constants[item['id']] for item in inputs], [
            'plane', 'anchorPoint', 'module', 'toothNumber', 'pressureAngle',
            'boreDiameter', 'thickness', 'chamferTooth', 'sketchOnly', 'parentComponent'])
        self.assertEqual([item['label'] for item in inputs], [
            'Target Plane', 'Anchor Point', 'Module', 'Tooth Number', 'Pressure Angle',
            'Bore Diameter', 'Thickness', 'Apply chamfer to teeth',
            'Generate sketches, but do not build body', 'Parent Component'])
        self.assertEqual([item.get('unit') for item in inputs], [
            None, None, '', '', 'deg', None, 'mm', 'mm', None, None])
        self.assertEqual([item.get('default') for item in inputs], [
            None, None, {'real': 1}, {'real': 17}, {'radians': 20}, '0 mm',
            {'millimeters': 10}, {'real': 0}, False, None])
        self.assertEqual([item['prompt'] for item in inputs if item['kind'] == 'selection'], [
            'Select the plane to build the gear on',
            'Select the point the gear is centered on',
            'Select the component to build the gear in'])
        params = self.data['parameters']
        self.assertEqual([self.constants[item['name']] for item in params], [
            'Module', 'ToothNumber', 'PressureAngle', 'BoreDiameter', 'Thickness',
            'ChamferTooth', 'SketchOnly', 'PitchCircleDiameter', 'PitchCircleRadius',
            'BaseCircleDiameter', 'BaseCircleRadius', 'RootCircleDiameter',
            'RootCircleRadius', 'TipCircleDiameter', 'TipCircleRadius', 'InvoluteSteps',
            'ToothSpaceAngleAtRoot', 'ToothSpaceArcAtRoot', 'FilletClearance', 'FilletRadius'])
        self.assertEqual([item['unit'] for item in params], [
            '', '', 'rad', 'mm', 'mm', 'mm', '', 'mm', 'mm', 'mm', 'mm', 'mm',
            'mm', 'mm', 'mm', '', '', 'mm', '', 'mm'])
        self.assertEqual([item['comment'] for item in params], [
            'Module of the gear', 'Number of teeth', 'Pressure angle', 'Bore diameter',
            'Thickness of the gear', 'Chamfer distance applied to the teeth',
            'Generate sketches only', 'Pitch circle diameter', 'Pitch circle radius',
            'Base circle diameter', 'Base circle radius', 'Root circle diameter',
            'Root circle radius', 'Tip circle diameter', 'Tip circle radius',
            'Number of points sampled along each involute flank',
            'Angular width of the tooth space at the root circle',
            'Arc length of the tooth space at the root circle',
            'Clearance factor applied to the root fillet radius', 'Radius of the root fillets'])
        self.assertEqual([item.get('expression') for item in params[7:]], [
            '{PARAM_MODULE} * {PARAM_TOOTH_NUMBER}', '{PARAM_PITCH_DIAMETER} / 2',
            '{PARAM_PITCH_DIAMETER} * cos({PARAM_PRESSURE_ANGLE})',
            '{PARAM_BASE_DIAMETER} / 2',
            '{PARAM_PITCH_DIAMETER} - 2.5 * {PARAM_MODULE}',
            '{PARAM_ROOT_DIAMETER} / 2',
            '{PARAM_PITCH_DIAMETER} + 2 * {PARAM_MODULE}',
            '{PARAM_TIP_DIAMETER} / 2', '15', None,
            '{PARAM_ROOT_RADIUS} * {PARAM_TOOTH_SPACE_ANGLE}', '0.9',
            '({PARAM_TOOTH_SPACE_ARC} / 2) * {PARAM_FILLET_CLEARANCE} * {fillet_helix_factor}'])

    def test_duplicate_and_missing_inputs(self):
        data = copy.deepcopy(self.data)
        data['inputs'][1]['id'] = data['inputs'][0]['id']
        with self.assertRaisesRegex(exact_values.ExactValueError, 'duplicate input'):
            exact_values.validate(data, self.constants)
        data = copy.deepcopy(self.data)
        del data['inputs'][0]['label']
        with self.assertRaisesRegex(exact_values.ExactValueError, 'exactly'):
            exact_values.validate(data, self.constants)

    def test_invalid_unit_unknown_reference_and_cycle(self):
        data = copy.deepcopy(self.data)
        data['parameters'][0]['unit'] = 'bogus'
        with self.assertRaisesRegex(exact_values.ExactValueError, 'invalid unit'):
            exact_values.validate(data, self.constants)
        data = copy.deepcopy(self.data)
        data['parameters'][7]['expression'] = '{PARAM_UNKNOWN} * 2'
        with self.assertRaisesRegex(exact_values.ExactValueError, 'unknown parameter'):
            exact_values.validate(data, self.constants)
        data = copy.deepcopy(self.data)
        data['parameters'][7]['expression'] = '{PARAM_PITCH_RADIUS} * 2'
        with self.assertRaisesRegex(exact_values.ExactValueError, 'cycle'):
            exact_values.validate(data, self.constants)

    def test_handoff_and_module_drift(self):
        with open(os.path.join(ROOT, 'spec', 'spurgear', 'steps.md'), encoding='utf-8') as handle:
            steps = handle.read()
        handoff = exact_values.check_steps(steps, self.data, self.constants)
        with open(os.path.join(ROOT, 'lib', 'geargen', 'spurgear.py'), encoding='utf-8') as handle:
            source = handle.read()
        self.assertEqual(exact_values.render_module(source, handoff), source)
        changed = steps.replace('"Target Plane"', '"Wrong Plane"', 1)
        with self.assertRaisesRegex(exact_values.ExactValueError, 'differ'):
            exact_values.check_steps(changed, self.data, self.constants)
        changed_module = source.replace("'Target Plane'", "'Wrong Plane'", 1)
        self.assertNotEqual(exact_values.render_module(changed_module, handoff), changed_module)
        unsafe = source.replace('        self.addExtraPrimaryParameters(inputs)',
                                '        self.buildBody(None)\n        self.addExtraPrimaryParameters(inputs)', 1)
        with self.assertRaisesRegex(exact_values.ExactValueError, 'non-parameter code'):
            exact_values.render_module(unsafe, handoff)

    def test_renderer_fills_emitter_skeleton_without_transcription(self):
        skeleton = '''import math
class SpurGearCommandInputsConfigurator:
    @classmethod
    def configure(cls, cmd):
        pass
class SpurGearGenerator:
    def processInputs(self, inputs):
        self.parentComponent = None
        self.registerDerivedParameters()
    def registerDerivedParameters(self):
        pass
'''
        handoff = exact_values.payload(self.data, self.constants)
        result = exact_values.render_module(skeleton, handoff)
        self.assertIn("INPUT_ID_PLANE = 'plane'", result)
        self.assertIn("'Select the plane to build the gear on'", result)
        self.assertIn('self.addExtraPrimaryParameters(inputs)', result)
        self.assertIn('PARAM_FILLET_RADIUS, adsk.core.ValueInput.createByString(expression)', result)
        self.assertEqual(exact_values.render_module(result, handoff), result)

    def test_helical_extension_uses_the_same_schema_and_renderer(self):
        data, constants = exact_values.load(ROOT, 'helicalgear')
        self.assertEqual(constants['INPUT_ID_HELIX_ANGLE'], 'helixAngle')
        self.assertEqual(constants['PARAM_HELIX_ANGLE'], 'HelixAngle')
        self.assertEqual(data['inputs'], [{
            'id': 'INPUT_ID_HELIX_ANGLE', 'kind': 'value', 'label': 'Helix Angle',
            'unit': 'deg', 'default': {'radians': 14.5}}])
        self.assertEqual(data['parameters'], [{
            'name': 'PARAM_HELIX_ANGLE', 'unit': 'rad',
            'comment': 'Helix angle for the helical gear', 'input': 'INPUT_ID_HELIX_ANGLE'}])
        with open(os.path.join(ROOT, 'spec', 'helicalgear', 'steps.md'), encoding='utf-8') as handle:
            handoff = exact_values.check_steps(handle.read(), data, constants)
        with open(os.path.join(ROOT, 'lib', 'geargen', 'helicalgear.py'), encoding='utf-8') as handle:
            source = handle.read()
        self.assertEqual(exact_values.render_module(source, handoff), source)

        skeleton = '''class HelicalGearCommandConfigurator(SpurGearCommandInputsConfigurator):
    @classmethod
    def configure(cls, cmd):
        pass
class HelicalGearGenerator(SpurGearGenerator):
    def addExtraPrimaryParameters(self, inputs):
        pass
'''
        rendered = exact_values.render_module(skeleton, handoff)
        self.assertIn('super().configure(cmd)', rendered)
        self.assertIn('math.radians(14.5)', rendered)
        self.assertIn("get_value(inputs, INPUT_ID_HELIX_ANGLE, 'rad')", rendered)
        self.assertIn("'Helix angle for the helical gear'", rendered)
        self.assertEqual(exact_values.render_module(rendered, handoff), rendered)


if __name__ == '__main__':
    unittest.main()
