import gzip
import hashlib
import json
import os
import unittest


ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..'))
PROBE_DIR = os.path.join(ROOT, 'diagnostics', 'projection_probe')
RESULT_DIR = os.path.join(PROBE_DIR, 'results')
RESULT_NAME = 'projection_probe_results_20260907T192923_209867.json.gz'
RESULT_SHA256 = '71564ab2f16c12e6004fddf4d221bdc68e25b09e8d62be5bd59cfe7263e89885'


def available(value):
    return {'status': 'available', 'value': value}


class ProjectionProbeEvidenceTests(unittest.TestCase):
    def setUp(self):
        result_path = os.path.join(RESULT_DIR, RESULT_NAME)
        with gzip.open(result_path, 'rb') as handle:
            self.result_bytes = handle.read()
        self.result = json.loads(self.result_bytes)
        self.stages = {stage['name']: stage for stage in self.result['stages']}

    def test_retained_result_is_the_reviewed_native_run(self):
        self.assertEqual(hashlib.sha256(self.result_bytes).hexdigest(), RESULT_SHA256)
        self.assertEqual(self.result['fusion_version'], available('2704.1.53'))
        self.assertEqual(self.result['native_run']['status'], 'error')
        self.assertEqual([(stage['name'], stage['status']) for stage in self.result['stages']], [
            ('create_and_verify_owned_design', 'complete'),
            ('create_root_source_and_child_context', 'complete'),
            ('project_source_to_reference_only_tools', 'complete'),
            ('build_bore_recipe', 'complete'),
            ('move_source_before_fixed_anchor_repair', 'complete'),
            ('add_fixed_local_anchor_repair', 'error'),
        ])

        source = self.stages['create_root_source_and_child_context']['observations']
        self.assertEqual(source['source_fixed_readback'], available(True))
        self.assertEqual(source['source_point']['is_fully_constrained'], available(True))

        tools = self.stages['project_source_to_reference_only_tools']['observations']
        self.assertTrue(tools['compute_all_return'])
        self.assertEqual(tools['projection_call']['expression'], 'sketch.project(entity)')
        self.assertFalse(tools['projection_call']['fallback_used'])
        self.assertEqual(tools['projected_point']['is_fixed'], available(False))
        self.assertEqual(tools['projected_point']['is_fully_constrained'], available(True))
        self.assertEqual(tools['projected_point']['is_linked'], available(True))
        self.assertEqual(tools['projected_point']['is_reference'], available(True))
        self.assertEqual(tools['reference_only_sketch']['is_fully_constrained'], available(True))

        bore = self.stages['build_bore_recipe']['observations']
        self.assertTrue(bore['compute_before_coincidence_return'])
        self.assertTrue(bore['compute_after_coincidence_return'])
        self.assertEqual(bore['before_coincidence']['source_point_is_fixed'], available(True))
        self.assertEqual(bore['before_coincidence']['diameter_dimension_is_driving'], available(True))
        self.assertEqual(
            bore['before_coincidence']['movable_local_origin']['is_fixed'], available(False))
        self.assertEqual(
            bore['before_coincidence']['movable_local_origin']['is_fully_constrained'], available(False))
        self.assertEqual(bore['before_coincidence']['sketch']['is_fully_constrained'], available(False))
        self.assertEqual(bore['after_coincidence']['sketch']['is_fully_constrained'], available(True))
        self.assertEqual(bore['after_coincidence']['circle']['is_fully_constrained'], available(True))
        self.assertEqual(
            bore['after_coincidence']['movable_local_origin']['is_fully_constrained'], available(True))

        movement = self.stages['move_source_before_fixed_anchor_repair']['observations']
        self.assertTrue(movement['move_return'])
        self.assertTrue(movement['compute_all_return'])
        expected_delta = {'x': 0.5, 'y': 0.25, 'z': 0.0}
        self.assertEqual(set(movement['observed_world_deltas_cm']), {
            'source', 'tools_projection', 'bore_projection', 'bore_circle_center', 'bore_local_origin',
        })
        for delta in movement['observed_world_deltas_cm'].values():
            self.assertEqual(delta, expected_delta)

        repair = self.stages['add_fixed_local_anchor_repair']
        self.assertEqual(repair['status'], 'error')
        self.assertEqual(repair['error']['type'], 'RuntimeError')
        self.assertIn('VCS_SKETCH_OVER_CONSTRAINTS', repair['error']['message'])


if __name__ == '__main__':
    unittest.main()
