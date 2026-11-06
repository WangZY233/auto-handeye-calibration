import json
from pathlib import Path
import unittest
from handeye.trajectory import replay, validate_trajectory
from handeye.core import solve

ROOT = Path(__file__).resolve().parents[1]

class TrajectoryTests(unittest.TestCase):
    def test_full_replay_and_solve(self):
        import sys
        sys.path.insert(0, str(ROOT / 'examples'))
        from simulated_adapter import Adapter
        trajectory = json.loads((ROOT / 'examples/simulated_trajectory.json').read_text())
        checkpoints = []
        result = replay(Adapter(), trajectory, lambda state: checkpoints.append((state['status'], len(state['samples']))))
        self.assertEqual(result['status'], 'complete')
        self.assertEqual(result['completed_segments'], 16)
        self.assertEqual(len(result['samples']), 16)
        self.assertEqual(checkpoints[-1], ('complete', 16))
        fit = solve(result)
        self.assertEqual(len(fit['validation_residuals']), 4)
        self.assertLess(max(r['translation_mm'] for r in fit['validation_residuals']), 1e-5)

    def test_whole_segment_then_feedback_and_failure_checkpoint(self):
        class Adapter:
            def __init__(self): self.calls=[]; self.closed=False
            def execute_segment(self, segment):
                self.calls.append(('move', len(segment['waypoints'])))
                if len(self.calls) == 3: raise RuntimeError('arrival failed')
            def read_pair(self):
                self.calls.append(('read',))
                return {'T_base_flange': [[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]],
                        'T_camera_board': [[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]]}
            def close(self): self.closed=True
        adapter=Adapter(); snapshots=[]
        import copy
        traj={'version':1,'segments':[{'waypoints':[1,2,3], 'capture':True}, {'waypoints':[4], 'capture':True}]}
        with self.assertRaisesRegex(RuntimeError, 'arrival failed'):
            replay(adapter, traj, lambda r:snapshots.append(copy.deepcopy(r)))
        self.assertEqual(adapter.calls, [('move',3),('read',),('move',1)])
        self.assertTrue(adapter.closed)
        self.assertEqual(snapshots[-1]['status'], 'failed')
        self.assertEqual(snapshots[-1]['completed_segments'], 1)
        self.assertEqual(len(snapshots[-1]['samples']), 1)

    def test_invalid_trajectory(self):
        for value in ({'version': 1, 'segments': []}, {'version': 1, 'segments':[{'waypoints':[]}]}):
            with self.assertRaises(ValueError): validate_trajectory(value)
