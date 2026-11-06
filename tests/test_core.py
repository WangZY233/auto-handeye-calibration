import json
from pathlib import Path
import unittest
from unittest.mock import patch
import numpy as np
from handeye.core import collect, solve

class CoreTests(unittest.TestCase):
    def test_synthetic_fit_and_held_out(self):
        data = json.loads((Path(__file__).resolve().parents[1] / "examples/synthetic.json").read_text())
        result = solve(data)
        self.assertEqual(len(result["validation_residuals"]), 4)
        self.assertLess(max(x["translation_mm"] for x in result["validation_residuals"]), 1e-5)
        self.assertLess(max(x["rotation_deg"] for x in result["validation_residuals"]), 1e-5)
        self.assertFalse(result["physical_accuracy_verified"])

    def test_collect_preserves_metadata_and_closes(self):
        class Adapter:
            closed = False
            def read_pair(self):
                return {"T_base_flange": np.eye(4), "T_camera_board": np.eye(4), "metadata": {"stamp": 1}}
            def close(self): self.closed = True
        adapter = Adapter()
        with patch("builtins.input", return_value=""):
            data = collect(adapter, 3)
        self.assertTrue(adapter.closed)
        self.assertEqual(data["samples"][0]["metadata"]["stamp"], 1)

    def test_invalid_pose_rejected(self):
        with self.assertRaises(ValueError):
            solve({"samples": [{"T_base_flange": [[1]], "T_camera_board": np.eye(4).tolist()}] * 3})

if __name__ == "__main__": unittest.main()
