"""Eye-to-hand pose-pair acquisition and calibration."""
from importlib import import_module
import numpy as np
from .solver import solve_eye_to_hand
from .transforms import validate_transform, pose_error


def load_adapter(spec):
    """Load a no-argument adapter factory given as module:attribute."""
    module, name = spec.split(":", 1)
    return getattr(import_module(module), name)()


def collect(adapter, count):
    """Collect stationary paired observations; adapter controls synchronization.

    The caller positions the arm. This function sends no motion commands.
    """
    samples = []
    try:
        for i in range(count):
            input(f"Move to pose {i + 1}/{count}, settle, then press Enter: ")
            pair = adapter.read_pair()
            samples.append({"T_base_flange": validate_transform(pair["T_base_flange"]).tolist(),
                            "T_camera_board": validate_transform(pair["T_camera_board"]).tolist(),
                            "metadata": pair.get("metadata", {})})
    finally:
        adapter.close()
    return {"convention": "T_A_B maps coordinates from B to A; metres", "samples": samples}


def solve(data):
    samples = data["samples"]
    training = [s for s in samples if s.get("role", "train") == "train"]
    validation = [s for s in samples if s.get("role", "train") == "validation"]
    if any(s.get("role", "train") not in ("train", "validation") for s in samples):
        raise ValueError("role must be train or validation")
    result = solve_eye_to_hand([s["T_base_flange"] for s in training],
                               [s["T_camera_board"] for s in training])
    x, mount = np.asarray(result["T_base_camera"]), np.asarray(result["T_flange_board"])
    result["validation_residuals"] = [pose_error(validate_transform(s["T_base_flange"]) @ mount,
                                                   x @ validate_transform(s["T_camera_board"]))
                                      for s in validation]
    result["physical_accuracy_verified"] = False
    return result
