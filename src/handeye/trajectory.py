"""Replay recorded segments through a user-supplied robot/camera adapter."""
from copy import deepcopy
from .transforms import validate_transform


def validate_trajectory(trajectory):
    """Validate the portable envelope; adapters interpret waypoint payloads."""
    if not isinstance(trajectory, dict) or trajectory.get("version") != 1:
        raise ValueError("trajectory version must be 1")
    segments = trajectory.get("segments")
    if not isinstance(segments, list) or not segments:
        raise ValueError("trajectory must contain at least one segment")
    for index, segment in enumerate(segments):
        if not isinstance(segment, dict) or not isinstance(segment.get("waypoints"), list) or not segment["waypoints"]:
            raise ValueError(f"segment {index} must contain waypoints")
        if not isinstance(segment.get("capture", False), bool):
            raise ValueError(f"segment {index} capture must be boolean")
        if segment.get("role", "train") not in ("train", "validation"):
            raise ValueError(f"segment {index} role must be train or validation")
    return trajectory


def replay(adapter, trajectory, save):
    """Execute complete segments, acquire after actual arrival, checkpoint progress.

    execute_segment must block until the robot has physically arrived and settled,
    and raise on failure. read_pair returns feedback and image from the same time.
    `save` persists every checkpoint, including partial results after an error.
    """
    result = {"convention": "T_A_B maps coordinates from B to A; metres",
              "trajectory": deepcopy(trajectory), "samples": [],
              "completed_segments": 0, "status": "running"}
    try:
        validate_trajectory(trajectory)
        save(result)
        for index, segment in enumerate(trajectory["segments"]):
            # The whole recorded segment is passed through, preserving timing and
            # avoiding a stop at every recorded waypoint.
            receipt = adapter.execute_segment(deepcopy(segment))
            result["completed_segments"] = index + 1
            if segment.get("capture", False):
                pair = adapter.read_pair()
                sample = {"T_base_flange": validate_transform(pair["T_base_flange"]).tolist(),
                          "T_camera_board": validate_transform(pair["T_camera_board"]).tolist(),
                          "role": segment.get("role", "train"),
                          "segment_index": index,
                          "metadata": pair.get("metadata", {}),
                          "motion_receipt": receipt}
                result["samples"].append(sample)
            save(result)
        result["status"] = "complete"
        save(result)
        return result
    except BaseException as exc:
        result["status"] = "failed"
        result["error"] = {"type": type(exc).__name__, "message": str(exc)}
        save(result)
        raise
    finally:
        adapter.close()
