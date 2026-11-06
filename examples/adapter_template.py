"""Implement this factory using your robot and camera SDKs.

The factory returns an object with read_pair() and close(). Synchronize the
robot measurement with the image, and record timestamps in metadata.
"""
class Adapter:
    def execute_segment(self, segment):
        # Send the entire timed waypoint segment using your robot SDK. Block until
        # feedback confirms arrival AND the robot has settled; raise on failure.
        # Return a JSON-serializable receipt (or None), never claim success merely
        # because the command was sent. No read_pair() happens until this returns.
        raise NotImplementedError("Implement recorded trajectory execution")

    def read_pair(self):
        # Return matched, stationary poses as homogeneous 4x4 matrices.
        # T_base_flange: controller feedback end frame -> robot base.
        # T_camera_board: target -> original camera optical frame.
        # Translation is in metres. Do not return a commanded pose.
        raise NotImplementedError("Connect your robot and camera SDK here")

    def close(self):
        pass


def create():
    return Adapter()
