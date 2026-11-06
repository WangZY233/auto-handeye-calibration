"""Offline demo only: prerecorded synthetic observations, no robot commands."""
import json
from pathlib import Path

class Adapter:
    def __init__(self):
        self.samples = json.loads((Path(__file__).parent / "synthetic.json").read_text())["samples"]
        self.index = None

    def execute_segment(self, segment):
        self.index = segment["example_sample_index"]
        return {"mode": "simulation", "arrived": True}

    def read_pair(self):
        return self.samples[self.index]

    def close(self):
        pass

def create():
    return Adapter()
