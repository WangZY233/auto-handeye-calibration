import argparse
import json
import os
from pathlib import Path
import tempfile
from .core import collect, load_adapter, solve
from .trajectory import replay, validate_trajectory


def _write_new(path, value):
    with Path(path).open("x", encoding="utf-8") as f:
        json.dump(value, f, indent=2, ensure_ascii=False, allow_nan=False)
        f.write("\n")


def _checkpoint(path, value):
    path = Path(path)
    name = None
    try:
        with tempfile.NamedTemporaryFile(mode="w", encoding="utf-8", dir=path.parent,
                                         prefix="." + path.name, delete=False) as f:
            name = f.name
            json.dump(value, f, indent=2, ensure_ascii=False, allow_nan=False)
            f.write("\n")
            f.flush()
            os.fsync(f.fileno())
        os.replace(name, path)
    finally:
        if name and os.path.exists(name):
            os.unlink(name)


def main():
    parser = argparse.ArgumentParser(description="Eye-to-hand calibration, stationary camera")
    commands = parser.add_subparsers(dest="command", required=True)
    capture = commands.add_parser("collect", help="manually collect paired observations")
    capture.add_argument("--adapter", required=True, help="Python factory module:attribute")
    capture.add_argument("--count", type=int, required=True)
    capture.add_argument("--output", required=True)
    run = commands.add_parser("replay", help="execute recorded segments and collect pose pairs")
    run.add_argument("trajectory", help="trajectory JSON")
    run.add_argument("--adapter", required=True, help="Python factory module:attribute")
    run.add_argument("--output", required=True, help="new file for progressive capture results")
    fit = commands.add_parser("solve", help="solve from paired poses")
    fit.add_argument("input")
    fit.add_argument("--output", required=True)
    args = parser.parse_args()
    if args.command == "collect":
        if args.count < 3:
            parser.error("count must be at least 3")
        _write_new(args.output, collect(load_adapter(args.adapter), args.count))
    elif args.command == "replay":
        trajectory = validate_trajectory(json.loads(Path(args.trajectory).read_text(encoding="utf-8")))
        # Claim output before constructing the adapter or commanding motion.
        _write_new(args.output, {"status": "initializing", "samples": []})
        try:
            adapter = load_adapter(args.adapter)
        except BaseException as exc:
            _checkpoint(args.output, {"status": "failed", "samples": [], "error": str(exc)})
            raise
        replay(adapter, trajectory, lambda data: _checkpoint(args.output, data))
    else:
        _write_new(args.output, solve(json.loads(Path(args.input).read_text(encoding="utf-8"))))
    print(f"Saved {args.output}")

if __name__ == "__main__":
    main()
