"""Replay timestamped firmware-boundary JSONL; optionally compare recorded outputs."""
import argparse
import json
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from balance_lab.firmware.recording import replay

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("recording", type=Path)
    parser.add_argument("--no-compare", action="store_true", help="run inputs from another firmware/log without asserting equality")
    args = parser.parse_args()
    print(json.dumps(replay(args.recording, compare=not args.no_compare)))
