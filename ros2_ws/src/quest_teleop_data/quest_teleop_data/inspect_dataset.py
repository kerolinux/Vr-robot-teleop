#!/usr/bin/env python3
"""Print a health report for a recorded dataset:   ros2 run quest_teleop_data inspect_dataset FILE.hdf5"""

import argparse
import sys

from .dataset_io import read_file_attrs, summarize


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("file")
    args = ap.parse_args(argv)

    attrs = read_file_attrs(args.file)
    rows = summarize(args.file)
    print(f"{args.file}\n  schema v{attrs.get('schema_version')}, robot {attrs.get('robot')}, "
          f"{attrs.get('rate_hz')} Hz, created {attrs.get('created')}, {len(rows)} demo(s)\n")
    print(f"{'demo':8} {'samples':>8} {'sec':>7} {'success':>8} {'tracked':>8} {'engaged':>8}  problems")
    bad = 0
    for r in rows:
        bad += bool(r["problems"])
        print(f"{r['name']:8} {r['samples']:8d} {r['duration']:7.1f} {str(r['success']):>8} "
              f"{r['tracked']:8.0%} {r['engaged']:8.0%}  {'; '.join(r['problems']) or 'ok'}")
    ok = sum(r["success"] for r in rows)
    print(f"\n{ok}/{len(rows)} labelled successful, {bad} with problems")
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
