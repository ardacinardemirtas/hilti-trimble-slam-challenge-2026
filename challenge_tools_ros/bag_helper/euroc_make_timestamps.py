#!/usr/bin/env python3
"""
Generate an ORB_SLAM3-style timestamps file from a EuRoC cam0/data.csv.

ORB_SLAM3 stereo_inertial_euroc expects a flat text file with one timestamp per
line (in nanoseconds as integers), matching the PNG filenames in cam0/data/.

Usage:
    python3 euroc_make_timestamps.py <euroc_dir> [output_timestamps.txt]

If the output path is omitted, it is written to <euroc_dir>/mav0/cam0/timestamps.txt.
"""

import sys
from pathlib import Path


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)

    euroc_dir = Path(sys.argv[1])
    cam0_csv = euroc_dir / "mav0" / "cam0" / "data.csv"

    if not cam0_csv.exists():
        sys.exit(f"ERROR: {cam0_csv} not found. Is this a valid EuRoC directory?")

    out_path = Path(sys.argv[2]) if len(sys.argv) >= 3 else (euroc_dir / "mav0" / "cam0" / "timestamps.txt")
    out_path.parent.mkdir(parents=True, exist_ok=True)

    timestamps = []
    with open(cam0_csv) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            # EuRoC data.csv format: timestamp_ns,filename
            ts_ns = line.split(",")[0]
            timestamps.append(ts_ns)

    with open(out_path, "w") as f:
        for ts in timestamps:
            f.write(ts + "\n")

    print(f"Wrote {len(timestamps)} timestamps → {out_path}")


if __name__ == "__main__":
    main()
