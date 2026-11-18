"""Recompute the hand-eye calibration from an existing dataset (no robot, no camera needed).

Usage (inside the container, from /work):
    python3 compute_from_dataset.py datasets/2026-11-18_16-30-00 --eth
    python3 compute_from_dataset.py datasets/2026-11-18_16-30-00 --eih
"""

import argparse
from pathlib import Path

import zivid

from universal_robots_perform_hand_eye_calibration import _save_hand_eye_results, perform_hand_eye_calibration


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("dataset", type=Path)
    g = parser.add_mutually_exclusive_group(required=True)
    g.add_argument("--eth", "--eye-to-hand", action="store_true")
    g.add_argument("--eih", "--eye-in-hand", action="store_true")
    args = parser.parse_args()

    _app = zivid.Application()  # noqa: F841 (needed to load ZDF files)
    mode = "eye-in-hand" if args.eih else "eye-to-hand"
    transform, residuals = perform_hand_eye_calibration(mode, args.dataset)
    _save_hand_eye_results(args.dataset, transform, residuals)
    print(f"\nSaved results in {args.dataset}")


if __name__ == "__main__":
    main()
