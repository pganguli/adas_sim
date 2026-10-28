"""Helpers shared by the log tools: find and load an adas_sim CSV log."""

import csv
from pathlib import Path
from typing import Any

import numpy as np
import numpy.typing as npt

FloatArray = npt.NDArray[np.floating[Any]]

# LongitudinalMode values (include/types/planning_types.h).
MODE_NAMES = {0: "OFF", 1: "CRUISE", 2: "FOLLOW", 3: "EBRAKE"}


def latest_log() -> str:
    """Newest CSV in out/ (adas_sim names them <scenario>_seed<N>.csv)."""
    logs = sorted(Path("out").glob("*.csv"), key=lambda p: p.stat().st_mtime)
    if not logs:
        raise SystemExit("no log in out/: run `make run` first")
    return str(logs[-1])


def load(path: str) -> dict[str, FloatArray]:
    """Every CSV column as a float array, keyed by its header name."""
    with open(path, newline="") as f:
        rows = list(csv.DictReader(f))
    return {k: np.array([float(r[k]) for r in rows]) for k in rows[0]}
