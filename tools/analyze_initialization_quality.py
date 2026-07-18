#!/usr/bin/env python3
"""Extract accepted VINS initializations and their early health signals.

The parser intentionally consumes the persistent text logs produced by the
clean-repeat runner.  It does not infer ground truth quality: an accepted
initialization is labelled ``reset`` only when the estimator is reset later in
the same log, and ``survived`` otherwise.  Run outcome and recovery count are
included as separate context so threshold and corruption failures are not
silently combined.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import re
import sys
from pathlib import Path
from typing import Any, Iterable


FLOAT = r"[-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?"


def _fields(line: str) -> dict[str, float]:
    return {
        key: float(value)
        for key, value in re.findall(rf"([a-zA-Z_]+)=({FLOAT})", line)
    }


def _vector_after(line: str, marker: str, count: int = 3) -> list[float] | None:
    if marker not in line:
        return None
    values = re.findall(FLOAT, line.split(marker, 1)[1])
    if len(values) < count:
        return None
    return [float(value) for value in values[:count]]


def _norm(values: list[float] | None) -> float | None:
    if values is None:
        return None
    return math.sqrt(sum(value * value for value in values))


def _load_metrics(run_dir: Path) -> dict[str, Any]:
    path = run_dir / "metrics.json"
    if not path.exists():
        return {}
    with path.open(encoding="utf-8") as stream:
        return json.load(stream)


def parse_run(run_dir: Path) -> list[dict[str, Any]]:
    log_path = run_dir / "vio_vins.log"
    if not log_path.exists():
        raise FileNotFoundError(log_path)

    metrics = _load_metrics(run_dir)
    pending: dict[str, Any] = {}
    records: list[dict[str, Any]] = []
    active: dict[str, Any] | None = None

    with log_path.open(encoding="utf-8", errors="replace") as stream:
        for line_number, line in enumerate(stream, 1):
            if "SFM geometry diagnostics:" in line:
                pending.update({f"sfm_{k}": v for k, v in _fields(line).items()})
            elif "SFM BA summary:" in line:
                pending.update({f"ba_{k}": v for k, v in _fields(line).items()})
                pending["ba_converged"] = "Termination: CONVERGENCE" in line
            elif "Linear alignment diagnostics:" in line:
                pending.update(
                    {f"alignment_{k}": v for k, v in _fields(line).items()}
                )
            elif "Linear alignment raw solution:" in line:
                values = _fields(line)
                pending["alignment_raw_scale"] = values.get("scale")
            elif "Visual/IMU alignment candidate:" in line:
                values = _fields(line)
                pending["alignment_refined_scale"] = values.get("scale")
            elif "Initialization state:" in line:
                active = dict(pending)
                active.update(
                    {
                        "run_id": run_dir.name,
                        "log_path": str(log_path),
                        "accept_line": line_number,
                        "outcome": metrics.get("outcome"),
                        "recovery_count": metrics.get(
                            "vins_recovery_count", metrics.get("recovery_count")
                        ),
                    }
                )
                values = _fields(line)
                active["features"] = values.get("features")
                active["initial_accel_bias_norm"] = _norm(
                    _vector_after(line, "accel_bias=")
                )
                active["initial_gyro_bias_norm"] = _norm(
                    _vector_after(line, "gyro_bias=")
                )
                active["early_update_count"] = 0
                active["early_max_position_delta"] = 0.0
                active["early_max_accel_bias_norm"] = active[
                    "initial_accel_bias_norm"
                ]
            elif active is not None and "Early nonlinear update:" in line:
                values = _fields(line)
                active["early_update_count"] += 1
                active["early_max_position_delta"] = max(
                    active["early_max_position_delta"],
                    values.get("position_delta", 0.0),
                )
                bias_norm = _norm(_vector_after(line, "accel_bias_after="))
                if bias_norm is not None:
                    active["early_max_accel_bias_norm"] = max(
                        active["early_max_accel_bias_norm"] or 0.0, bias_norm
                    )
            elif "reset state successfully" in line and active is not None:
                active["health_label"] = "reset"
                active["reset_line"] = line_number
                records.append(active)
                active = None
                pending = {}

    if active is not None:
        active["health_label"] = "survived"
        active["reset_line"] = None
        records.append(active)

    for index, record in enumerate(records, 1):
        record["initialization_index"] = index
    return records


def iter_run_dirs(inputs: Iterable[Path]) -> Iterable[Path]:
    seen: set[Path] = set()
    for input_path in inputs:
        candidates = (
            [input_path]
            if (input_path / "vio_vins.log").exists()
            else sorted(input_path.glob("*_run*"))
        )
        for candidate in candidates:
            resolved = candidate.resolve()
            if resolved not in seen and (candidate / "vio_vins.log").exists():
                seen.add(resolved)
                yield candidate


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("inputs", nargs="+", type=Path)
    parser.add_argument("--format", choices=("json", "tsv"), default="json")
    args = parser.parse_args()

    records: list[dict[str, Any]] = []
    for run_dir in iter_run_dirs(args.inputs):
        records.extend(parse_run(run_dir))

    if args.format == "json":
        json.dump(records, sys.stdout, indent=2, sort_keys=True)
        sys.stdout.write("\n")
    elif records:
        columns = sorted({key for record in records for key in record})
        writer = csv.DictWriter(sys.stdout, fieldnames=columns, dialect="excel-tab")
        writer.writeheader()
        writer.writerows(records)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
