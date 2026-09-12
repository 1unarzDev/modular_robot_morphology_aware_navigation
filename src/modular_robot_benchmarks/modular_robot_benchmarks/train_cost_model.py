from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path


FEATURES = ("nominal_time", "nominal_energy", "nominal_failure_probability",
            "radius", "distance", "angle")
TARGETS = ("observed_time", "observed_energy", "failed")


def fit(input_path: Path, output_path: Path, ridge: float = 1e-6) -> None:
    with input_path.open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    if len(rows) < len(FEATURES) + 1:
        raise ValueError(f"need at least {len(FEATURES) + 1} observations")
    matrix = [[1.0] + [float(row[name]) for name in FEATURES] for row in rows]
    weights = []
    for target in TARGETS:
        labels = [float(row[target]) for row in rows]
        weights.append(_ridge_regression(matrix, labels, ridge))
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(json.dumps({
        "schema_version": 1,
        "features": ["bias", *FEATURES],
        "targets": list(TARGETS),
        "weights": weights,
        "observation_count": len(rows),
    }, indent=2) + "\n", encoding="utf-8")


def _ridge_regression(matrix, labels, ridge):
    columns = len(matrix[0])
    normal = [[sum(row[i] * row[j] for row in matrix) for j in range(columns)]
              for i in range(columns)]
    for i in range(1, columns):
        normal[i][i] += ridge
    rhs = [sum(row[i] * label for row, label in zip(matrix, labels))
           for i in range(columns)]
    return _solve(normal, rhs)


def _solve(matrix, vector):
    size = len(vector)
    augmented = [row[:] + [value] for row, value in zip(matrix, vector)]
    for pivot in range(size):
        best = max(range(pivot, size), key=lambda row: abs(augmented[row][pivot]))
        augmented[pivot], augmented[best] = augmented[best], augmented[pivot]
        if abs(augmented[pivot][pivot]) < 1e-12:
            raise ValueError("training data is rank deficient")
        scale = augmented[pivot][pivot]
        augmented[pivot] = [value / scale for value in augmented[pivot]]
        for row in range(size):
            if row == pivot:
                continue
            factor = augmented[row][pivot]
            augmented[row] = [current - factor * base
                              for current, base in zip(augmented[row], augmented[pivot])]
    return [augmented[row][-1] for row in range(size)]


def main() -> None:
    parser = argparse.ArgumentParser(description="Fit calibrated traversal and docking costs")
    parser.add_argument("observations", type=Path)
    parser.add_argument("--output", type=Path, default=Path("models/edge_costs.json"))
    parser.add_argument("--ridge", type=float, default=1e-6)
    args = parser.parse_args()
    fit(args.observations, args.output, args.ridge)


if __name__ == "__main__":
    main()

