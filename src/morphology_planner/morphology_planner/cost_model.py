from __future__ import annotations

from dataclasses import dataclass
import json
from math import log
from typing import Protocol

from .catalog import Morphology, Objective, Transition


@dataclass(frozen=True)
class EdgeEstimate:
    time: float
    energy: float
    failure_probability: float

    def objective_cost(self, objective: Objective) -> float:
        probability = min(1.0 - 1e-9, max(0.0, self.failure_probability))
        return self.time + objective.lambda_energy * self.energy + objective.lambda_risk * -log(1.0 - probability)


class EdgeCostModel(Protocol):
    def traversal(self, morphology: Morphology, distance: float, angle: float) -> EdgeEstimate: ...
    def reconfiguration(self, transition: Transition) -> EdgeEstimate: ...


class AnalyticCostModel:
    def traversal(self, morphology: Morphology, distance: float, angle: float) -> EdgeEstimate:
        duration = distance / morphology.limits.linear + angle / morphology.limits.angular
        energy = distance * (10.0 + 2.5 * morphology.radius) + angle * 2.0
        return EdgeEstimate(duration, energy, 0.0)

    def reconfiguration(self, transition: Transition) -> EdgeEstimate:
        return EdgeEstimate(transition.time, transition.energy, transition.failure_probability)


class OnnxCostModel(AnalyticCostModel):
    """Optional ONNX adapter with a safe analytic fallback.

    Models receive one row containing nominal time, energy, risk, morphology
    radius, distance, and angle, and return corrected time, energy, and risk.
    A missing or invalid runtime never makes planning unavailable.
    """

    def __init__(self, model_path: str) -> None:
        try:
            import onnxruntime as ort
            self.session = ort.InferenceSession(model_path, providers=["CPUExecutionProvider"])
            self.input_name = self.session.get_inputs()[0].name
        except Exception:
            self.session = None

    def _predict(self, estimate: EdgeEstimate, radius: float, distance: float, angle: float) -> EdgeEstimate:
        if self.session is None:
            return estimate
        try:
            import numpy as np
            features = np.asarray([[estimate.time, estimate.energy,
                                    estimate.failure_probability, radius,
                                    distance, angle]], dtype=np.float32)
            output = self.session.run(None, {self.input_name: features})[0][0]
            return EdgeEstimate(max(0.0, float(output[0])), max(0.0, float(output[1])),
                                min(0.999, max(0.0, float(output[2]))))
        except Exception:
            return estimate

    def traversal(self, morphology: Morphology, distance: float, angle: float) -> EdgeEstimate:
        return self._predict(super().traversal(morphology, distance, angle),
                             morphology.radius, distance, angle)

    def reconfiguration(self, transition: Transition) -> EdgeEstimate:
        return self._predict(super().reconfiguration(transition),
                             transition.swept_radius, 0.0, 0.0)


class LinearCalibratedCostModel(AnalyticCostModel):
    """Small, auditable learned model used when ONNX is unnecessary."""

    def __init__(self, model_path: str) -> None:
        with open(model_path, encoding="utf-8") as stream:
            value = json.load(stream)
        if value.get("schema_version") != 1:
            raise ValueError("unsupported learned cost model schema")
        self.weights = value["weights"]

    def _predict(self, estimate: EdgeEstimate, radius: float, distance: float, angle: float) -> EdgeEstimate:
        features = [1.0, estimate.time, estimate.energy,
                    estimate.failure_probability, radius, distance, angle]
        outputs = [sum(weight * feature for weight, feature in zip(row, features))
                   for row in self.weights]
        return EdgeEstimate(max(0.0, outputs[0]), max(0.0, outputs[1]),
                            min(0.999, max(0.0, outputs[2])))

    def traversal(self, morphology: Morphology, distance: float, angle: float) -> EdgeEstimate:
        return self._predict(super().traversal(morphology, distance, angle),
                             morphology.radius, distance, angle)

    def reconfiguration(self, transition: Transition) -> EdgeEstimate:
        return self._predict(super().reconfiguration(transition),
                             transition.swept_radius, 0.0, 0.0)
