"""Morphology-augmented navigation planner."""

from .catalog import (
    Catalog, Morphology, PostTransitionVerification, Transition,
    VerificationStage, load_catalog,
)
from .grid import OccupancyGrid
from .planner import HybridPlan, HybridSegment, HybridState, MorphologyAStar
from .route_first import RouteFirstAdaptationPlanner
from .methods import MethodPlanner, make_method_planner, plan_signature
from .transition_policy import (
    CoupledTransitionPolicy, PodSensingState, TransitionDecision,
    build_transition_trajectories, build_verification_trajectories,
)

__all__ = [
    "Catalog", "Morphology", "PostTransitionVerification", "Transition",
    "VerificationStage", "load_catalog", "OccupancyGrid",
    "HybridPlan", "HybridSegment", "HybridState", "MorphologyAStar",
    "RouteFirstAdaptationPlanner",
    "MethodPlanner", "make_method_planner", "plan_signature",
    "CoupledTransitionPolicy", "PodSensingState", "TransitionDecision",
    "build_transition_trajectories", "build_verification_trajectories",
]
