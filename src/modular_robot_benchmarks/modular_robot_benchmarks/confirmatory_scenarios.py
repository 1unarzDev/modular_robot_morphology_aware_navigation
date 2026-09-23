from __future__ import annotations

from dataclasses import asdict, dataclass
from random import Random

from morphology_planner import FiducialStation, HybridState, PodSensingState
from morphology_planner.catalog import Transition
from morphology_planner.grid import OCCUPIED, OccupancyGrid
from morphology_planner.transition_validation import Box3

from .design import CONFIRMATORY_FAMILIES, WORKSHOP_VARIANTS

# Workshop transition post (staging-site frame, robot facing the door). The
# compact_to_narrow relocation sweeps pod_4 to |y| <= 0.423 m near x = +0.30 m
# and pod_5 symmetrically near x = -0.30 m, while the assembled robot never
# exceeds |y| = 0.2875 m. A post centered at |y| = 0.41 m therefore intersects
# one pod's relocation path but stays ~6 cm clear of the driving robot. It sits
# below the lidar plane and is absent from the occupancy map.
WORKSHOP_POST_SIZE = (0.12, 0.12, 0.12)
WORKSHOP_POST_OFFSETS = {
    "workshop_blocked_a": (0.30, 0.41),
    "workshop_blocked_b": (-0.30, -0.41),
}
# Geometry-coupled staging requires the 1.0 m transition clearance disk to
# clear the wall, so the attractive site is 1.0 m before the wall centerline.
WORKSHOP_SITE_SETBACK_M = 1.0

# Compact collision footprint half-length including wheel envelopes (catalog).
COMPACT_HALF_LENGTH_M = 0.40
# Covers the declared +/-0.015 m spawn offset and +/-0.035 rad spawn yaw with margin.
SPAWN_SHELF_CLEARANCE_M = 0.10
# The shelf must obstruct relocation, not driving. On the pod track (+0.20 m)
# it also lay on the door approach, where the narrow robot's left pods run, so
# no route to the goal existed once it was treated as a driving obstacle.
# Following the workshop post, it sits in the band only the relocation sweep
# reaches: inner edge 0.365 m clears the compact 0.36 m safety footprint (and
# the physical robot's 0.2875 m) while overlapping pod_4's sweep to 0.423 m.
SHELF_LATERAL_OFFSET_M = 0.415
SHELF_DEPTH_M = 0.10
# A shelf reaching back to the spawn covers every door-line site, so with it
# also blocking driving no feasible site remains (the narrow robot has +/-0.02 m
# of lateral slack in the doorway and must transition on the door line). It
# ends where it did, at the staging band's door side, over a fixed length that
# covers the direct site's pod_4 excursion (x +0.15..+0.45 m) and leaves an
# earlier door-line site whose sweep clears it. See ADR 0004.
SHELF_LENGTH_M = 0.60
# combined_constraints occlusion around the feasibility-aware site (ADR 0004),
# widened under ADR 0006 from 0.60 m to 2.10 m about the same centre. A site is
# rejected when any moved pod is shadowed, and the pods sit at only four
# discrete x offsets, so a shadow narrower than the 0.70 m gap between -0.35 and
# +0.35 m produces disjoint slivers a site can sit between. A contiguous
# rejected band therefore needs about 2.12 m, and the declared 0.60 m band was
# not physically realizable at all. Screened at 9/9 separating, 0 unplanned;
# 1.60 m gives 0/9, exactly as the discrete-offset argument predicts.
OCCLUSION_LEAD_M = 1.25
OCCLUSION_TRAIL_M = 0.85
OCCLUSION_HALF_HEIGHT_M = 0.25

# ADR 0006. An observability region states what the design intends; the
# fiducial station and the elevated screen are the physical cause that produces
# it. The screen hangs between a station on the south wall and the staging
# corridor: a sight line falling from STATION_HEIGHT_M to a pod at ground level
# stays above SCREEN_UNDERSIDE_M for the first ~0.67 of its length, so a screen
# at SCREEN_FRACTION of the way lies inside that span and shadows whatever the
# ray reaches beyond it. Sites nearer the station -- lower in y -- keep a clear
# line and stay observable, which leaves a distinct feasible site to choose.
#
# The shadow is the declared region ERODED by the pods' reach, because the
# rejected set is the shadow dilated by that reach and never equal to it.
#
# The screen clears the tallest morphology (0.30 m), so by the rule at
# `planner.py:88` it constrains no driving; it is z-disjoint from the
# relocation sweeps, so it changes no transition feasibility; it is above the
# lidar plane (0.12 m), so it never enters `/scan`, the occupancy map, or the
# static map AMCL localizes against; and `box_clearance` is 3D, so it stays
# about 0.4 m clear of the pods and cannot register as a contact. It is an
# optical obstacle only, and the Gate 2 report confirms it by leaving every
# geometry and feasibility site where it was before ADR 0006.
#
# What the physical model reproduces is the region's x extent. Its y extent is
# a consequence of the station and screen geometry rather than of the declared
# bounds, so `observability_regions` remains the statement of intent in y.
STATION_WALL_OFFSET_M = 0.15
STATION_HEIGHT_M = 1.80
SCREEN_UNDERSIDE_M = 0.60
SCREEN_TOP_M = 2.00
SCREEN_DEPTH_M = 0.10
SCREEN_FRACTION = 0.50
TRANSITION_POD_REACH_M = 0.71


@dataclass(frozen=True)
class ObservabilityRegion:
    min_x: float
    max_x: float
    min_y: float
    max_y: float
    connector_visible: bool
    covariance_trace: float

    def contains(self, x: float, y: float) -> bool:
        return self.min_x <= x <= self.max_x and self.min_y <= y <= self.max_y


@dataclass(frozen=True)
class ConfirmatoryScenario:
    family: str
    layout_index: int
    world_seed: int
    neutral_control: bool
    grid: OccupancyGrid
    start: tuple[int, int]
    goal: tuple[int, int]
    transition_obstacles: tuple[Box3, ...]
    observability_regions: tuple[ObservabilityRegion, ...]
    parameters: dict
    # ADR 0006. `observability_regions` states the design's intent; the station
    # and the elevated screen in `transition_obstacles` are the physical cause
    # that has to reproduce it. A layout declaring no station is not
    # station-gated and keeps the behaviour every world had before ADR 0006.
    fiducial_stations: tuple[FiducialStation, ...] = ()

    @property
    def layout_id(self) -> str:
        return f"{self.family}-{self.layout_index:02d}"

    def sensing_for(self, transition: Transition, state: HybridState):
        wx, wy = self.grid.cell_center(state.x, state.y)
        visible, covariance = True, 0.003
        for region in self.observability_regions:
            if region.contains(wx, wy):
                visible = region.connector_visible
                covariance = region.covariance_trace
                break
        return {
            pod: PodSensingState(visible, covariance)
            for pod in transition.moved_pods
        }

    def manifest(self) -> dict:
        return {
            "schema_version": 1,
            "family": self.family,
            "layout_id": self.layout_id,
            "world_seed": self.world_seed,
            "neutral_control": self.neutral_control,
            "start": list(self.start),
            "goal": list(self.goal),
            "parameters": self.parameters,
            "transition_obstacles": [asdict(box) for box in self.transition_obstacles],
            "observability_regions": [asdict(region) for region in self.observability_regions],
            "fiducial_stations": [asdict(station) for station in self.fiducial_stations],
        }


def make_confirmatory_scenario(
    family: str,
    layout_index: int,
    world_seed: int,
    resolution: float = 0.1,
) -> ConfirmatoryScenario:
    if family in WORKSHOP_VARIANTS:
        return make_workshop_scenario(family, world_seed, resolution)
    if family not in CONFIRMATORY_FAMILIES:
        raise ValueError(f"unknown confirmatory family: {family}")
    if not 0 <= layout_index < 12:
        raise ValueError("confirmatory layout index must be in [0, 12)")
    rng = Random(world_seed)
    grid = OccupancyGrid(round(6.0 / resolution), round(3.5 / resolution), resolution)
    _border(grid)
    wall_x = round(3.0 / resolution) + rng.randrange(-1, 2)
    door_center = round(1.75 / resolution) + rng.randrange(-1, 2)
    # Use an odd cell count so the opening is centered on the route cell. The
    # compact 0.70 m safety footprint is rejected at wall contact, while the
    # narrow 0.52 m safety footprint retains 0.09 m clearance per side.
    door_cells = max(7, round(0.7 / resolution))
    door_low = door_center - door_cells // 2
    for y in range(1, grid.height - 1):
        if not door_low <= y < door_low + door_cells:
            grid.set_value(wall_x, y, OCCUPIED)
    start = (round(0.6 / resolution), door_center)
    goal = (round(5.0 / resolution), door_center)
    neutral = layout_index % 4 == 0

    # The direct staging band is broad enough to admit the 2D clearance disk.
    band_x0 = 0.0
    band_x1 = wall_x * resolution - 0.8 + 0.05 * rng.randrange(3)
    center_y = (door_center + 0.5) * resolution
    band_half_height = 0.45 + 0.05 * rng.randrange(3)
    # The shelf is below pod-body height, so it still begins beyond the compact
    # robot's spawn footprint in x.
    start_x_m = (start[0] + 0.5) * resolution
    shelf_x0 = max(start_x_m + COMPACT_HALF_LENGTH_M + SPAWN_SHELF_CLEARANCE_M,
                   band_x1 - SHELF_LENGTH_M)
    obstacle = Box3(
        "raised_transition_shelf",
        ((shelf_x0 + band_x1) / 2.0, center_y + SHELF_LATERAL_OFFSET_M, 0.14),
        (band_x1 - shelf_x0, SHELF_DEPTH_M, 0.08),
    )
    poor_region = ObservabilityRegion(
        band_x0, band_x1,
        center_y - band_half_height, center_y + band_half_height,
        False, 0.04,
    )

    obstacles: tuple[Box3, ...] = ()
    regions: tuple[ObservabilityRegion, ...] = ()
    if not neutral:
        if family == "reconfiguration_workspace":
            obstacles = (obstacle,)
        elif family == "docking_observability":
            regions = (poor_region,)
        else:
            obstacles = (obstacle,)
            regions = (ObservabilityRegion(
                # Occlude the staging lane where the feasibility-aware site
                # falls once the shelf blocks the direct site: the door-line
                # stretch just before the shelf. A distinct visible feasible
                # site must remain, which the manipulation check verifies.
                shelf_x0 - OCCLUSION_LEAD_M, shelf_x0 + OCCLUSION_TRAIL_M,
                center_y - OCCLUSION_HALF_HEIGHT_M, center_y + OCCLUSION_HALF_HEIGHT_M,
                False, 0.04,
            ),)

    parameters = {
        "resolution": resolution,
        "width": grid.width,
        "height": grid.height,
        "wall_x_cell": wall_x,
        "door_center_cell": door_center,
        "door_cells": door_cells,
        "direct_band_x": [band_x0, band_x1],
        "direct_band_half_height": band_half_height,
    }
    # Every confirmatory layout carries a station: without one the sensing
    # method has no observation source at all and fails closed everywhere,
    # including in the neutral controls it must agree in. Only a layout that
    # declares a region gets the screen that shadows it.
    #
    # The station sits abeam the region it has to shadow, which for
    # `combined_constraints` is offset from the staging band's centre. Anchoring
    # it on the band instead skews the projection and costs that family one
    # separating layout.
    station = _station_for(regions[0] if regions else poor_region)
    if regions:
        obstacles = (*obstacles, _screen_for(station, regions[0], center_y))
    return ConfirmatoryScenario(
        family, layout_index, world_seed, neutral, grid, start, goal,
        obstacles, regions, parameters, (station,),
    )


def make_workshop_scenario(
    variant: str, world_seed: int, resolution: float = 0.1,
) -> ConfirmatoryScenario:
    """One passage environment; variants differ only in the transition post."""
    if variant not in WORKSHOP_VARIANTS:
        raise ValueError(f"unknown workshop variant: {variant}")
    # Layout 0 is a neutral control: walls and doorway only.
    base = make_confirmatory_scenario("reconfiguration_workspace", 0, world_seed, resolution)
    site_x = (base.parameters["wall_x_cell"] + 0.5) * resolution - WORKSHOP_SITE_SETBACK_M
    site_y = (base.parameters["door_center_cell"] + 0.5) * resolution
    obstacles: tuple[Box3, ...] = ()
    if variant in WORKSHOP_POST_OFFSETS:
        dx, dy = WORKSHOP_POST_OFFSETS[variant]
        obstacles = (Box3(
            "transition_post",
            (site_x + dx, site_y + dy, WORKSHOP_POST_SIZE[2] / 2.0),
            WORKSHOP_POST_SIZE,
        ),)
    parameters = {**base.parameters, "workshop_variant": variant,
                  "attractive_site_xy": [site_x, site_y]}
    return ConfirmatoryScenario(
        variant, 0, world_seed, variant == "workshop_neutral", base.grid,
        base.start, base.goal, obstacles, (), parameters,
    )


def _station_for(region: ObservabilityRegion) -> FiducialStation:
    """The workspace station, on the south wall abeam the region it shadows.

    A layout with no declared region passes the staging band, which only fixes
    where its station stands; with no screen every site stays observable.
    """
    return FiducialStation(
        "staging_station",
        ((region.min_x + region.max_x) / 2.0, STATION_WALL_OFFSET_M,
         STATION_HEIGHT_M),
    )


def _screen_for(
    station: FiducialStation, region: ObservabilityRegion, center_y: float,
) -> Box3 | None:
    """The elevated screen whose shadow produces `region`, or None if the
    region is narrower than the pods' reach can realize."""
    low = region.min_x + TRANSITION_POD_REACH_M
    high = region.max_x - TRANSITION_POD_REACH_M
    if high <= low:
        raise ValueError(
            f"an observability region {region.max_x - region.min_x:.2f} m wide "
            f"cannot be realized: the pods' reach is "
            f"{TRANSITION_POD_REACH_M:.2f} m (ADR 0006)")
    x_station, y_station = station.position[0], station.position[1]
    y_screen = y_station + SCREEN_FRACTION * (center_y - y_station)
    near = x_station + SCREEN_FRACTION * (low - x_station)
    far = x_station + SCREEN_FRACTION * (high - x_station)
    return Box3(
        "elevated_sight_screen",
        ((near + far) / 2.0, y_screen, (SCREEN_UNDERSIDE_M + SCREEN_TOP_M) / 2.0),
        (far - near, SCREEN_DEPTH_M, SCREEN_TOP_M - SCREEN_UNDERSIDE_M),
    )


def _border(grid: OccupancyGrid) -> None:
    for x in range(grid.width):
        grid.set_value(x, 0, OCCUPIED)
        grid.set_value(x, grid.height - 1, OCCUPIED)
    for y in range(grid.height):
        grid.set_value(0, y, OCCUPIED)
        grid.set_value(grid.width - 1, y, OCCUPIED)
