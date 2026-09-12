from pathlib import Path

from morphology_planner import load_catalog


CATALOG = Path(__file__).parents[1] / "src/modular_robot_description/config/morphologies.yaml"


def test_catalog_has_fixed_inventory_and_six_morphologies():
    catalog = load_catalog(CATALOG)
    assert set(catalog.morphologies) == {
        "compact_diff", "ackermann", "wide_swerve", "narrow_tandem",
        "long_crawler", "articulated",
    }
    assert len(catalog.transitions) == 10
    assert all(catalog.morphologies[t.source] and catalog.morphologies[t.target]
               for t in catalog.transitions)


def test_every_transition_has_a_distinct_id_and_moved_pod():
    catalog = load_catalog(CATALOG)
    assert len({t.id for t in catalog.transitions}) == len(catalog.transitions)
    assert all(t.moved_pods for t in catalog.transitions)


def test_confirmatory_subset_contains_only_physically_supported_modes():
    subset = load_catalog(CATALOG).supported_experiment_subset()
    assert set(subset.morphologies) == {"compact_diff", "narrow_tandem"}
    assert {transition.id for transition in subset.transitions} == {
        "compact_to_narrow", "narrow_to_compact",
    }
