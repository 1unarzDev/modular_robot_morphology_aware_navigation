from modular_robot_benchmarks.scenarios import FAMILIES, make_scenario


def test_all_scenario_families_are_deterministic():
    for family in FAMILIES:
        first = make_scenario(family, 17)
        second = make_scenario(family, 17)
        assert first.grid.data == second.grid.data
        assert first.start == second.start
        assert first.goal == second.goal


def test_scenario_seeds_change_randomized_geometry():
    assert make_scenario("route_choice", 1).grid.data != make_scenario("route_choice", 5).grid.data

