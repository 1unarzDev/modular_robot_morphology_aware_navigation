from pathlib import Path

from modular_robot_benchmarks.mission_batch import configuration_hash


def test_configuration_hash_covers_files_and_exact_parameters(tmp_path):
    first = tmp_path / "a"; first.write_text("one")
    second = tmp_path / "b"; second.write_text("two")
    baseline = configuration_hash([first, second], {"deadline": 300})
    assert baseline == configuration_hash([second, first], {"deadline": 300})
    assert baseline != configuration_hash([first, second], {"deadline": 301})
    second.write_text("changed")
    assert baseline != configuration_hash([first, second], {"deadline": 300})
