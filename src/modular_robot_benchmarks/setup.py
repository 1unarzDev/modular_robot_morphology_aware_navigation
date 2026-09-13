from setuptools import find_packages, setup

package_name = "modular_robot_benchmarks"
setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools", "PyYAML", "numpy"],
    zip_safe=True,
    maintainer="Morphology Navigation Team",
    maintainer_email="maintainer@example.com",
    description="Morphology navigation benchmarks",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "benchmark_planner = modular_robot_benchmarks.runner:main",
            "generate_benchmark_sdf = modular_robot_benchmarks.sdf_export:main",
            "train_cost_model = modular_robot_benchmarks.train_cost_model:main",
            "morphology_study = modular_robot_benchmarks.study:main",
            "run_morphology_missions = modular_robot_benchmarks.mission_batch:main",
            "morphology_engineering = modular_robot_benchmarks.engineering_qualification:main",
            "qualify_assembly_motion = modular_robot_benchmarks.motion_qualification:main",
            "qualify_detached_pod = modular_robot_benchmarks.detached_pod_qualification:main",
            "qualify_roundtrip_batch = modular_robot_benchmarks.roundtrip_qualification:main",
        ]
    },
    test_suite="unittest.TestSuite",
)
