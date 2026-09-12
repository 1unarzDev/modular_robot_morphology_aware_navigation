from setuptools import find_packages, setup

package_name = "morphology_manager"
setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools", "PyYAML"],
    zip_safe=True,
    maintainer="Morphology Navigation Team",
    maintainer_email="maintainer@example.com",
    description="Authoritative morphology manager",
    license="Apache-2.0",
    entry_points={"console_scripts": ["morphology_manager = morphology_manager.node:main"]},
    test_suite="unittest.TestSuite",
)
