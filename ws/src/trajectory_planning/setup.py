from setuptools import setup, find_packages
from glob import glob

package_name = "fsd_trajectory_planning"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", glob("launch/*.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Wolfi",
    maintainer_email="w.bradfisch@gmx.de",
    description="Trajectory planning with rann Sack",
    license="MIT",
    entry_points={
        "console_scripts": [
            # Matches launch executable name
            "trajectory_planning_node = trajectory_planning.trajectory_node:main",
        ],
    },
)
