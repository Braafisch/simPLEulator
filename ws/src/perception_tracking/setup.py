from setuptools import setup, find_packages
from glob import glob

package_name = "fsd_perception_tracking"

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
    description="Nearest-neighbor based cone tracking node",
    license="MIT",
    entry_points={
        "console_scripts": [
            # ros2 run perception_tracking tracking
            "tracking = perception_tracking.perception_node:main",
            # alternative, more explicit name
            "perception_tracking_node = perception_tracking.perception_node:main",
        ],
    },
)
