from setuptools import setup

package_name = "fsd_bringup"

setup(
    name=package_name,
    version="0.0.1",
    packages=[],  # no Python modules, only launch files
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", ["launch/fsd.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Wolfi",
    maintainer_email="w.bradfisch@gmx.de",
    description="Bringup package to start FSD stack",
    license="MIT",
)

