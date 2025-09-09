from setuptools import setup, find_packages

package_name = "fsd_localization"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        # install launch files
        ("share/" + package_name + "/launch", ["launch/localization.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Wolfi",
    maintainer_email="w.bradfisch@gmx.de",
    description="Simple 2D localization",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "localization = localization.localization:main",
        ],
    },
)
