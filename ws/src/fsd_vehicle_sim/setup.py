from setuptools import setup
import os
from glob import glob

package_name = 'fsd_vehicle_sim'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        # make package discoverable
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        # install package.xml
        ('share/' + package_name, ['package.xml']),
        # install launch files
        ('share/' + package_name + '/launch', glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Wolfi',
    maintainer_email='w.bradfisch@gmx.de',
    description='Simple bicycle vehicle simulator with GPS and ground speed',
    license='MIT',
    entry_points={
        'console_scripts': [
            # ros2 run fsd_vehicle_sim bicycle_sim
            'bicycle_sim = fsd_vehicle_sim.bicycle_sim_node:main',
        ],
    },
)
