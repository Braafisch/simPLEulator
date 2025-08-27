from setuptools import setup

package_name = 'fsd_accel_track'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/accel_track.launch.py']),
        # ('share/' + package_name + '/meshes', ['meshes/cone.dae']),  # optional
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Wolfi',
    maintainer_email='w.bradfisch@gmx.de',
    description='FSD Accel Track markers for RViz',
    license='MIT',
    entry_points={
        'console_scripts': [
            'accel_track_markers = fsd_accel_track.accel_track_markers:main',
        ],
    },
)
