import os
from collections import defaultdict
from pathlib import Path

from setuptools import find_packages, setup

package_name = 'smart_logistics_kit_lite'


def collect_data_files(*directories):
    files_by_destination = defaultdict(list)
    for directory in directories:
        root = Path(directory)
        if not root.is_dir():
            continue
        for path in sorted(root.rglob('*')):
            if not path.is_file():
                continue
            if '__pycache__' in path.parts or path.suffix in {'.pyc', '.pyo'}:
                continue
            destination = os.path.join('share', package_name, str(path.parent))
            files_by_destination[destination].append(str(path))
    return sorted(files_by_destination.items())


data_files = [
    ('share/ament_index/resource_index/packages',
        ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    ('share/' + package_name + '/resource', ['resource/SIMFANG.TTF']),
]

data_files += collect_data_files('launch', 'param', 'rviz', 'map', 'graphs', 'scripts')

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lanni',
    maintainer_email='weijun.xie@elephantrobotics.com',
    description='Smart Logistics Kit Lite for ROS2 Humble',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'main = smart_logistics_kit_lite.main:main',
            'route_bridge = smart_logistics_kit_lite.route_bridge:main',
            'parking = smart_logistics_kit_lite.parking:main',
            'arm_controller = smart_logistics_kit_lite.arm_controller:main',
        ],
    },
)
