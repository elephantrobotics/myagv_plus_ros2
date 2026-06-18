import os
from collections import defaultdict
from pathlib import Path

from setuptools import find_packages, setup

package_name = 'smart_logistics_kit'


def collect_data_files(*directories):
    files_by_destination = defaultdict(list)
    for directory in directories:
        root = Path(directory)
        for path in sorted(root.rglob('*')):
            if not path.is_file():
                continue
            if '__pycache__' in path.parts or path.suffix in {'.pyc', '.pyo'}:
                continue
            destination = os.path.join('share', package_name, str(path.parent))
            files_by_destination[destination].append(str(path))
    return sorted(files_by_destination.items())


def collect_share_root_files(directory, exclude_names=None):
    exclude_names = set(exclude_names or [])
    root = Path(directory)
    return [
        str(path)
        for path in sorted(root.iterdir())
        if path.is_file()
        and path.name not in exclude_names
        and path.suffix not in {'.pyc', '.pyo'}
    ]


data_files = [
    ('share/ament_index/resource_index/packages',
        ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
]

share_root_files = collect_share_root_files('resource', exclude_names={package_name})
if share_root_files:
    data_files.append((os.path.join('share', package_name), share_root_files))

data_files += collect_data_files('launch', 'param', 'rviz', 'map', 'graphs')

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your@email.com',
    description='Smart Logistics Kit Lite for ROS2 Humble',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'main = smart_logistics_kit.main:main',
            'autocharge_coordinator = smart_logistics_kit.autocharge_coordinator:main',
            'test_route_server = smart_logistics_kit.test_route_server:main',
            'route_bridge = smart_logistics_kit.route_bridge:main',
            'parking = smart_logistics_kit.parking:main',
        ],
    },
)
