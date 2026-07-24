from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'myagv_plus_autocharge'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Elephant Robotics',
    maintainer_email='support@elephantrobotics.com',
    description='myAGV Plus automatic charging system',
    license='MIT',
    entry_points={
        'console_scripts': [
            'combined_auto_recharger = myagv_plus_autocharge.combined_auto_recharger:main',
        ],
    },
)
