from setuptools import find_packages, setup

package_name = 'smart_logistics_kit_lite'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/resource',['resource/SIMFANG.TTF']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lanni',
    maintainer_email='weijun.xie@elephantrobotics.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'main = smart_logistics_kit_lite.main:main',
            'parking = smart_logistics_kit_lite.parking:main',
            'arm_controller = smart_logistics_kit_lite.arm_controller:main',
            'relative_move = smart_logistics_kit_lite.relative_move:main'
        ],
    },
)
