from setuptools import find_packages, setup

package_name = 'myagv_plus_camera'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config',
            ['config/camera.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lanni',
    maintainer_email='weijun.xie@elephantrobotics.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'csi_camera = myagv_plus_camera.csi_camera:main'
        ],
    },
)
