from setuptools import setup

package_name = 'myagv_plus_pedestrian_follower'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/pedestrian_follower.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='AGV Plus pedestrian following node',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pedestrian_follower = myagv_plus_pedestrian_follower.pedestrian_follower:main',
            'gesture_control = myagv_plus_pedestrian_follower.gesture_control:main',
        ],
    },
)
