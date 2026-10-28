from setuptools import setup

package_name = 'myagv_plus_joystick_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'pymycobot'],
    dependency_links=[],

    zip_safe=True,
    maintainer='elephant',
    maintainer_email='elephant@example.com',
    description='Joystick control for myagv_plus',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'joystick_controller = myagv_plus_joystick_control.joystick_controller:main',
        ],
    },
)
