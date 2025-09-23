from setuptools import find_packages, setup

package_name = 'drone_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='claudia',
    maintainer_email='Claudia@student.uts.edu.au',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'teleop = drone_control.teleop:main',
            'teleop_joystick = drone_control.teleop_joystick:main',
            'open_loop_control = drone_control.open_loop_control:main',
            'drone_position_control = drone_control.drone_position_control:main'
        ],
    },
)
