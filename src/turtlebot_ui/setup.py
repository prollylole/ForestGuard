from setuptools import find_packages, setup

package_name = 'turtlebot_ui'

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
    maintainer='yut',
    maintainer_email='yuto.j.boittiaux@student.uts.edu.au',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
    'console_scripts': [
        'run_ui = turtlebot_ui.app:main',
        'controller_bridge = turtlebot_ui.controller_bridge:main',
        'twist_scaler = turtlebot_ui.twist_scaler:main',
    ],
    }
)

