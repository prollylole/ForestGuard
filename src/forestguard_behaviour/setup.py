from setuptools import find_packages, setup

package_name = 'forestguard_behaviour'

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
    maintainer='john',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
    'console_scripts': [
        'run_ui             = forestguard_behaviour.app:main',
        'controller_bridge  = forestguard_behaviour.controller_bridge:main',
        'twist_scaler       = forestguard_behaviour.twist_scaler:main',
        'battery_sim        = forestguard_behaviour.battery_sim:main',
        'autonomy           = forestguard_behaviour.autonomy:main',
    ],
    }
)
