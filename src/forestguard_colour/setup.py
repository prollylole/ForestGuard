from setuptools import setup, find_packages
from glob import glob
import os

package_name = 'forestguard_colour'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(),  # picks up forestguard_colour because of __init__.py
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Install ALL launch files, including tree_mapper.launch.py
        (os.path.join('share', package_name, 'launch'),
         glob('launch/*.launch.py')),
        # (optional) if you add configs later:
        # (os.path.join('share', package_name, 'config'), glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='john',
    maintainer_email='john@example.com',
    description='Colour-based perception for ForestGuard',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'tree_detector      =   forestguard_colour.tree_detector_node:main',
            'tree_detector_v2   =   forestguard_colour.tree_detector_v2:main',
            'hsv_calibrator     =   forestguard_colour.hsv_calibrator:main',
            'camera_tree_mapper =   forestguard_colour.camera_tree_mapper:main',  
            'lidar_tree_mapper  =   forestguard_colour.lidar_tree_mapper:main',
            'tree_colour_confirmer = forestguard_colour.tree_colour_confirmer:main',
        ],
    },
)
