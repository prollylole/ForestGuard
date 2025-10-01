# forest_drone/setup.py
from setuptools import setup
import os
from glob import glob

package_name = 'forest_drone'

data_files = [
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    ('share/' + package_name + '/launch', glob('launch/*.py')),

     # Install launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
]

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name] if os.path.isdir(package_name) else [],
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='claudia',
    maintainer_email='Claudia@student.uts.edu.au',
    description='forest_drone launch file to bring up forest and drone',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
