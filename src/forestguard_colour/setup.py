from setuptools import setup

package_name = 'forestguard_colour'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/colour_detector.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='john',
    maintainer_email='john@example.com',
    description='Colour-based perception for ForestGuard',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tree_detector = forestguard_colour.tree_detector_node:main',
            'tree_detector_v2 = forestguard_colour.tree_detector_v2:main',
            'hsv_calibrator = forestguard_colour.hsv_calibrator:main',
        ],
    },
)
