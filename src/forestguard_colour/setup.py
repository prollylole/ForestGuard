from setuptools import setup
package_name = 'forestguard_colour'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/colour_detector.launch.py',
            'launch/colour_detector_params.launch.py',   # ← ensure this line exists
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='john',
    maintainer_email='cohnjhen@gmail.com',
    description='ForestGuard colour detector node (step 3)',
    license='MIT',
    entry_points={
        'console_scripts': [
            'tree_detector = forestguard_colour.tree_detector_node:main',
            'hsv_calibrator = forestguard_colour.hsv_calibrator:main',
        ],
    },
)
