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
            'launch/color_detector.launch.py',
            'launch/color_detector_params.launch.py',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='You',
    maintainer_email='you@example.com',
    description='Camera subscriber: converts ROS Image to OpenCV (step 2).',
    license='MIT',
    entry_points={
        'console_scripts': [
            'tree_detector = forestguard_colour.tree_detector_node:main',
            'hsv_calibrator = forestguard_colour.hsv_calibrator:main',
        ],
    },  
)

