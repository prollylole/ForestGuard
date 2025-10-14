from setuptools import setup
from glob import glob

package_name = 'forest_guard_sim'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/worlds', glob('worlds/*.sdf')),
        ('share/' + package_name + '/models/forest_env', [
            'models/forest_env/model.sdf',
            'models/forest_env/model.config',
        ]),
        ('share/' + package_name + '/models/forest_env/meshes',
         glob('models/forest_env/meshes/*')),
        ('share/' + package_name + '/models/forest_env/materials/textures',
         glob('models/forest_env/materials/textures/*')),
         ('share/' + package_name + '/models/tree1', [
    'models/tree1/model.sdf',
    'models/tree1/model.config',
]),
('share/' + package_name + '/models/tree1/meshes',
 glob('models/tree1/meshes/*')),
('share/' + package_name + '/models/tree1/materials/textures',
 glob('models/tree1/materials/textures/*')),
 ('share/' + package_name + '/models/tree2', [
    'models/tree2/model.sdf',
    'models/tree2/model.config',
]),
('share/' + package_name + '/models/tree2/meshes',
 glob('models/tree2/meshes/*')),
('share/' + package_name + '/models/tree2/materials/textures',
 glob('models/tree2/materials/textures/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yut',
    maintainer_email='yuto.j.boittiaux@student.uts.edu.au',
    description='Gazebo world with a textured ground environment',
    license='MIT',
    tests_require=['pytest'],
    entry_points={'console_scripts': []},
)