from setuptools import setup
import os

package_name = 'forest_guard_sim'

def gather_package_data(package_root, install_share):
    """
    Walk package_root and return a data_files-style list of tuples:
      (destination_dir, [file1, file2, ...])
    destination_dir is relative and will be joined with 'share/<package_name>' by setup().
    Only regular files are included (no directories).
    """
    data = []
    for root, dirs, files in os.walk(package_root):
        if not files:
            continue
        # relative path under package root
        rel_dir = os.path.relpath(root, package_root)
        # map to share/<package_name>/<rel_dir> (if rel_dir == '.', map to share/<package_name>/<package_root>)
        if rel_dir == '.':
            dest = os.path.join('share', package_name, package_root)
        else:
            dest = os.path.join('share', package_name, package_root, rel_dir)
        # add full paths for files
        paths = [os.path.join(root, f) for f in files]
        data.append((dest, paths))
    return data

data_files = [
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
]

# gather standard directories if they exist
for d in ('launch', 'config', 'worlds', 'models', 'urdf', 'test', 'maps'):
    if os.path.isdir(d):
        data_files.extend(gather_package_data(d, os.path.join('share', package_name, d)))

# also include top-level files like README if present
if os.path.isfile('README.md'):
    data_files.append((os.path.join('share', package_name), ['README.md']))

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='claudia',
    maintainer_email='your_email@example.com',
    description='Forest Guard simulation package',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [],
    },
)