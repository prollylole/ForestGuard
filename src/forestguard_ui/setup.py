from setuptools import setup, find_packages

package_name = 'forestguard_ui'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(include=[package_name, f"{package_name}.*"]),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yut',
    maintainer_email='you@example.com',
    description='PySide6 GUI for ForestGuard',
    license='MIT',
    entry_points={
        'console_scripts': [
            'run_ui = forestguard_ui.app:main',
        ],
    },
)
