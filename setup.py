import os
from setuptools import setup
from setuptools import find_packages
from glob import glob

def collect_files_with_paths(base_dir, install_base):
    """Recursively collect all files in a directory, preserving subdirectory structure."""
    collected_files = []
    for root, _, files in os.walk(base_dir):
        for file in files:
            # Calculate the install directory relative to the base directory
            install_path = os.path.join(install_base, os.path.relpath(root, base_dir))
            # Add file and its target installation path
            collected_files.append((install_path, [os.path.join(root, file)]))
    return collected_files

package_name = 'antsy_description'

data_files = [
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
    (os.path.join('share', package_name, 'config'), glob('config/*')),
]

# Add all files in the 'meshes' folder
data_files.extend(collect_files_with_paths('meshes', f'share/{package_name}/meshes'))

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Pedro Rocha',
    maintainer_email='pedrosr711@gmail.com',
    description='The ' + package_name + ' package',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [],
    },
)
