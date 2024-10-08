from setuptools import find_packages, setup

import os 
from glob import glob

package_name = 'abb_arm'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))), 
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.*'))),
        (os.path.join('share', package_name, 'worlds'), glob(os.path.join('worlds', '*.world'))),           
        (os.path.join('share', package_name, 'description', 'urdf'), glob(os.path.join('description','urdf', '*.xacro'))),
        (os.path.join('share', package_name, 'description', 'meshes', 'collision'), glob(os.path.join('description','meshes','collision', '*.stl'))),
        (os.path.join('share', package_name, 'description', 'meshes', 'visual'), glob(os.path.join('description','meshes','visual', '*.dae'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mandred',
    maintainer_email='mandredking@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'trajectory_planner_script = abb_arm.trajectory_planner:main'
        ],
    },
)
