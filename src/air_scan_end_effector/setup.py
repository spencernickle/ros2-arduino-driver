from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'air_scan_end_effector'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
		(os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
		(os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*[yaml]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='spencer',
    maintainer_email='spencer@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
			'air_scan_driver = air_scan_end_effector.air_scan_end_effector:main',
            'test_commands = air_scan_end_effector.test_commands:main'
        ],
    },
)
