from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'tension_estimator'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nuc_6g_life_3',
    maintainer_email='eduardomilanezaraujo@gmail.com',
    description='ROS2 package for force/tension estimation from tactile sensors',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'force_estimator_node = tension_estimator.force_estimator_node:main',
        ],
    },
)
