import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'ros2_nanollm'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
	('share/' + package_name, glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kshaltiel',
    maintainer_email='kshaltiel@nvidia.com',
    description='ROS 2 package for NanoLLM',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
		'nano_llm_py = ros2_nanollm.nano_llm_py:main'
        ],
    },
)
