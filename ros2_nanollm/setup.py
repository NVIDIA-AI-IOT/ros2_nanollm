# SPDX-FileCopyrightText: Copyright (c) year NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

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
        ('share/' + package_name + '/configs', glob('configs/*.yaml')),
    ],
    install_requires=['setuptools', 'nanollm_interfaces'],
    zip_safe=True,
    maintainer='kshaltiel',
    maintainer_email='kshaltiel@nvidia.com',
    description='ROS 2 package for NanoLLM',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
		'nano_llm_py = ros2_nanollm.nano_llm_py:main',
        'offline_llm_video_description_py = ros2_nanollm.offline_llm_video_description_py:main',
        'rosbag_nano_llm_py = ros2_nanollm.rosbag_nano_llm_py:main',
        'rosbag_image_caption_scorer = ros2_nanollm.rosbag_image_caption_scorer:main',
        'ground_truth_label_collector = ros2_nanollm.ground_truth_label_collector:main',
        # 'image_saver = image_saver.image_saver:main',
        ],
    },
    
)
