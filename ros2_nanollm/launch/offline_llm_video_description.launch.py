# SPDX-FileCopyrightText: Copyright (c) <year> NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    launch_args = [
        DeclareLaunchArgument(
            'api',
            default_value='mlc',
            description='The model backend to use'),
        DeclareLaunchArgument(
            'quantization',
            default_value='q4f16_ft',
            description='The quantization method to use'),
        DeclareLaunchArgument(
            'video_path',
            default_value='/ros2_workspace/vlm_sample_videos/PXL_20250304_071458198.mp4',
            description='Input video file path to process'),
        DeclareLaunchArgument(
            'query',
            default_value='Describe the image.',
            description='The query to hand over to LLM'),
    ]

    # NanoOWL parameters
    # thresholds = LaunchConfiguration('thresholds')
    # image_encoder_engine = LaunchConfiguration('image_encoder_engine')

    #NanoLLM Parameters 
    model = LaunchConfiguration('model')
    api = LaunchConfiguration('api')
    quantization = LaunchConfiguration('quantization')
    video_path = LaunchConfiguration('video_path')
    query = LaunchConfiguration('query')

    nanollm_node = Node(
            package='ros2_nanollm', #make sure package is named this
            executable='offline_llm_video_description_py',
            parameters=[{
                'model': 'Efficient-Large-Model/VILA1.5-3b',
                # 'model': 'Efficient-Large-Model/VILA1.5-13b',
                'api': api, #'mlc',
                'quantization': quantization, #'q4f16_ft',
                'video_path': video_path,
                'query': query,
            }]
    )
    
    # final_launch_description = launch_args + [cam2image_node] + [nanollm_node]
    final_launch_description = launch_args + [nanollm_node]
    
    return LaunchDescription(final_launch_description)
