# ros2_nanollm
ROS2 nodes for LLM, VLM, VLA

[NanoLLM](https://github.com/dusty-nv/NanoLLM/tree/main) optimizes many different models to run on the NVIDIA Jetson Orin. This project provides a ROS 2 package for scene description using NanoLLM. 


## Setup 

1. Set up your ROS development environment by following the instructions [here](https://docs.ros.org/en/humble/Installation.html).
2. Ensure that Docker is installed by following the instructions [here](https://docs.docker.com/engine/install/).
3. Install the NanoLLM Docker Container by following the installation instructions [here](https://dusty-nv.github.io/NanoLLM/install.html).
4. Clone the required project under ```${ros2_ws}/src```
   
   ```
   jetson-containers run -v ~/ros2_ws:/root/ros2_ws dustynv/ros:humble-llm-r36.3.0
   ```
