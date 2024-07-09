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

## Usage

```ros2 launch ros2_nanollm camera_input_example.launch.py model:=<path-to-model> api:=<model-backend> quantization:=<method>```

## ROS Parameters

| ROS Parameter | Type | Default | Description |
| --- | --- | --- | --- |
| model | string | Efficient-Large-Model/Llama-3-VILA1.5-8B | The model to use |
| api | string | mlc | The model backend to use |
| quantization | string | q4f16_ft | Quantization method to use |

## Topics Subscribed

| ROS Topic | Interface | Description |
| --- | --- | --- |
| input_image | [sensor_msgs/Image](https://github.com/ros2/common_interfaces/blob/humble/sensor_msgs/msg/Image.msg) | The image on which analysis is to be performed |
| input_query | [std_msgs/String](https://github.com/ros2/common_interfaces/blob/humble/std_msgs/msg/String.msg) | A prompt for the model to generate a corresponding response |

## Topics Published

| ROS Topic | Interface | Description |
| --- | --- | --- |
| output_msg | [std_msgs/String](https://github.com/ros2/common_interfaces/blob/humble/std_msgs/msg/String.msg) | The output message summarizing the model's conclusions from the inputs |
