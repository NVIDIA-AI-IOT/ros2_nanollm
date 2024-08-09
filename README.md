# ros2_nanollm
ROS2 nodes for LLM, VLM, VLA

[NanoLLM](https://github.com/dusty-nv/NanoLLM/tree/main) optimizes many different models to run on the NVIDIA Jetson Orin. This project provides a ROS 2 package for scene description using NanoLLM. 

<p align="center">
  <img src="assets/sample.gif" width="65%">
</p>


## Setup 

1. Run through these [setup steps](https://github.com/dusty-nv/jetson-containers/blob/master/docs/setup.md) to get your Docker engine configured.
2. Set up your ROS development environment by following the instructions [here](https://docs.ros.org/en/humble/Installation.html).
3. Install the NanoLLM Docker Container by following the installation instructions [here](https://dusty-nv.github.io/NanoLLM/install.html).

### Dev Mode

By default, the `ros2_nanollm` package is built into the container and installed under `/ros2_workspace` (which is an environment automatically sourced on container startup).  But if you would like to edit this package interactively, you can do so by cloning it to an external workspace on your host device, and then mounting that workspace overlaid on the original location.

```
# make a ROS workspace somewhere outside container, and clone ros2_nanollm
mkdir -p ~/ros2_workspace/src
cd ~/ros2_workspace/src
git clone https://github.com/NVIDIA-AI-IOT/ros2_nanollm

# start nano_llm:humble container, mounting in your own workspace
jetson-containers run -v ~/ros2_workspace:/ros2_workspace $(autotag nano_llm:humble)

# build the mounted workspace (this is running inside container at this point)
cd /ros2_workspace
colcon build --symlink-install --base-paths src
bash /ros2_workspace/install/setup.bash

# check that the nodes are still there
ros2 pkg list | grep ros2_nanollm
ros2 pkg executables ros2_nanollm
```

## Usage

```
jetson-containers run $(autotag nano_llm:humble) /
    ros2 launch ros2_nanollm camera_input_example.launch.py /
        model:=<path-to-model> api:=<model-backend> quantization:=<method>
```

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
