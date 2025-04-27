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

import rclpy 
from rclpy.node import Node
from std_msgs.msg import String
from nanollm_interfaces.msg import StringStamped
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
from PIL import Image as im
from nano_llm import NanoLLM, ChatHistory
import numpy as np
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
 

class Rosbag_Nano_LLM(Node):

    def __init__(self):
        super().__init__('rosbag_nano_llm')
        
        #EDIT MODEL HERE 
        self.declare_parameter('model', "Efficient-Large-Model/VILA1.5-3b") #inserting vila
        self.declare_parameter('api', "mlc")
        self.declare_parameter('quantization', "q4f16_ft")
        self.declare_parameter('is_compressed', False)
        self.declare_parameter('query', "Describe the image.")

        self.model_name = self.get_parameter('model').get_parameter_value().string_value
        self.query = self.get_parameter('query').get_parameter_value().string_value
        self.is_compressed = self.get_parameter('is_compressed').get_parameter_value().bool_value

        # Subscriber for input query
        self.query_subscription = self.create_subscription(
            String,
            'input_query',
            self.query_listener_callback,
            10)
        self.query_subscription  # prevent unused variable warning

        # Subscriber for input image
        if self.is_compressed:
            qos_profile = QoSProfile(
                depth=10,
                reliability=QoSReliabilityPolicy.BEST_EFFORT
            )
            self.image_subscription = self.create_subscription(
                CompressedImage,
                'input_image',
                self.compressedimage_listener_callback,
                qos_profile)
        else:
            self.image_subscription = self.create_subscription(
                Image,
                'input_image',
                self.image_listener_callback,
                10)
        self.image_subscription  # prevent unused variable warning

        # To convert ROS image message to OpenCV image
        self.cv_br = CvBridge() 
        self.cv_img = None
      
        #load the model 
        self.get_logger().info(f"load model: {self.model_name}")
        self.model = NanoLLM.from_pretrained(self.model_name)

        #chatHistory var 
        self.chat_history = ChatHistory(self.model)

        ##  PUBLISHER
        self.output_publisher = self.create_publisher(StringStamped, '/output', 10)
        if self.is_compressed:
            self.image_publisher =  self.create_publisher(Image, "/source_image", 10)

        timer_period = 0.001
        self.timer = self.create_timer(timer_period, self.nano_llm_inference)            

    def query_listener_callback(self, msg):
        #can change with user needs 
        self.get_logger().info(f"query_listener_callback: {msg.data}") # debug
        self.query = msg.data


    def compressedimage_listener_callback(self, data):
        # self.get_logger().info(f"compressedimage_listener_callback")
        self.image_stamp = data.header.stamp
        self.cv_img = self.cv_br.compressed_imgmsg_to_cv2(data, "bgr8")
        img_msg = self.cv_br.cv2_to_imgmsg(self.cv_img)
        img_msg.header.stamp = data.header.stamp
        img_msg.header.frame_id = "image"
        self.image_publisher.publish(img_msg)
        # self.nano_llm_inference()


    def image_listener_callback(self, data): 
        # self.get_logger().info(f"image_listener_callback") # debug
        # call model with input_query and input_image 
        self.image_stamp = data.header.stamp
        self.cv_img = self.cv_br.imgmsg_to_cv2(data, 'rgb8')
        # self.nano_llm_inference()


    def nano_llm_inference(self):
        # self.get_logger().info(f"nano_llm_inference") # debug
        if self.cv_img is not None:
            # self.get_logger().info(f"get new frame") # debug
            input_query = self.query
            stamp = self.image_stamp
            PIL_img = im.fromarray(self.cv_img)

            # Parsing input text prompt
            prompt = input_query.strip("][()")
            text = prompt.split(',')
            self.get_logger().info('Your query: %s' % text) #can check to see what the query is 

            #chathistory 
            self.chat_history.append('user', image=PIL_img)
            self.chat_history.append('user', prompt, use_cache=True)
            embedding, _ = self.chat_history.embed_chat()
        
            output = self.model.generate(
                inputs=embedding,
                kv_cache=self.chat_history.kv_cache,
                min_new_tokens = 10,
                streaming = False, 
                do_sample = True,
            )

            #FIX PUBLISHER 
            # output_msg = String()
            output_msg = StringStamped()
            output_msg.header.stamp = stamp
            output_msg.data = output
            self.output_publisher.publish(output_msg)
            self.get_logger().info(f"output: {output_msg}")
            self.get_logger().info(f"Published output: {output}")

            self.chat_history.reset()
            self.cv_img = None



def main(args=None):
    rclpy.init(args=args)
    rosbag_nano_llm = Rosbag_Nano_LLM()
    # rosbag_nano_llm.nano_llm_inference()
    rclpy.spin(rosbag_nano_llm)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    rosbag_nano_llm.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

