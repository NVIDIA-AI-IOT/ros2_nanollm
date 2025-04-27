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
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from PIL import Image as im
from nano_llm import NanoLLM, ChatHistory
import numpy as np
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy
import cv2
 

class Offline_LLM_Video_Description(Node):

    def __init__(self):
        super().__init__('Offline_LLM_Video_Description')
        
        #EDIT MODEL HERE 
        self.declare_parameter('model', "Efficient-Large-Model/VILA1.5-3b") #inserting vila
        self.declare_parameter('api', "mlc")
        self.declare_parameter('quantization', "q4f16_ft")
        self.declare_parameter('video_path', "")
        self.declare_parameter('query', "Describe the image.")

        self.model_name = self.get_parameter('model').get_parameter_value().string_value
        self.query = self.get_parameter('query').get_parameter_value().string_value

        video_path = self.get_parameter('video_path').get_parameter_value().string_value
        self.cap = cv2.VideoCapture(video_path)

        # To convert ROS image message to OpenCV image
        self.cv_br = CvBridge() 
      
        #load the model 
        self.get_logger().info(f"load model: {self.model_name}")
        self.model = NanoLLM.from_pretrained(self.model_name)

        #chatHistory var 
        self.chat_history = ChatHistory(self.model)

        ##  PUBLISHER
        self.image_publisher = self.create_publisher(Image, '/input_image', 10)
        self.output_publisher = self.create_publisher(StringStamped, '/output', 10)

        self.describe_video_image()

    def describe_video_image(self): 
        input_query = self.query     

        ret = True
        while ret:
            ret, frame = self.cap.read()
            PIL_img = im.fromarray(frame)
            self.get_logger().info(f'Current Frame: {self.cap.get(cv2.CAP_PROP_POS_FRAMES)}')

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

            stamp = self.get_clock().now().to_msg()
            #FIX PUBLISHER 
            output_msg = StringStamped()
            output_msg.header.stamp = stamp
            output_msg.data = output
            self.output_publisher.publish(output_msg)
            self.get_logger().info(f"output: {output_msg}")
            self.get_logger().info(f"Published output: {output}")

            img_msg = self.cv_br.cv2_to_imgmsg(frame, encoding="bgr8")
            img_msg.header.stamp = stamp
            img_msg.header.frame_id = "image"
            self.image_publisher.publish(img_msg)
            self.get_logger().info(f"Published {self.cap.get(cv2.CAP_PROP_POS_FRAMES)}-th image.")

            self.chat_history.reset()

def main(args=None):
    rclpy.init(args=args)
    offline_llm_video_description = Offline_LLM_Video_Description()

    rclpy.spin(offline_llm_video_description)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    offline_llm_video_description.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

