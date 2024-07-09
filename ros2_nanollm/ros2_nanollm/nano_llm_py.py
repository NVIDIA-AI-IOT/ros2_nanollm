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

import rclpy #this imports the main rclpy package, which is the ROS 2 client library for Python. rclpy provides the Python bindings for the ROS 2 
                #middleware, allowing you to create ROS 2 nodes, publish and subscribe to topics, and interact with the ROS 2 system.
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from PIL import Image as im
from nano_llm import NanoLLM, ChatHistory
import numpy as np
 

class Nano_LLM_Subscriber(Node):

    def __init__(self):
        super().__init__('nano_llm_subscriber')
        
        #EDIT MODEL HERE 
        self.declare_parameter('model', "Efficient-Large-Model/Llama-3-VILA1.5-8B") #inserting vila
        self.declare_parameter('api', "mlc")
        self.declare_parameter('quantization', "q4f16_ft")
      
        # Subscriber for input query
        self.query_subscription = self.create_subscription(
            String,
            'input_query',
            self.query_listener_callback,
            10)
        self.query_subscription  # prevent unused variable warning

        # Subscriber for input image
        self.image_subscription = self.create_subscription(
            Image,
            'input_image',
            self.image_listener_callback,
            10)
        self.image_subscription  # prevent unused variable warning

        # To convert ROS image message to OpenCV image
        self.cv_br = CvBridge() 
        
        

        #load the model 
        self.model = NanoLLM.from_pretrained("Efficient-Large-Model/Llama-3-VILA1.5-8B")

        #chatHistory var 
        self.chat_history = ChatHistory(self.model)

        ##  PUBLISHER
        self.output_publisher = self.create_publisher(String, 'output', 10)
        self.query = "Describe the image."

    def query_listener_callback(self, msg):
        #can change with user needs 
        self.query = msg.data


    def image_listener_callback(self, data): 
        input_query = self.query
        #input_model = self.get_parameter('model').get_parameter_value().string_value
        
       
        # call model with input_query and input_image 
        cv_img = self.cv_br.imgmsg_to_cv2(data, 'rgb8')
        PIL_img = im.fromarray(cv_img)
        #image_np = np.array(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, 3))
        #load the model 
        #model = NanoLLM.from_pretrained(input_model)

        #search to see if you can even do this
        #embedding = self.model.embed_image(PIL_img, return_tensors='np', return_dict=False)


        # Parsing input text prompt
        prompt = input_query.strip("][()")
        text = prompt.split(',')
        self.get_logger().info('Your query: %s' % text) #can check to see what the query is 


        #chathistory 
        self.chat_history.append('user', image=PIL_img)
        self.chat_history.append('user', prompt, use_cache=True)
        embedding, _ = self.chat_history.embed_chat()

        #text_token = self.model.embed_text(prompt, return_tensors = 'np')

#        input = {
 #           'image': embedding,
  #          'text': text_token
   #     }
        
        #check with asawaree
        output = self.model.generate(
            inputs=embedding,
            kv_cache=self.chat_history.kv_cache,
#            prompt = prompt,
            min_new_tokens = 10,
            streaming = False, 
            do_sample = True,
        )

        #FIX PUBLISHER 
        output_msg = String()
        output_msg.data = output
        self.output_publisher.publish(output_msg)
        self.get_logger().info(f"Published output: {output}")

        self.chat_history.reset()



def main(args=None):
    rclpy.init(args=args)

    nano_llm_subscriber = Nano_LLM_Subscriber()

    rclpy.spin(nano_llm_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    nano_llm_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


#use print statements to understand dusty code example
#install cam2image node 
#use the launch file as a template 

#launch
#change nanoowl_node to nanollm in cam launch -- launch arg specific to nanollm 
#dont need nano_owl_example.launch -- used for noncamera images like rosbag
