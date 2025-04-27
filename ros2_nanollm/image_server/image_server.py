import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
import datetime

class ImageSaver(Node):
    def __init__(self):
        super().__init__('image_saver')

        # 画像保存用ディレクトリ
        self.image_save_path = os.path.expanduser("./ros2_images")
        os.makedirs(self.image_save_path, exist_ok=True)

        # CvBridgeインスタンス（ROS画像メッセージをOpenCV形式に変換）
        self.bridge = CvBridge()

        # 画像のサブスクライバ
        self.image_subscription = self.create_subscription(
            Image,
            'input_image',
            self.image_callback,
            10)
        
        # キーボード入力のサブスクライバ（スペースキー用）
        self.keyboard_subscription = self.create_subscription(
            String,
            'keyboard',
            self.keyboard_callback,
            10)

        self.latest_image = None
        self.get_logger().info("ImageSaver Node is ready. Press SPACE to save an image.")

    def image_callback(self, msg):
        """ 画像を受信したときに保存 """
        self.latest_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

    def keyboard_callback(self, msg):
        """ スペースキーが押されたら画像を保存 """
        if msg.data == "SPACE" and self.latest_image is not None:
            timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = os.path.join(self.image_save_path, f"image_{timestamp}.jpg")
            cv2.imwrite(filename, self.latest_image)
            self.get_logger().info(f"Image saved: {filename}")

def main(args=None):
    rclpy.init(args=args)
    node = ImageSaver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
