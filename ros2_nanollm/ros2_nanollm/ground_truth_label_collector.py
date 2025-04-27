import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String
import cv2
import numpy as np
import csv
import os
import yaml
import sys
from ament_index_python.packages import get_package_share_directory

class LabelCollectorNode(Node):
    def __init__(self):
        super().__init__('label_collector_node')

        self.label_config = self.load_label_config()
        if 's' in self.label_config:
            self.get_logger().error("キー 's' は is_stopping 用に予約されているため、ラベル定義で使用できません。")
            rclpy.shutdown()
            sys.exit(1)
        if 'q' in self.label_config:
            self.get_logger().error("キー 'q' は 終了 用に予約されているため、ラベル定義で使用できません。")
            rclpy.shutdown()
            sys.exit(1)

        self.key_to_label = self.label_config
        self.current_key = list(self.key_to_label.keys())[0]
        self.is_stopping = 0
        self.csv_rows = []
        self.save_dir = None
        self.last_input_time = self.get_clock().now()
        self.input_timeout_sec = 3.0

        qos = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.subscription = self.create_subscription(
            CompressedImage, '/input_image', self.listener_callback, qos)
        self.dir_subscription = self.create_subscription(
            String, '/save_dir', self.save_dir_callback, 10)
        self.create_timer(0.5, self.check_input_timeout)

        cv2.namedWindow("Input Image", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("Input Image", 800, 600)

        label_list = ", ".join(f"{k}={v['name']}" for k, v in self.key_to_label.items())
        self.get_logger().info(f"キー操作: {label_list}, s=停車切替, q=手動保存+終了")

    def load_label_config(self):
        try:
            pkg_path = get_package_share_directory('ros2_nanollm')
            yaml_path = os.path.join(pkg_path, 'configs', 'labels.yaml')
            with open(yaml_path, 'r') as f:
                data = yaml.safe_load(f)
                return data['labels']
        except Exception as e:
            self.get_logger().error(f"labels.yaml の読み込みに失敗: {e}")
            rclpy.shutdown()
            sys.exit(1)

    def save_dir_callback(self, msg: String):
        new_dir = msg.data
        if self.csv_rows and self.save_dir and new_dir != self.save_dir:
            self.get_logger().info(f"/save_dir が変更されました。前のディレクトリ ({self.save_dir}) に保存します。")
            self.save_csv(self.save_dir)
            self.csv_rows.clear()
        self.save_dir = new_dir
        self.get_logger().info(f"新しい保存先ディレクトリが設定されました: {self.save_dir}")

    def listener_callback(self, msg: CompressedImage):
        self.last_input_time = self.get_clock().now()
        np_arr = np.frombuffer(msg.data, np.uint8)
        image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        if image is None:
            self.get_logger().warn("画像のデコードに失敗しました")
            return

        label_text = self.key_to_label[self.current_key]['name']
        stopping_text = "True" if self.is_stopping == 1 else "False"
        display_text = f'Label: {label_text} | Stopping: {stopping_text}'
        self.draw_label_overlay(image, display_text)

        cv2.imshow("Input Image", image)

        key = cv2.waitKey(1) & 0xFF
        if key != 255:
            key_char = chr(key).lower()
            if key_char == 's':
                self.is_stopping = 1 - self.is_stopping
                self.get_logger().info(f'is_stopping toggled to {self.is_stopping}')
            elif key_char == 'q':
                self.get_logger().info("手動保存 & ノード終了要求")
                self.save_csv()
                cv2.destroyAllWindows()
                rclpy.shutdown()
            elif key_char in self.key_to_label:
                self.current_key = key_char
                label_name = self.key_to_label[self.current_key]['name']
                self.get_logger().info(f'Label set to {label_name} ({self.current_key})')

        timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        csv_label = self.key_to_label[self.current_key]['value']
        self.csv_rows.append([timestamp, csv_label, self.is_stopping])

    def draw_label_overlay(self, image, text, position=(10, 100)):
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 4.0
        thickness = 8
        cv2.putText(image, text, position, font, font_scale, (255, 255, 255), thickness + 4, cv2.LINE_AA)
        cv2.putText(image, text, position, font, font_scale, (0, 255, 0), thickness, cv2.LINE_AA)

    def check_input_timeout(self):
        now = self.get_clock().now()
        elapsed = (now - self.last_input_time).nanoseconds * 1e-9
        if elapsed > self.input_timeout_sec and self.csv_rows:
            self.get_logger().warn(f"{self.input_timeout_sec}秒間 /input_image が届いていません。自動保存します。")
            self.save_csv()
            self.csv_rows.clear()
            self.last_input_time = now

    def save_csv(self, target_dir=None):
        dir_to_use = target_dir or self.save_dir
        if not dir_to_use:
            self.get_logger().error("保存先ディレクトリが指定されていません")
            return

        os.makedirs(dir_to_use, exist_ok=True)
        path = os.path.join(dir_to_use, 'ground_truth.csv')

        try:
            with open(path, mode='w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(['Timestamp', 'Label', 'is_stopping'])
                writer.writerows(self.csv_rows)
            self.get_logger().info(f"CSV を保存しました: {path}")
        except Exception as e:
            self.get_logger().error(f"CSV保存中にエラー発生: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = LabelCollectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()

if __name__ == '__main__':
    main()