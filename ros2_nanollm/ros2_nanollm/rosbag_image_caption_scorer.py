# SPDX-FileCopyrightText: Copyright (c) <year>
# SPDX-License-Identifier: Apache-2.0

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from nanollm_interfaces.msg import StringStamped
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
from nano_llm import NanoLLM, ChatHistory
import numpy as np
import time
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
import os
import csv
import cv2

def wrap_text(text, max_width, font_face, font_scale, thickness):
    """
    C++ の wrapText 処理に相当する機能を実装。
    長いテキストを空白区切りの単語単位でチェックし、max_width を超える場合に改行を入れた行のリストを返す。
    """
    lines = []
    words = text.split()
    current_line = ""

    for word in words:
        test_line = word if not current_line else (current_line + " " + word)
        (text_width, _), _ = cv2.getTextSize(test_line, font_face, font_scale, thickness)
        if text_width > max_width:
            if current_line:
                lines.append(current_line)
            current_line = word
        else:
            current_line = test_line

    if current_line:
        lines.append(current_line)
    return lines

class RosbagImageCaptionScorer(Node):
    def __init__(self):
        super().__init__('rosbag_image_caption_scorer')

        # パラメータ
        self.declare_parameter('model', "Efficient-Large-Model/VILA1.5-3b")
        self.declare_parameter('api', "mlc")
        self.declare_parameter('quantization', "q4f16_ft")
        self.declare_parameter('is_compressed', False)
        self.declare_parameter('query', "Please classify the location as private road, public road, or parking lot.")

        self.model_name = self.get_parameter('model').get_parameter_value().string_value
        self.query = self.get_parameter('query').get_parameter_value().string_value
        self.is_compressed = self.get_parameter('is_compressed').get_parameter_value().bool_value

        # ディレクトリパスを受け取るサブスクライバ
        self.save_dir = None
        self.dir_subscription = self.create_subscription(
            String,
            'save_dir',
            self.dir_callback,
            10
        )

        # クエリ受信
        self.query_subscription = self.create_subscription(
            String,
            'input_query',
            self.query_listener_callback,
            10
        )

        # 画像受信 (raw or compressed)
        if self.is_compressed:
            qos_profile = QoSProfile(
                depth=10,
                reliability=QoSReliabilityPolicy.BEST_EFFORT
            )
            self.image_subscription = self.create_subscription(
                CompressedImage,
                'input_image',
                self.compressedimage_listener_callback,
                qos_profile
            )
        else:
            self.image_subscription = self.create_subscription(
                Image,
                'input_image',
                self.image_listener_callback,
                10
            )

        self.cv_br = CvBridge()
        self.cv_img = None
        self.image_stamp = None

        # モデルのロード
        self.get_logger().info(f"load model: {self.model_name}")
        self.model = NanoLLM.from_pretrained(self.model_name)

        # チャット履歴用
        self.chat_history = ChatHistory(self.model)

        # Publisher
        self.output_publisher = self.create_publisher(StringStamped, '/output', 10)
        if self.is_compressed:
            self.image_publisher = self.create_publisher(Image, "/source_image", 10)

        timer_period = 0.001
        self.timer = self.create_timer(timer_period, self.nano_llm_inference)

    def dir_callback(self, msg: String):
        """画像やCSVの保存先ディレクトリを受け取る"""
        self.save_dir = msg.data
        self.get_logger().info(f"Set output directory to: {self.save_dir}")

    def query_listener_callback(self, msg: String):
        """クエリを受け取って更新"""
        self.get_logger().info(f"query_listener_callback: {msg.data}")
        self.query = msg.data

    def compressedimage_listener_callback(self, data: CompressedImage):
        """CompressedImage を受信して cv_img に変換"""
        self.image_stamp = data.header.stamp
        self.cv_img = self.cv_br.compressed_imgmsg_to_cv2(data, "bgr8")
        if self.is_compressed:
            img_msg = self.cv_br.cv2_to_imgmsg(self.cv_img, encoding="bgr8")
            img_msg.header.stamp = data.header.stamp
            img_msg.header.frame_id = "image"
            self.image_publisher.publish(img_msg)

    def image_listener_callback(self, data: Image):
        """Image を受信して cv_img に変換"""
        self.image_stamp = data.header.stamp
        in_bgr = self.cv_br.imgmsg_to_cv2(data, "bgr8")
        self.cv_img = in_bgr

    def nano_llm_inference(self):
        """
        - 画像があれば LLM で推論
        - 出力テキストを、元画像サイズをそのまま保持した状態で、下部に大きな黒枠を追加して描画し保存
        - CSV に結果を追記
        """
        if self.cv_img is None:
            return

        stamp = self.image_stamp
        prompt = self.query.strip("][()")

        # LLM内部がPIL画像を要求する場合：RGB変換
        cv_img_rgb = cv2.cvtColor(self.cv_img, cv2.COLOR_BGR2RGB)
        from PIL import Image as PILImage
        pil_img = PILImage.fromarray(cv_img_rgb)

        # 推論
        self.chat_history = ChatHistory(self.model)
        self.chat_history.append('user', prompt, use_cache=True)
        self.chat_history.append('user', image=pil_img)
        embedding, _ = self.chat_history.embed_chat()
        t0 = time.perf_counter_ns()
        output = self.model.generate(
            inputs=embedding,
            kv_cache=self.chat_history.kv_cache,
            min_new_tokens=1,
            max_new_tokens=20,
            streaming=False,
            do_sample=False,

        )
        t1 = time.perf_counter_ns()
        self.get_logger().info(f"LLM Prediction time: {(t1 - t0) / 1e9:.4f} sec")
        # 出力の Publish
        output_msg = StringStamped()
        output_msg.header.stamp = stamp
        output_msg.data = output
        self.output_publisher.publish(output_msg)
        self.get_logger().info(f"Published output: {output}")

        # ===== 画像下部にテキスト描画し保存 =====
        if self.save_dir:
            inference_img_dir = os.path.join(self.save_dir, "inference_images")
            os.makedirs(inference_img_dir, exist_ok=True)

            # 画像ファイル名用のタイムスタンプ（従来の sec_nanosec 形式）
            timestamp_filename = f"{stamp.sec}_{stamp.nanosec}"
            image_file_path = os.path.join(inference_img_dir, f"{timestamp_filename}.png")

            # CSV 用タイムスタンプ：秒単位の数値
            timestamp_csv = f"{stamp.sec + stamp.nanosec * 1e-9:.6f}"

            # 元画像サイズはそのまま保持し、下部に黒枠を追加（クロップはしない）
            height, width, _ = self.cv_img.shape
            # 黒枠の高さを元画像の1/4に設定
            black_bar_height = height // 4
            new_height = height + black_bar_height

            new_image = np.zeros((new_height, width, 3), dtype=np.uint8)
            new_image[:height, :width] = self.cv_img

            # テキスト描画用設定
            font_face = cv2.FONT_HERSHEY_SIMPLEX
            font_scale = 2.0  # 初期フォントサイズ
            thickness = 3
            color = (255, 255, 255)
            margin = 20
            max_text_width = width - (margin * 2)

            # 折り返し処理
            lines = wrap_text(output, max_text_width, font_face, font_scale, thickness)
            (sample_w, sample_h), sample_base = cv2.getTextSize("A", font_face, font_scale, thickness)
            line_height = sample_h + 8
            total_text_height = len(lines) * line_height

            # テキストが黒枠内に収まらなければ、フォントサイズを小さくして調整
            while total_text_height > black_bar_height and font_scale > 0.5:
                font_scale -= 0.1
                lines = wrap_text(output, max_text_width, font_face, font_scale, thickness)
                (sample_w, sample_h), sample_base = cv2.getTextSize("A", font_face, font_scale, thickness)
                line_height = sample_h + 8
                total_text_height = len(lines) * line_height

            # 黒枠内にテキストを垂直中央に配置する開始Y座標の計算
            start_y = height + (black_bar_height - total_text_height) // 2 + line_height

            for line in lines:
                (text_w, text_h), base = cv2.getTextSize(line, font_face, font_scale, thickness)
                text_x = max((width - text_w) // 2, 0)
                cv2.putText(new_image, line, (text_x, start_y), font_face, font_scale, color, thickness, cv2.LINE_AA)
                start_y += line_height

            # 画像保存
            cv2.imwrite(image_file_path, new_image)
            self.get_logger().info(f"Saved inference image: {image_file_path}")

            # CSVにスコアを追記
            score = self.calculate_score(output)
            csv_path = os.path.join(self.save_dir, "inference_result.csv")
            write_header = not os.path.exists(csv_path)
            with open(csv_path, 'a', newline='', encoding='utf-8') as f:
                writer = csv.writer(f)
                if write_header:
                    writer.writerow(["Timestamp", "Result", "filepath","output"])
                writer.writerow([timestamp_csv, score, image_file_path, output])
            self.get_logger().info(f"Appended score {score} to {csv_path}")

        # 推論完了後リセット
        self.chat_history.reset()
        self.cv_img = None

    def calculate_score(self, output: str) -> int:
        """
        LLM 出力の文字列に応じてスコアを付与する。
          1) "parking lot" が含まれる場合 → 2
          2) "private" と "public" の両方含む場合 → 0
          3) "private" のみ → -1
          4) "public" のみ → +1
          5) 上記いずれにも該当しなければ -3
        """
        out_lower = output.lower()
        # if "parking lot" in out_lower:
        #     return 2
        # has_private = "private" in out_lower
        # has_public = "public" in out_lower
        # if has_private and has_public:
        #     return 0
        # if has_private:
        #     return -1
        # if has_public:
        #     return 1
        # return -3
        has_private = "private" in out_lower or "parking lot" in out_lower
        has_public = "public" in out_lower or "intersection" in out_lower
        if has_private and has_public:
            return 0
        if has_private:
            return -1
        if has_public:
            return 1
        return -3

        

def main(args=None):
    rclpy.init(args=args)
    node = RosbagImageCaptionScorer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()