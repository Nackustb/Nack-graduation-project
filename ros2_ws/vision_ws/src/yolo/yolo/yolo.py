import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import torch
from ultralytics import YOLO
import time
import numpy as np

class YOLOv11ROS2(Node):
    def __init__(self):
        super().__init__('yolov11_detector')

        # 选择 GPU（如果可用）
        self.device = "cuda" if torch.cuda.is_available() else "cpu"
        self.get_logger().info(f"Using device: {self.device}")

        # 加载 YOLO 模型
        self.model = YOLO("/home/nack/vision_ws/src/yolo_ros/weights/best.pt").to(self.device)

        # 获取类别名称
        self.class_names = self.model.names if self.model.names else {0: "Class_0", 1: "Class_1", 2: "Class_2"}

        # OpenCV 转 ROS2
        self.bridge = CvBridge()

        # 订阅摄像头话题
        self.subscription = self.create_subscription(
            Image,
            '/camera/color/image_raw',  # 修改为你的摄像头话题
            self.image_callback,
            10)

        # 发布检测结果
        self.publisher = self.create_publisher(Image, '/test', 10)
        self.text_publisher = self.create_publisher(String, '/test_text', 10)

        self.get_logger().info("YOLOv11 ROS2 Node Started!")

    def image_callback(self, msg):
        """ 处理摄像头图像并进行目标检测 """
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

        # 记录时间计算 FPS
        start_time = time.time()

        # YOLO 预测
        results = self.model.predict(cv_image, device=self.device)

        # 解析检测结果
        detection_texts = []
        for r in results:
            for box in r.boxes.data:
                x1, y1, x2, y2, conf, cls = map(float, box[:6])
                if conf < 0.5:  # 置信度阈值
                    continue
                
                cls = int(cls)
                class_name = self.class_names.get(cls, f"ID {cls}")

                # 记录检测信息
                detection_texts.append(f"{class_name} {conf:.2f} at [{int(x1)}, {int(y1)}, {int(x2)}, {int(y2)}]")

                # 绘制检测框
                cv2.rectangle(cv_image, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
                cv2.putText(cv_image, f"{class_name} {conf:.2f}", (int(x1), int(y1) - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        # 计算 FPS
        fps = 1.0 / (time.time() - start_time)
        cv2.putText(cv_image, f"FPS: {fps:.2f}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

        # 发布检测结果图像
        result_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")
        self.publisher.publish(result_msg)

        # 发布文本检测信息
        if detection_texts:
            text_msg = String()
            text_msg.data = "; ".join(detection_texts)
            self.text_publisher.publish(text_msg)

def main(args=None):
    rclpy.init(args=args)
    node = YOLOv11ROS2()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
