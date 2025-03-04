import cv2
import numpy as np
import pyrealsense2 as rs
from ultralytics import YOLO

# --------------------- 初始化 YOLO 模型 ---------------------
model = YOLO("./runs/detect/train3/weights/best.pt")  # 加载训练好的 YOLOv11 模型
class_names = model.names  # 获取类别名称

# --------------------- 初始化 RealSense 深度相机 ---------------------
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
profile = pipeline.start(config)

depth_sensor = profile.get_device().first_depth_sensor()
depth_scale = depth_sensor.get_depth_scale()
print("Depth Scale is:", depth_scale)

depth_frame_global = None  # 全局深度帧变量
selected_point = None  # 记录鼠标点击位置

# --------------------- 鼠标点击获取深度 ---------------------
def mouse_callback(event, x, y, flags, param):
    global selected_point, depth_frame_global
    if event == cv2.EVENT_LBUTTONDOWN:  # 左键点击
        selected_point = (x, y)
        if depth_frame_global is not None:
            depth = depth_frame_global.get_distance(x, y)
            print(f"点击坐标：{selected_point}, 深度：{depth:.2f} 米")

cv2.namedWindow("YOLO + RealSense")
cv2.setMouseCallback("YOLO + RealSense", mouse_callback)

confidence_threshold = 0.5  # 目标检测置信度阈值

# --------------------- 处理视频流 ---------------------
try:
    while True:
        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        if not depth_frame or not color_frame:
            continue
        
        depth_frame_global = depth_frame  # 存储最新深度帧
        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())

        # --------------------- YOLO 目标检测 ---------------------
        results = model(color_image)
        for r in results:
            for box in r.boxes:
                conf = box.conf[0].item()
                if conf < confidence_threshold:
                    continue  # 过滤低置信度目标
                
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                cls = int(box.cls[0].item())
                class_name = class_names[cls] if cls in class_names else f"ID {cls}"
                
                # 计算检测框中心点
                cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
                depth_value = depth_frame.get_distance(cx, cy)  # 获取深度值
                
                # 绘制检测框和中心点深度信息
                cv2.rectangle(color_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.circle(color_image, (cx, cy), 5, (0, 0, 255), -1)
                text = f"{class_name} {conf:.2f} Depth: {depth_value:.2f}m"
                cv2.putText(color_image, text, (x1, y1 - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        
        # 显示鼠标点击点的深度信息
        # if selected_point is not None:
        #     depth_value = depth_frame.get_distance(selected_point[0], selected_point[1])
        #     cv2.circle(color_image, selected_point, 5, (0, 0, 255), -1)
        #     text = f"Depth: {depth_value:.2f}m"
        #     cv2.putText(color_image, text, (selected_point[0] + 10, selected_point[1]),
        #                 cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        # 显示结果
        cv2.imshow("YOLO + RealSense", color_image)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    pipeline.stop()
    cv2.destroyAllWindows()
