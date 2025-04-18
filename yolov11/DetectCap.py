from ultralytics import YOLO
import cv2
import torch
import time

# 选择 GPU（如果可用）
device = "cuda" if torch.cuda.is_available() else "cpu"
print(f"Using device: {device}")

# 加载 YOLO 模型
model = YOLO("./runs/detect/train2/weights/best.pt").to(device)

# 获取类别名称（如果为空，需手动指定）
class_names = model.names if model.names else {0: "Class_0", 1: "Class_1", 2: "Class_2"}  

# 打开摄像头
cap = cv2.VideoCapture(1)  # 如果 0 不行，尝试 1 或 2

# 获取摄像头分辨率
ret, frame = cap.read()
if ret:
    height, width, channels = frame.shape
    print(f"Camera Resolution: {width}x{height}, Channels: {channels}")
else:
    print("Error: Could not read frame from camera.")
    cap.release()
    exit()

confidence_threshold = 0.5  # 置信度阈值

# 计算 FPS 相关变量
prev_time = time.time()

while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        break

    # 计算 FPS
    current_time = time.time()
    fps = 1.0 / (current_time - prev_time)
    prev_time = current_time

    # YOLO 预测
    results = model.predict(frame, device=device)

    for r in results:
        for box in r.boxes.data:  
            x1, y1, x2, y2, conf, cls = map(float, box[:6])  
            if conf < confidence_threshold:
                continue  

            cls = int(cls)
            class_name = class_names.get(cls, f"ID {cls}")  

            # 绘制检测框
            cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
            cv2.putText(frame, f"{class_name} {conf:.2f}", (int(x1), int(y1) - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

    # 在图像上显示 FPS 和 分辨率
    cv2.putText(frame, f"FPS: {fps:.2f}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
    cv2.putText(frame, f"Resolution: {width}x{height}", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

    # 显示结果
    cv2.imshow("YOLOv11 Live Detection", frame)
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
