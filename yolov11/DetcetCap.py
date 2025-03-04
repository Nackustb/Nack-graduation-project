from ultralytics import YOLO
import cv2

# 加载模型
model = YOLO("./runs/detect/train3/weights/best.pt")  

# 读取类别名称（如果你的数据集有 classes.txt 或你知道类别）
class_names = model.names  # 直接从 YOLO 模型获取类别名称

cap = cv2.VideoCapture(1)  # 0 或 1 选择摄像头
confidence_threshold = 0.5  # 设置置信度阈值

while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        break

    results = model(frame)
    for r in results:
        for box in r.boxes:
            conf = box.conf[0].item()  # 置信度
            if conf < confidence_threshold:
                continue  # 过滤低置信度目标

            x1, y1, x2, y2 = map(int, box.xyxy[0])  
            cls = int(box.cls[0].item())  # 获取类别索引
            class_name = class_names[cls] if cls in class_names else f"ID {cls}"  # 获取类别名称

            # 绘制检测框
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(frame, f"{class_name} {conf:.2f}", (x1, y1 - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

    cv2.imshow("YOLOv11 Live Detection", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()