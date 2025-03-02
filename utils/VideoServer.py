import cv2
import socket
import struct
import numpy as np
from ultralytics import YOLO

# 加载 YOLO 模型（请修改为你实际的权重路径）
model = YOLO("./yolov11/runs/detect/train/weights/best.pt")

UDP_IP = "127.0.0.1"  # 监听所有网卡
UDP_PORT = 5005

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))
print(f"监听 {UDP_IP}:{UDP_PORT} ...")

while True:
    try:
        # 首先接收4字节头部，获取分片数量
        header, addr = sock.recvfrom(4)
        num_fragments = struct.unpack("I", header)[0]

        # 接收所有分片，按索引存储
        fragments = [None] * num_fragments
        for i in range(num_fragments):
            frag_data, addr = sock.recvfrom(65507)
            frag_index = struct.unpack("I", frag_data[:4])[0]
            fragments[frag_index] = frag_data[4:]
        
        # 重组完整数据
        data = b"".join(fragments)
        np_data = np.frombuffer(data, dtype=np.uint8)
        frame = cv2.imdecode(np_data, cv2.IMREAD_COLOR)
        if frame is None:
            continue

        # 使用 YOLO 模型检测
        results = model(frame)
        for r in results:
            for box in r.boxes:
                conf = box.conf[0].item()
                if conf < 0.5:
                    continue
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                cls = int(box.cls[0].item())
                class_name = model.names[cls] if model.names and cls in model.names else str(cls)
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(frame, f"{class_name} {conf:.2f}", (x1, y1 - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        cv2.imshow("UDP YOLOv11 Detection", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    except Exception as e:
        print("发生错误：", e)

sock.close()
cv2.destroyAllWindows()
