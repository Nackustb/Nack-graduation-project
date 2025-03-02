import cv2
import socket
import struct

UDP_IP = "127.0.0.1"  # 修改为接收端的实际 IP 地址
UDP_PORT = 5005
MAX_DGRAM = 60000  # 每个数据包最大字节数

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
cap = cv2.VideoCapture(1)

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # 降低图像质量以减小数据量（可根据需要调整）
    ret, buffer = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 50])
    if not ret:
        continue

    data = buffer.tobytes()
    # 计算分片数量
    num_fragments = (len(data) - 1) // MAX_DGRAM + 1

    # 先发送一个头部，告知接收端后续数据分片数量（4字节整数）
    header = struct.pack("I", num_fragments)
    sock.sendto(header, (UDP_IP, UDP_PORT))

    # 逐片发送数据，每个数据包前面加上4字节的片段索引
    for i in range(num_fragments):
        fragment = data[i * MAX_DGRAM : (i + 1) * MAX_DGRAM]
        frag_header = struct.pack("I", i)  # 片段索引
        sock.sendto(frag_header + fragment, (UDP_IP, UDP_PORT))
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
sock.close()
