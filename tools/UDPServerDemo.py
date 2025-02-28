import socket
import struct

# 设置接收端的IP和端口
UDP_IP = "127.0.0.1"  # 本地地址
UDP_PORT = 5005

# 创建UDP套接字
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))

print(f"服务器已启动，等待接收数据...")

while True:
    # 接收数据
    data, addr = sock.recvfrom(4)  # 接收4字节数据（整数占4字节）
    number = struct.unpack('!I', data)[0]  # 将字节数据转换为整数
    print(f"接收到来自 {addr} 的数字：{number}")
