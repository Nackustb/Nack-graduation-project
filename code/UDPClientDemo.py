import socket
import struct

# 设置发送端的目标IP和端口
UDP_IP = "127.0.0.1"  # 目标地址
UDP_PORT = 5005

# 要发送的数字
number = 12345

# 将整数转换为字节数据
data = struct.pack('!I', number)

# 创建UDP套接字
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# 发送数据
sock.sendto(data, (UDP_IP, UDP_PORT))
print(f"已发送数字：{number}")
