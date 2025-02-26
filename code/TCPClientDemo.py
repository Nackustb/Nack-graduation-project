import socket

TCP_IP = '127.0.0.1'
TCP_PORT = 5000
# 创建TCP套接字
client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

# 连接到服务器
client_socket.connect((TCP_IP, TCP_PORT))

# 发送数据
client_socket.send(b'Hello, Server!')

# 接收响应
data = client_socket.recv(1024)
print(f"收到响应：{data.decode()}")

# 关闭连接
client_socket.close()