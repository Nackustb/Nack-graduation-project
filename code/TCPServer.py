import socket


# 创建TCP套接字
server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

# 绑定IP和端口
server_socket.bind(('127.0.0.1', 5000))

# 开始监听，最大连接数为5
server_socket.listen(5)
print("服务器已启动，等待连接...")

while True:
    # 接受客户端连接
    client_socket, client_address = server_socket.accept()
    print(f"连接来自 {client_address}")

    # 接收数据
    data = client_socket.recv(1024)
    print(f"收到数据：{data.decode()}")

    # 发送响应
    client_socket.send(b"Hello, Client!")

    # 关闭连接
    client_socket.close()