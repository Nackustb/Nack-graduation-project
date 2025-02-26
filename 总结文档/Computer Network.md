# TCP与UDP使用教程

## 一、网络编程主要函数介绍

### 1. `socket` 函数

`socket`函数用于创建一个新的套接字。它的基本格式如下：

```
socket.socket(family, type)
socket.socket(family, type, proto=0)
```

- **family**：指定套接字的地址族，`AF_INET`表示IPv4，`AF_INET6`表示IPv6。
- **type**：指定套接字的类型，`SOCK_STREAM`表示TCP，`SOCK_DGRAM`表示UDP。
- **proto**：协议类型，通常使用0，表示默认协议。

### 2. `bind` 函数

`bind`函数用于将套接字与本地的IP地址和端口号进行绑定。格式如下：

```
socket.bind(address)
```

- **address**：一个元组，包含IP地址和端口号，例如`('127.0.0.1', 5000)`。

### 3. `listen` 函数

`listen`函数用于使套接字进入监听模式，等待客户端的连接请求。格式如下：

```
socket.listen(backlog)
```

- **backlog**：最大连接数，表示操作系统允许的未处理连接的最大数量。

### 4. `accept` 函数

`accept`函数用于接受客户端的连接请求。格式如下：

```
socket.accept()
```

- 返回值：一个二元组 `(client_socket, client_address)`，`client_socket`是一个新的套接字对象，用于与客户端通信；`client_address`是客户端的地址。

### 5. `connect` 函数

`connect`函数用于客户端连接到服务器。格式如下：

```
socket.connect(address)
```

- **address**：服务器的IP地址和端口号。

### 6. `send` 函数

`send`函数用于通过已连接的套接字发送数据。格式如下：

```
socket.send(data)
```

- **data**：要发送的数据，通常是字节类型。

### 7. `recv` 函数

`recv`函数用于从已连接的套接字接收数据。格式如下：

```
socket.recv(bufsize)
```

- **bufsize**：要接收的最大字节数。

### 8. `recvfrom` 函数

`recvfrom`函数用于从UDP套接字接收数据并返回数据的发送者地址。格式如下：

```
socket.recvfrom(bufsize)
```

- **bufsize**：要接收的最大字节数。
- 返回值：一个二元组 `(data, address)`，`data`是接收到的数据，`address`是发送者的地址。

### 9. `sendto` 函数

`sendto`函数用于通过UDP套接字发送数据。格式如下：

```
socket.sendto(data, address)
```

- **data**：要发送的数据，通常是字节类型。
- **address**：目标地址，包含IP地址和端口号。

## 二、TCP 和 UDP 原理上的区别

| 特性         | TCP                              | UDP                          |
| ------------ | -------------------------------- | ---------------------------- |
| 连接方式     | 面向连接，客户端与服务器建立连接 | 无连接，数据报文独立发送     |
| 数据传输保证 | 保证数据可靠传输，无丢包、无乱序 | 不保证数据可靠性，可能丢包   |
| 数据传输顺序 | 保证数据顺序                     | 不保证数据顺序               |
| 传输速度     | 较慢，因需要建立连接和确认机制   | 较快，直接发送，不需确认     |
| 适用场景     | 需要高可靠性的应用，如文件传输   | 实时性要求高的应用，如视频流 |

## 三、TCP 编程

### 1. 服务端代码

```
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
```

### 2. 客户端代码

```
import socket

# 创建TCP套接字
client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

# 连接到服务器
client_socket.connect(('127.0.0.1', 5000))

# 发送数据
client_socket.send(b"Hello, Server!")

# 接收响应
data = client_socket.recv(1024)
print(f"收到响应：{data.decode()}")

# 关闭连接
client_socket.close()
```

## 四、UDP 编程

### 1. 服务端代码

```
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
```

```
import cv2
import socket
import numpy as np

# 设置服务器端IP和端口
UDP_IP = "127.0.0.1"  # 本地地址
UDP_PORT = 5005  # 端口号

# 创建UDP套接字
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))

print(f"服务器已启动，等待接收数据...")

while True:
    # 接收数据
    data, addr = sock.recvfrom(65536)  # 这里的缓冲区大小是65536字节
    # 将数据解码成图像
    nparr = np.frombuffer(data, np.uint8)
    frame = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
    
    if frame is not None:
        cv2.imshow('Received Video Frame', frame)

    # 按 'q' 键退出
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# 释放资源
sock.close()
cv2.destroyAllWindows()

```

### 2. 客户端代码

```
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
```

```
import cv2
import socket
import numpy as np

# 设置客户端的IP和端口
UDP_IP = "127.0.0.1"  # 目标服务器地址
UDP_PORT = 5005  # 端口号

# 创建UDP套接字
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# 打开视频文件
cap = cv2.VideoCapture('video.mp4')  # 替换为你的视频文件路径

while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        break

    # 将每帧图像编码成JPEG格式
    _, encoded_frame = cv2.imencode('.jpg', frame)
    
    # 将图像数据通过UDP发送给服务器
    sock.sendto(encoded_frame.tobytes(), (UDP_IP, UDP_PORT))

    # 显示发送的帧
    cv2.imshow('Sending Video Frame', frame)
    
    # 按 'q' 键退出
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# 释放资源
cap.release()
sock.close()
cv2.destroyAllWindows()
```



## 总结

- **TCP编程**：通过`socket()`创建套接字，`bind()`绑定IP和端口，`listen()`监听端口，`accept()`接受客户端连接，`recv()`和`send()`用于接收和发送数据。
- **UDP编程**：通过`socket()`创建套接字，`bind()`绑定端口，`recvfrom()`和`sendto()`用于接收和发送数据。
- **TCP和UDP的区别**：TCP是面向连接的协议，保证数据可靠传输；UDP是无连接的协议，速度快但不保证可靠性。