# 导入 Intel RealSense 库，用于访问摄像头的数据流
import pyrealsense2 as rs
# 导入 numpy 库，用于数值运算和数组操作
import numpy as np
# 导入 OpenCV 库，用于图像处理和显示
import cv2

# --------------------- RealSense 摄像头初始化配置 ---------------------

# 创建一个管道对象，用于管理摄像头的数据流
pipeline = rs.pipeline()

# 创建配置对象，用于设置所需的数据流类型、分辨率、格式和帧率
config = rs.config()

# 启用深度流：设置分辨率为 640x480，格式为 z16，帧率为 30fps
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
# 启用彩色流：设置分辨率为 640x480，格式为 bgr8（OpenCV 默认的颜色格式），帧率为 30fps
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# 启动管道，开始传输数据流，返回的 profile 包含当前摄像头的配置信息
profile = pipeline.start(config)

# --------------------- 获取深度比例（深度值单位转换因子） ---------------------

# 从设备配置中获取第一个深度传感器对象
depth_sensor = profile.get_device().first_depth_sensor()
# 获取深度比例，表示每个深度单位对应的实际距离（通常单位为米）
depth_scale = depth_sensor.get_depth_scale()
print("Depth Scale is: ", depth_scale)

# --------------------- 全局变量定义 ---------------------

# 存储用户鼠标点击时选择的图像像素坐标，初始为 None
selected_point = None
# 用于存储当前最新的一帧深度数据，初始为 None
depth_frame_global = None

# --------------------- 定义鼠标点击事件回调函数 ---------------------

def mouse_callback(event, x, y, flags, param):
    """
    鼠标事件回调函数
    当用户在图像窗口中点击左键时，获取该点的像素坐标和深度信息
    """
    global selected_point, depth_frame_global  # 使用全局变量保存点击的点和最新的深度帧
    # 如果检测到鼠标左键按下事件
    if event == cv2.EVENT_LBUTTONDOWN:
        # 保存点击的像素坐标（x, y）
        selected_point = (x, y)
        # 如果已经有深度数据帧，则获取该坐标点对应的深度信息
        if depth_frame_global is not None:
            # get_distance 方法返回该点的深度（单位：米）
            depth = depth_frame_global.get_distance(x, y)
            # 在控制台打印点击坐标和对应的深度值
            print("点击坐标：", selected_point, "深度：{:.2f} 米".format(depth))

# --------------------- 主程序循环 ---------------------

try:
    # 创建一个名为 'RealSense' 的窗口用于显示图像
    cv2.namedWindow('RealSense')
    # 设置该窗口的鼠标回调函数，当有鼠标事件发生时会调用 mouse_callback 函数
    cv2.setMouseCallback('RealSense', mouse_callback)
    
    # 无限循环，实时获取并显示摄像头数据
    while True:
        # 等待下一帧数据（阻塞等待）
        frames = pipeline.wait_for_frames()
        # 从帧中提取深度帧和彩色帧
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        # 如果无法获取到深度帧或彩色帧，则跳过当前循环
        if not depth_frame or not color_frame:
            continue
        
        # 将当前获取的深度帧保存到全局变量中，供鼠标回调函数使用
        depth_frame_global = depth_frame

        # 将深度帧和彩色帧数据转换为 numpy 数组，便于后续处理
        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())
        
        # 如果用户已在图像上点击，则在彩色图像上绘制标记和深度信息
        if selected_point is not None:
            # 在用户点击的坐标处绘制一个红色实心圆（半径为 5 像素）
            cv2.circle(color_image, selected_point, 5, (0, 0, 255), -1)
            # 获取点击位置的深度值（单位：米）
            depth_value = depth_frame.get_distance(selected_point[0], selected_point[1])
            # 格式化深度文本信息
            text = "Depth: {:.2f} m".format(depth_value)
            # 将深度信息文本显示在点击点右侧
            cv2.putText(color_image, text, (selected_point[0] + 10, selected_point[1]),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        
        # 显示带有标记和深度信息的彩色图像
        cv2.imshow('RealSense', color_image)
        # 按下 'q' 键时退出循环（cv2.waitKey(1) 返回的是按键的 ASCII 码）
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

# --------------------- 清理工作：关闭摄像头和窗口 ---------------------

finally:
    # 停止 RealSense 管道，释放摄像头资源
    pipeline.stop()
    # 关闭所有 OpenCV 创建的窗口
    cv2.destroyAllWindows()
