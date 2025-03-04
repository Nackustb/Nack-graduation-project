import pyrealsense2 as rs
import cv2
import numpy as np
import time

def main():
    # 创建管道和配置
    pipeline = rs.pipeline()
    config = rs.config()
    
    # 启用彩色流，这里设置为1280x720，30 FPS
    # 如果设备支持1920x1080，可修改为：config.enable_stream(rs.stream.color, 1920, 1080, rs.format.bgr8, 30)
    config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 15)
    
    try:
        # 启动 RealSense 管道
        pipeline.start(config)
    except Exception as e:
        print("启动 RealSense 管道时出错:", e)
        return

    # 设置视频写入器，保存为 MP4 格式
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    out = cv2.VideoWriter('output.mp4', fourcc, 30.0, (1280, 720))
    
    # 设置录制时长（单位：秒）
    duration = 30
    start_time = time.time()
    
    try:
        while time.time() - start_time < duration:
            # 等待一帧数据
            frames = pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            if not color_frame:
                continue
            
            # 将帧数据转换为 NumPy 数组
            color_image = np.asanyarray(color_frame.get_data())
            
            # 将帧写入视频文件
            out.write(color_image)
            
            # 显示实时画面
            cv2.imshow('RealSense', color_image)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    finally:
        # 释放资源
        pipeline.stop()
        out.release()
        cv2.destroyAllWindows()
    
    print("视频录制完成，已保存为 output.mp4")

if __name__ == '__main__':
    main()
