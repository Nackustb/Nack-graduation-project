import cv2
import os

def extract_frames_from_video(video_path, output_dir, frame_rate):
    # 打开视频文件
    cap = cv2.VideoCapture(video_path)
    if not cap.isOpened():
        print(f"无法打开视频文件: {video_path}")
        return

    # 获取视频的帧率
    fps = cap.get(cv2.CAP_PROP_FPS)
    if fps == 0:
        print(f"无法获取视频帧率: {video_path}")
        cap.release()
        return

    # 计算每隔多少帧保存一张图片
    frame_interval = int(fps / frame_rate)
    print(f"每隔 {frame_interval} 帧保存一张图片")

    # 创建输出目录（如果不存在）
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)
        print(f"已创建输出目录: {output_dir}")

    frame_count = 0
    saved_count = 0
    while True:
        ret, frame = cap.read()
        if not ret:
            break

        frame_count += 1
        if frame_count % frame_interval == 0:
            # 生成图片文件名
            image_filename = os.path.join(output_dir, f"{os.path.basename(video_path)}_frame_{saved_count:04d}.jpg")
            # 保存图片
            cv2.imwrite(image_filename, frame)
            saved_count += 1
            print(f"已保存图片: {image_filename}")

    cap.release()
    print(f"已处理视频: {video_path}")
    print(f"共保存 {saved_count} 张图片到 {output_dir}")

def extract_frames_from_directory(input_dir, output_dir, frame_rate):
    # 获取指定目录下的所有文件
    for root, dirs, files in os.walk(input_dir):
        for file in files:
            # 检查文件扩展名是否为视频格式
            if file.lower().endswith(('.mp4', '.avi', '.mov', '.mkv')):
                video_path = os.path.join(root, file)
                print(f"开始处理视频文件: {video_path}")
                extract_frames_from_video(video_path, output_dir, frame_rate)
                print(f"完成处理视频文件: {video_path}")

if __name__ == "__main__":
    input_dir = r'C:\Users\Nack\Desktop\Graduation-Design-USTB\ultralytics\video'  # 替换为您的视频文件夹路径
    output_dir = r'C:\Users\Nack\Desktop\Graduation-Design-USTB\ultralytics\ImageData\JPEGImages'  # 替换为您希望保存图片的目录
    frame_rate = 1  # 替换为您希望的帧率（每秒保存的图片数量）

    print(f"开始从目录 {input_dir} 中提取视频帧...")
    extract_frames_from_directory(input_dir, output_dir, frame_rate)
    print(f"所有视频帧提取完成，保存到目录 {output_dir}")
