import cv2
import os
import multiprocessing

def extract_frames_from_video(video_path, output_dir, frame_rate):
    cap = cv2.VideoCapture(video_path)
    if not cap.isOpened():
        print(f"无法打开视频文件: {video_path}")
        return

    fps = cap.get(cv2.CAP_PROP_FPS)
    if fps == 0:
        print(f"无法获取视频帧率: {video_path}")
        cap.release()
        return

    frame_interval = int(fps / frame_rate)
    print(f"每隔 {frame_interval} 帧保存一张图片")

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
            image_filename = os.path.join(output_dir, f"{os.path.basename(video_path)}_frame_{saved_count:04d}.jpg")
            cv2.imwrite(image_filename, frame)
            saved_count += 1
            print(f"已保存图片: {image_filename}")

    cap.release()
    print(f"已处理视频: {video_path}")
    print(f"共保存 {saved_count} 张图片到 {output_dir}")

def process_video(args):
    video_path, output_dir, frame_rate = args
    extract_frames_from_video(video_path, output_dir, frame_rate)

def extract_frames_from_directory(input_dir, output_dir, frame_rate):
    video_files = []
    for root, _, files in os.walk(input_dir):
        for file in files:
            if file.lower().endswith(('.mp4', '.avi', '.mov', '.mkv')):
                video_path = os.path.join(root, file)
                video_files.append((video_path, output_dir, frame_rate))
                print(f"发现视频文件: {video_path}")
    
    with multiprocessing.Pool(processes=multiprocessing.cpu_count()) as pool:
        pool.map(process_video, video_files)

if __name__ == "__main__":
    input_dir = r'C:\Users\Nack\Desktop\Nack-graduation-project\yolov11\video'
    output_dir = r'C:\Users\Nack\Desktop\Nack-graduation-project\yolov11\JPEGImages'
    frame_rate = 2
    
    print(f"开始从目录 {input_dir} 中提取视频帧...")
    extract_frames_from_directory(input_dir, output_dir, frame_rate)
    print(f"所有视频帧提取完成，保存到目录 {output_dir}")