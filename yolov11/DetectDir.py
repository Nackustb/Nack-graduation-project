import os
import cv2
import torch
import xml.etree.ElementTree as ET
from concurrent.futures import ThreadPoolExecutor, as_completed
from ultralytics import YOLO  # 确保你已正确安装 YOLOv11
from tqdm import tqdm  # 进度条

# 设置文件夹
image_folder = "JPEGImages"  # 存放输入图片
output_label_folder = "Annotations"  # 存放 XML 标注
output_image_folder = "output_images"  # 存放带框图片

# 创建文件夹
os.makedirs(output_label_folder, exist_ok=True)
os.makedirs(output_image_folder, exist_ok=True)

# 加载 YOLOv11 模型（请替换为你的模型权重路径）
model = YOLO(r"C:\Users\Nack\Desktop\Nack-graduation-project\yolov11\runs\detect\train3\weights\best.pt")  # 你的权重文件

# 获取所有图片文件
image_files = [f for f in os.listdir(image_folder) if f.endswith(('.jpg', '.png', '.jpeg'))]

# 颜色映射（随机分配类别颜色）
colors = {i: (int(255 * torch.rand(1)), int(255 * torch.rand(1)), int(255 * torch.rand(1))) for i in range(80)}

# 预定义类别名称（按你训练的类别修改）
class_names = {
    0: "SMA",  # 你的类别名称映射
}

# Pascal VOC 格式 XML 生成函数
def create_voc_xml(filename, path, width, height, depth, detections, save_path):
    annotation = ET.Element("annotation")

    folder = ET.SubElement(annotation, "folder")
    folder.text = "JPEGImages"

    fname = ET.SubElement(annotation, "filename")
    fname.text = filename

    fpath = ET.SubElement(annotation, "path")
    fpath.text = os.path.abspath(path)  # 保存绝对路径

    source = ET.SubElement(annotation, "source")
    database = ET.SubElement(source, "database")
    database.text = "Unknown"

    size = ET.SubElement(annotation, "size")
    w = ET.SubElement(size, "width")
    w.text = str(width)
    h = ET.SubElement(size, "height")
    h.text = str(height)
    d = ET.SubElement(size, "depth")
    d.text = str(depth)

    segmented = ET.SubElement(annotation, "segmented")
    segmented.text = "0"

    # 添加目标框
    for obj in detections:
        obj_elem = ET.SubElement(annotation, "object")

        name = ET.SubElement(obj_elem, "name")
        name.text = obj["name"]  # 确保写入的是类别名称，而不是 "Class0"

        pose = ET.SubElement(obj_elem, "pose")
        pose.text = "Unspecified"

        truncated = ET.SubElement(obj_elem, "truncated")
        truncated.text = "0"

        difficult = ET.SubElement(obj_elem, "difficult")
        difficult.text = "0"

        bndbox = ET.SubElement(obj_elem, "bndbox")

        xmin = ET.SubElement(bndbox, "xmin")
        xmin.text = str(obj["xmin"])
        ymin = ET.SubElement(bndbox, "ymin")
        ymin.text = str(obj["ymin"])
        xmax = ET.SubElement(bndbox, "xmax")
        xmax.text = str(obj["xmax"])
        ymax = ET.SubElement(bndbox, "ymax")
        ymax.text = str(obj["ymax"])

    tree = ET.ElementTree(annotation)
    tree.write(save_path)

# 处理单张图片的函数
def process_image(image_file):
    image_path = os.path.join(image_folder, image_file)
    output_image_path = os.path.join(output_image_folder, image_file)
    xml_path = os.path.join(output_label_folder, f"{os.path.splitext(image_file)[0]}.xml")

    img = cv2.imread(image_path)
    if img is None:
        print(f"无法读取图片: {image_file}")
        return image_file  # 返回失败的文件名
    
    height, width, depth = img.shape  # 获取图像尺寸

    # 目标检测
    results = model(img)

    detections = []  # 存储检测结果用于 XML

    # 处理检测结果
    for result in results:
        for box in result.boxes:
            cls = int(box.cls)  # 类别索引
            x_min, y_min, x_max, y_max = map(int, box.xyxy[0])  # 获取边界框
            color = colors.get(cls, (0, 255, 0))  # 获取颜色

            # 获取类别名称（如果类别未定义，默认为 Unknown）
            object_name = class_names.get(cls, "Unknown")

            # 画框
            cv2.rectangle(img, (x_min, y_min), (x_max, y_max), color, 2)
            cv2.putText(img, object_name, (x_min, y_min - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

            # 记录 XML 信息
            detections.append({
                "name": object_name,  # 修正类别名称
                "xmin": x_min,
                "ymin": y_min,
                "xmax": x_max,
                "ymax": y_max
            })

    # 保存带框图片
    cv2.imwrite(output_image_path, img)

    # 生成 XML 文件
    create_voc_xml(image_file, image_path, width, height, depth, detections, xml_path)

    return None  # 处理成功

# 使用多线程加速 + tqdm 显示进度条
num_threads = min(8, len(image_files))  # 限制线程数

# tqdm 进度条
with ThreadPoolExecutor(max_workers=num_threads) as executor:
    futures = {executor.submit(process_image, img): img for img in image_files}

    with tqdm(total=len(image_files), desc="Processing Images", unit="img") as pbar:
        for future in as_completed(futures):
            result = future.result()
            if result:
                print(f"❌ 处理失败: {result}")  # 失败的文件
            pbar.update(1)

print("✅ 所有图片处理完成，带框图片和 XML 标注已保存！")
