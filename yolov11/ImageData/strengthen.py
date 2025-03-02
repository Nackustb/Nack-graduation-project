import os
import random
from PIL import Image
import xml.etree.ElementTree as ET
import numpy as np
import imgaug.augmenters as iaa
from imgaug.augmentables.bbs import BoundingBox, BoundingBoxesOnImage
from multiprocessing import Pool, cpu_count
from tqdm import tqdm

def read_image(image_path):
    """读取图片"""
    return Image.open(image_path)

def read_annotation(annotation_path):
    """读取VOC格式的XML标签"""
    tree = ET.parse(annotation_path)
    root = tree.getroot()
    objects = []
    for obj in root.findall('object'):
        name = obj.find('name').text
        bndbox = obj.find('bndbox')
        xmin = int(bndbox.find('xmin').text)
        ymin = int(bndbox.find('ymin').text)
        xmax = int(bndbox.find('xmax').text)
        ymax = int(bndbox.find('ymax').text)
        objects.append((name, xmin, ymin, xmax, ymax))
    return objects

def save_image(image, image_path):
    """保存图片"""
    image.save(image_path)

def save_annotation(objects, image_path, annotation_path):
    """保存VOC格式的XML标签，并加入图片尺寸信息"""
    # 获取图片尺寸
    image = Image.open(image_path)
    width, height = image.size
    depth = len(np.array(image).shape)  # 通常为3 (RGB)

    # 创建XML结构
    root = ET.Element('annotation')

    # 添加图片尺寸信息
    size_elem = ET.SubElement(root, 'size')
    width_elem = ET.SubElement(size_elem, 'width')
    width_elem.text = str(width)
    height_elem = ET.SubElement(size_elem, 'height')
    height_elem.text = str(height)
    depth_elem = ET.SubElement(size_elem, 'depth')
    depth_elem.text = str(depth)

    # 添加segmented信息
    segmented_elem = ET.SubElement(root, 'segmented')
    segmented_elem.text = '0'  # 假设未进行分割

    # 添加object信息
    for obj in objects:
        obj_elem = ET.SubElement(root, 'object')
        name_elem = ET.SubElement(obj_elem, 'name')
        name_elem.text = obj[0]
        bndbox_elem = ET.SubElement(obj_elem, 'bndbox')
        xmin_elem = ET.SubElement(bndbox_elem, 'xmin')
        xmin_elem.text = str(obj[1])
        ymin_elem = ET.SubElement(bndbox_elem, 'ymin')
        ymin_elem.text = str(obj[2])
        xmax_elem = ET.SubElement(bndbox_elem, 'xmax')
        xmax_elem.text = str(obj[3])
        ymax_elem = ET.SubElement(bndbox_elem, 'ymax')
        ymax_elem.text = str(obj[4])

    # 保存XML文件
    tree = ET.ElementTree(root)
    tree.write(annotation_path)

def augment_image(image, objects, seq, output_image_folder, output_annotation_folder, image_name, annotation_name, augmentation_count):
    """对图片和标签进行数据增强"""
    # 保存原始图片和标签
    original_image_path = os.path.join(output_image_folder, image_name)
    original_annotation_path = os.path.join(output_annotation_folder, annotation_name)
    save_image(image, original_image_path)
    save_annotation(objects, original_image_path, original_annotation_path)

    # 将图片转换为 numpy 数组
    image_np = np.array(image)
    image_height, image_width = image_np.shape[:2]  # 获取图像尺寸

    # 创建边界框对象
    bbs = BoundingBoxesOnImage([
        BoundingBox(x1=obj[1], y1=obj[2], x2=obj[3], y2=obj[4]) for obj in objects
    ], shape=image_np.shape)

    # 生成增强图像
    for i in range(augmentation_count):
        # 执行增强
        image_aug, bbs_aug = seq(image=image_np, bounding_boxes=bbs)

        # 确保增强后的边界框坐标不会超出图像范围
        augmented_objects = []
        for obj, bb in zip(objects, bbs_aug):
            x1 = max(0, min(int(bb.x1), image_width - 1))  # 限制 x1 在合法范围内
            y1 = max(0, min(int(bb.y1), image_height - 1)) # 限制 y1 在合法范围内
            x2 = max(0, min(int(bb.x2), image_width - 1))  # 限制 x2 在合法范围内
            y2 = max(0, min(int(bb.y2), image_height - 1)) # 限制 y2 在合法范围内
            
            # 确保边界框有效（即 x2 > x1 且 y2 > y1）
            if x2 > x1 and y2 > y1:
                augmented_objects.append((obj[0], x1, y1, x2, y2))

        # 仅在有效边界框存在时才保存增强后的图片
        if augmented_objects:
            # 将增强后的图片转换回 PIL 格式
            augmented_image = Image.fromarray(image_aug)
            # 生成新文件名
            augmented_image_name = f"{image_name.replace('.jpg', '')}_aug_{i+1}.jpg"
            augmented_annotation_name = f"{annotation_name.replace('.xml', '')}_aug_{i+1}.xml"
            augmented_image_path = os.path.join(output_image_folder, augmented_image_name)
            augmented_annotation_path = os.path.join(output_annotation_folder, augmented_annotation_name)
            # 保存增强后的图片和标签
            save_image(augmented_image, augmented_image_path)
            save_annotation(augmented_objects, augmented_image_path, augmented_annotation_path)


def process_image(args):
    """处理单张图片的函数"""
    image_path, annotation_path, output_image_folder, output_annotation_folder, augmentation_count = args
    image_name = os.path.basename(image_path)
    annotation_name = os.path.basename(annotation_path)
    # 读取图片和标签
    image = read_image(image_path)
    objects = read_annotation(annotation_path)
    # 定义增强序列
    seq = iaa.Sequential([
        iaa.Fliplr(0.5),  # 50% 概率水平翻转
        iaa.Affine(rotate=(-30, 30)),  # 随机旋转 -30 到 30 度
        iaa.Multiply((0.8, 1.2)),  # 随机亮度调整
    ])
    augment_image(image, objects, seq, output_image_folder, output_annotation_folder, image_name, annotation_name, augmentation_count)
    return augmentation_count + 1  # 返回生成的图片数量（原始图片 + 增强图片）

def main(augmentation_count=5):
    image_folder = 'JPEGImages'
    annotation_folder = 'Annotations'
    output_image_folder = 'AugmentedImages'
    output_annotation_folder = 'AugmentedAnnotations'
    os.makedirs(output_image_folder, exist_ok=True)
    os.makedirs(output_annotation_folder, exist_ok=True)

    image_files = [f for f in os.listdir(image_folder) if f.endswith('.jpg')]
    annotation_files = [f.replace('.jpg', '.xml') for f in image_files]
    total_images = len(image_files)
    total_generated_images = total_images * (augmentation_count + 1)  # 原始图片 + 增强图片

    # 准备多进程参数
    args = [
        (os.path.join(image_folder, image_name), os.path.join(annotation_folder, annotation_name), output_image_folder, output_annotation_folder, augmentation_count)
        for image_name, annotation_name in zip(image_files, annotation_files)
    ]

    # 使用多进程处理
    with Pool(processes=cpu_count()) as pool:
        results = list(tqdm(pool.imap(process_image, args), total=len(args), desc="处理进度"))

    print(f"处理完成，共生成 {total_generated_images} 张图片。")

if __name__ == '__main__':
    main(augmentation_count=10)  # 设置每张图片生成 10 张增强图像
