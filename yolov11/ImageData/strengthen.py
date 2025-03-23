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

def save_annotation(objects, image_size, annotation_path):
    """保存VOC格式的XML标签"""
    width, height = image_size
    root = ET.Element('annotation')

    size_elem = ET.SubElement(root, 'size')
    ET.SubElement(size_elem, 'width').text = str(width)
    ET.SubElement(size_elem, 'height').text = str(height)
    ET.SubElement(size_elem, 'depth').text = '3'

    for obj in objects:
        obj_elem = ET.SubElement(root, 'object')
        ET.SubElement(obj_elem, 'name').text = obj[0]
        bndbox_elem = ET.SubElement(obj_elem, 'bndbox')
        ET.SubElement(bndbox_elem, 'xmin').text = str(max(0, min(width, obj[1])))
        ET.SubElement(bndbox_elem, 'ymin').text = str(max(0, min(height, obj[2])))
        ET.SubElement(bndbox_elem, 'xmax').text = str(max(0, min(width, obj[3])))
        ET.SubElement(bndbox_elem, 'ymax').text = str(max(0, min(height, obj[4])))

    tree = ET.ElementTree(root)
    tree.write(annotation_path)

def augment_image(image, objects, seq, output_image_folder, output_annotation_folder, image_name, annotation_name, augmentation_count):
    """对图片和标签进行数据增强"""
    width, height = image.size

    # 保存原始图片和标签
    original_image_path = os.path.join(output_image_folder, image_name)
    original_annotation_path = os.path.join(output_annotation_folder, annotation_name)
    save_image(image, original_image_path)
    save_annotation(objects, (width, height), original_annotation_path)

    image_np = np.array(image)

    bbs = BoundingBoxesOnImage([
        BoundingBox(x1=obj[1], y1=obj[2], x2=obj[3], y2=obj[4]) for obj in objects
    ], shape=image_np.shape)

    for i in range(augmentation_count):
        image_aug, bbs_aug = seq(image=image_np, bounding_boxes=bbs)

        # 修正边界框，防止超出边界
        bbs_aug = bbs_aug.clip_out_of_image()

        # 过滤掉无效边界框（宽高小于1像素的框）
        valid_bbs = [bb for bb in bbs_aug if bb.width > 1 and bb.height > 1]
        if not valid_bbs:
            continue  # 如果没有有效框，则跳过这个增强版本
        
        augmented_objects = [(objects[0][0], int(bb.x1), int(bb.y1), int(bb.x2), int(bb.y2)) for bb in valid_bbs]
        
        augmented_image = Image.fromarray(image_aug)

        augmented_image_name = f"{image_name.replace('.jpg', '')}_aug_{i+1}.jpg"
        augmented_annotation_name = f"{annotation_name.replace('.xml', '')}_aug_{i+1}.xml"
        augmented_image_path = os.path.join(output_image_folder, augmented_image_name)
        augmented_annotation_path = os.path.join(output_annotation_folder, augmented_annotation_name)

        save_image(augmented_image, augmented_image_path)
        save_annotation(augmented_objects, augmented_image.size, augmented_annotation_path)

def process_image(args):
    """处理单张图片的函数"""
    try:
        image_path, annotation_path, output_image_folder, output_annotation_folder, augmentation_count = args
        image_name = os.path.basename(image_path)
        annotation_name = os.path.basename(annotation_path)

        image = read_image(image_path)
        objects = read_annotation(annotation_path)

        if not objects:
            print(f"⚠️ 跳过 {image_name}，没有检测到有效目标")
            return 0

        seq = iaa.Sequential([
            iaa.Fliplr(0.5),  
            iaa.Affine(rotate=(-15, 15), scale=(0.8, 1.2)),  
            iaa.Multiply((0.9, 1.1)),  
            iaa.GaussianBlur(sigma=(0, 1.0))  
        ])

        augment_image(image, objects, seq, output_image_folder, output_annotation_folder, image_name, annotation_name, augmentation_count)
        return augmentation_count + 1  
    except Exception as e:
        print(f"❌ 处理 {args[0]} 时发生错误: {e}")
        return 0

def main(augmentation_count=5):
    image_folder = 'JPEGImages'
    annotation_folder = 'Annotations'
    output_image_folder = 'AugmentedImages'
    output_annotation_folder = 'AugmentedAnnotations'
    
    os.makedirs(output_image_folder, exist_ok=True)
    os.makedirs(output_annotation_folder, exist_ok=True)

    image_files = [f for f in os.listdir(image_folder) if f.endswith('.jpg')]
    annotation_files = [f.replace('.jpg', '.xml') for f in image_files]

    args = [
        (os.path.join(image_folder, img), os.path.join(annotation_folder, ann), output_image_folder, output_annotation_folder, augmentation_count)
        for img, ann in zip(image_files, annotation_files)
    ]

    with Pool(processes=cpu_count()) as pool:
        results = list(tqdm(pool.imap(process_image, args), total=len(args), desc="数据增强进度"))

    total_generated = sum(results)
    print(f"✅ 处理完成，共生成 {total_generated} 张增强图像")

if __name__ == '__main__':
    main(augmentation_count=10)
