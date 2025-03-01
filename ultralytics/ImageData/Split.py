import os
import shutil
import random

def split_data(file_path, label_path, new_file_path, train_rate, val_rate, test_rate):
    # 获取当前脚本所在目录
    current_directory = os.path.dirname(__file__)

    # 构建输入和输出文件夹的完整路径
    input_folder = os.path.join(current_directory, file_path)
    label_folder = os.path.join(current_directory, label_path)
    output_folder = os.path.join(current_directory, new_file_path)

    # 获取图片和标签文件列表
    images = os.listdir(input_folder)
    labels = os.listdir(label_folder)
    images_no_ext = {os.path.splitext(image)[0]: image for image in images}
    labels_no_ext = {os.path.splitext(label)[0]: label for label in labels}
    matched_data = [(img, images_no_ext[img], labels_no_ext[img]) for img in images_no_ext if img in labels_no_ext]

    # 检查未匹配的图片和标签文件
    unmatched_images = [img for img in images_no_ext if img not in labels_no_ext]
    unmatched_labels = [label for label in labels_no_ext if label not in images_no_ext]
    if unmatched_images:
        print("未匹配的图片文件:")
        for img in unmatched_images:
            print(images_no_ext[img])
    if unmatched_labels:
        print("未匹配的标签文件:")
        for label in unmatched_labels:
            print(labels_no_ext[label])

    # 打乱数据
    random.shuffle(matched_data)
    total = len(matched_data)
    train_data = matched_data[:int(train_rate * total)]
    val_data = matched_data[int(train_rate * total):int((train_rate + val_rate) * total)]
    test_data = matched_data[int((train_rate + val_rate) * total):]

    # 处理训练集
    for img_name, img_file, label_file in train_data:
        old_img_path = os.path.join(input_folder, img_file)
        old_label_path = os.path.join(label_folder, label_file)
        new_img_dir = os.path.join(output_folder, 'train', 'images')
        new_label_dir = os.path.join(output_folder, 'train', 'labels')
        os.makedirs(new_img_dir, exist_ok=True)
        os.makedirs(new_label_dir, exist_ok=True)
        shutil.copy(old_img_path, os.path.join(new_img_dir, img_file))
        shutil.copy(old_label_path, os.path.join(new_label_dir, label_file))

    # 处理验证集
    for img_name, img_file, label_file in val_data:
        old_img_path = os.path.join(input_folder, img_file)
        old_label_path = os.path.join(label_folder, label_file)
        new_img_dir = os.path.join(output_folder, 'val', 'images')
        new_label_dir = os.path.join(output_folder, 'val', 'labels')
        os.makedirs(new_img_dir, exist_ok=True)
        os.makedirs(new_label_dir, exist_ok=True)
        shutil.copy(old_img_path, os.path.join(new_img_dir, img_file))
        shutil.copy(old_label_path, os.path.join(new_label_dir, label_file))

    # 处理测试集
    for img_name, img_file, label_file in test_data:
        old_img_path = os.path.join(input_folder, img_file)
        old_label_path = os.path.join(label_folder, label_file)
        new_img_dir = os.path.join(output_folder, 'test', 'images')
        new_label_dir = os.path.join(output_folder, 'test', 'labels')
        os.makedirs(new_img_dir, exist_ok=True)
        os.makedirs(new_label_dir, exist_ok=True)
        shutil.copy(old_img_path, os.path.join(new_img_dir, img_file))
        shutil.copy(old_label_path, os.path.join(new_label_dir, label_file))

    print("数据集已划分完成")

if __name__ == '__main__':
    file_path = "AugmentedImages"  # 图片文件夹
    label_path = 'labels'  # 标签文件夹
    new_file_path = "VOCdevkit"  # 新数据存放位置
    split_data(file_path, label_path, new_file_path, train_rate=0.8, val_rate=0.1, test_rate=0.1)
