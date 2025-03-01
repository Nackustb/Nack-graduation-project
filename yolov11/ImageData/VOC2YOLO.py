import os
import xml.etree.ElementTree as ET

# 定义类别顺序
categories = ['SMA']
category_to_index = {category: index for index, category in enumerate(categories)}

# 获取当前脚本所在目录
current_directory = os.path.dirname(__file__)

# 定义输入文件夹和输出文件夹的相对路径
input_folder = 'AugmentedAnnotations'  # 相对路径
output_folder = 'labels'      # 相对路径

# 构建完整的输入和输出文件夹路径
input_folder_path = os.path.join(current_directory, input_folder)
output_folder_path = os.path.join(current_directory, output_folder)

# 确保输出文件夹存在
os.makedirs(output_folder_path, exist_ok=True)

# 获取输入文件夹中的所有XML文件
xml_files = [f for f in os.listdir(input_folder_path) if f.endswith('.xml')]
total_files = len(xml_files)

# 遍历输入文件夹中的所有XML文件
for idx, filename in enumerate(xml_files, start=1):
    xml_path = os.path.join(input_folder_path, filename)
    # 解析XML文件
    tree = ET.parse(xml_path)
    root = tree.getroot()
    # 提取图像的尺寸
    size = root.find('size')
    width = int(size.find('width').text)
    height = int(size.find('height').text)
    # 存储name和对应的归一化坐标
    objects = []

    # 遍历XML中的object标签
    for obj in root.findall('object'):
        name = obj.find('name').text
        if name in category_to_index:
            category_index = category_to_index[name]
        else:
            continue  # 如果name不在指定类别中，跳过该object

        bndbox = obj.find('bndbox')
        xmin = int(bndbox.find('xmin').text)
        ymin = int(bndbox.find('ymin').text)
        xmax = int(bndbox.find('xmax').text)
        ymax = int(bndbox.find('ymax').text)

        # 转换为中心点坐标和宽高
        x_center = (xmin + xmax) / 2.0
        y_center = (ymin + ymax) / 2.0
        w = xmax - xmin
        h = ymax - ymin

        # 归一化
        x = x_center / width
        y = y_center / height
        w = w / width
        h = h / height

        objects.append(f"{category_index} {x} {y} {w} {h}")

    # 输出结果到对应的TXT文件
    txt_filename = os.path.splitext(filename)[0] + '.txt'
    txt_path = os.path.join(output_folder_path, txt_filename)
    with open(txt_path, 'w') as f:
        for obj in objects:
            f.write(obj + '\n')

    # 输出处理进度
    print(f"已处理 {idx}/{total_files} 个文件: {filename}")

print("所有文件处理完成。")
