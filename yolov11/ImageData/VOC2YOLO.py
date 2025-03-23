import os
import xml.etree.ElementTree as ET
from multiprocessing import Pool, cpu_count
from tqdm import tqdm

# 定义类别顺序
categories = ['SMA']
category_to_index = {category: index for index, category in enumerate(categories)}

# 获取当前脚本所在目录
current_directory = os.path.dirname(__file__)

# 定义输入文件夹和输出文件夹的相对路径
input_folder = 'AugmentedAnnotations'  # 相对路径
output_folder = 'labels'  # 相对路径

# 构建完整的输入和输出文件夹路径
input_folder_path = os.path.join(current_directory, input_folder)
output_folder_path = os.path.join(current_directory, output_folder)

# 确保输出文件夹存在
os.makedirs(output_folder_path, exist_ok=True)

# 获取输入文件夹中的所有XML文件
xml_files = [f for f in os.listdir(input_folder_path) if f.endswith('.xml')]

# 处理 XML 的函数
def process_xml(filename):
    xml_path = os.path.join(input_folder_path, filename)
    
    # 解析 XML 文件
    tree = ET.parse(xml_path)
    root = tree.getroot()
    
    # 提取图像的尺寸
    size = root.find('size')
    width = int(size.find('width').text)
    height = int(size.find('height').text)

    objects = []  # 存储归一化坐标数据

    # 遍历 XML 中的 object 标签
    for obj in root.findall('object'):
        name = obj.find('name').text
        if name in category_to_index:
            category_index = category_to_index[name]
        else:
            continue  # 如果类别不在映射表中，跳过该 object

        bndbox = obj.find('bndbox')
        xmin = int(bndbox.find('xmin').text)
        ymin = int(bndbox.find('ymin').text)
        xmax = int(bndbox.find('xmax').text)
        ymax = int(bndbox.find('ymax').text)

        # 转换为中心点坐标和宽高
        x_center = (xmin + xmax) / 2.0 / width
        y_center = (ymin + ymax) / 2.0 / height
        w = (xmax - xmin) / width
        h = (ymax - ymin) / height

        objects.append(f"{category_index} {x_center:.6f} {y_center:.6f} {w:.6f} {h:.6f}")

    # 输出结果到对应的 TXT 文件
    txt_filename = os.path.splitext(filename)[0] + '.txt'
    txt_path = os.path.join(output_folder_path, txt_filename)
    with open(txt_path, 'w') as f:
        for obj in objects:
            f.write(obj + '\n')

    return filename  # 返回已处理的文件名，供 tqdm 统计进度

# 使用多进程 + tqdm 处理 XML
if __name__ == "__main__":
    num_workers = min(cpu_count(), 8)  # 限制最大进程数，防止 CPU 负载过高
    with Pool(processes=num_workers) as pool:
        # 使用 tqdm 显示进度条
        for _ in tqdm(pool.imap_unordered(process_xml, xml_files), total=len(xml_files), desc="Processing XML files", unit="file"):
            pass

    print("✅ 所有 XML 文件转换完成！")
