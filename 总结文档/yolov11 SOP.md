## prepare your dataset	

#### step 1 

```
put your video in the video dir
run ExtractFrames.py
```

#### step 2

```
conda activate LabelImg 
LabelImg with VOC
```

#### Step 3

```
cd C:\Users\Nack\Desktop\Graduation-Design-USTB\yolov11\ImageData
run strengthen.py
```

#### Step 4

```
run VOC2YOLO.py 
Notice!
Change the categories
```

#### Step 5

```
run Split.py
```

## train

```
change the train.yaml
yolo task=detect mode=train model=yolo11n.yaml pretrained=yolo11n.pt data=train.yaml epochs=200 imgsz=640 device=0 optimizer='SGD' workers=8 batch=64 amp=False cache=False
```

## detect

```
yolo predict model=runs/detect/train/weights/best.pt source='test1.jpg'
yolo predict model=runs/detect/train/weights/best.pt source=0
```

## Resume

```
yolo train model=runs/detect/train3/weights/last.pt resume=True
```

## Refer

```
https://blog.csdn.net/qq_67105081/article/details/143402823?fromshare=blogdetail&sharetype=blogdetail&sharerId=143402823&sharerefer=PC&sharesource=m0_64430023&sharefrom=from_link
```



## 参数说明



| 参数            | 值                                     | 说明                         |
| --------------- | -------------------------------------- | ---------------------------- |
| task            | detect                                 | 指定任务类型为目标检测       |
| mode            | train                                  | 设置模式为训练               |
| model           | ultralytics/cfg/models/11/yolo11n.yaml | 指定模型配置文件路径         |
| data            | data.yaml                              | 指定数据配置文件路径         |
| epochs          | 100                                    | 训练轮数                     |
| time            | None                                   | 不设置训练时间限制           |
| patience        | 50                                     | 早停（early stopping）耐心值 |
| batch           | 8                                      | 每个批次大小                 |
| imgsz           | 160                                    | 输入图像尺寸                 |
| save            | True                                   | 启用模型保存                 |
| save_period     | -1                                     | 每个epoch都保存              |
| cache           | disk                                   | 设置磁盘缓存                 |
| device          | 0                                      | 指定使用第一个GPU            |
| workers         | 8                                      | 数据加载的工作线程数         |
| project         | runs/train                             | 项目目录                     |
| name            | exp4                                   | 实验名称                     |
| exist_ok        | False                                  | 目录存在时不覆盖             |
| pretrained      | True                                   | 使用预训练权重               |
| optimizer       | SGD                                    | 选择优化器                   |
| verbose         | True                                   | 启用详细日志                 |
| seed            | 0                                      | 设置随机种子                 |
| deterministic   | False                                  | 训练过程是否确定性           |
| single_cls      | False                                  | 是否只训练一个类别           |
| rect            | False                                  | 是否使用矩形训练             |
| cos_lr          | False                                  | 余弦学习率调度               |
| close_mosaic    | 0                                      | Mosaic数据增强关闭程度       |
| resume          | False                                  | 是否从断点继续训练           |
| amp             | False                                  | 使用自动混合精度             |
| fraction        | 1.0                                    | 训练数据集的比例             |
| profile         | False                                  | 是否进行性能分析             |
| freeze          | None                                   | 冻结的模型层                 |
| multi_scale     | False                                  | 是否使用多尺度训练           |
| overlap_mask    | False                                  | 遮罩重叠                     |
| mask_ratio      | None                                   | 遮罩比例                     |
| dropout         | None                                   | dropout 率                   |
| val             | True                                   | 训练时是否进行验证           |
| split           | val                                    | 数据集分割方式               |
| save_json       | False                                  | 是否保存JSON格式结果         |
| save_hybrid     | False                                  | 是否保存混合精度模型         |
| conf            | None                                   | 置信度阈值                   |
| iou             | 0.7                                    | 交并比（IoU）阈值            |
| max_det         | 300                                    | 每张图像最大检测数量         |
| half            | False                                  | 是否使用半精度训练           |
| dnn             | False                                  | 是否使用DNN优化              |
| plots           | True                                   | 训练过程是否生成图表         |
| source          | None                                   | 训练数据来源                 |
| vid_stride      | 1                                      | 视频处理步长                 |
| stream_buffer   | False                                  | 是否为视频流使用缓冲区       |
| visualize       | False                                  | 是否可视化检测结果           |
| augment         | False                                  | 是否使用数据增强             |
| agnostic_nms    | False                                  | 类别无关的NMS                |
| classes         | None                                   | 要检测的类别                 |
| retina_masks    | False                                  | 是否使用RetinaMask           |
| embed           | None                                   | 是否嵌入额外特征             |
| show            | False                                  | 训练过程是否显示图像         |
| save_frames     | False                                  | 是否保存训练过程中的图像帧   |
| save_txt        | False                                  | 是否保存检测结果为文本       |
| save_conf       | False                                  | 是否保存检测置信度           |
| save_crop       | False                                  | 是否保存裁剪后的图像         |
| show_labels     | True                                   | 是否显示类别标签             |
| show_conf       | True                                   | 是否显示置信度               |
| show_boxes      | True                                   | 是否显示检测框               |
| line_width      | None                                   | 检测框线宽                   |
| format          | torchscript                            | 模型保存格式                 |
| keras           | False                                  | 是否使用Keras                |
| optimize        | False                                  | 是否优化模型                 |
| int8            | False                                  | 是否使用8位量化              |
| dynamic         | False                                  | 是否使用动态量化             |
| simplify        | True                                   | 是否简化模型                 |
| opset           | None                                   | ONNX操作集版本               |
| workspace       | 4                                      | 模型工作空间大小             |
| nms             | False                                  | 是否使用非极大值抑制         |
| lr0             | 0.01                                   | 初始学习率                   |
| lrf             | 0.01                                   | 最终学习率                   |
| momentum        | 0.937                                  | 优化器动量参数               |
| weight_decay    | 0.0005                                 | 权重衰减                     |
| warmup_epochs   | 3.0                                    | 预热周期                     |
| warmup_momentum | 0.8                                    | 预热动量                     |
| warmup_bias_lr  | 0.1                                    | 预热阶段偏置学习率乘数       |
| box             | 7.5                                    | 边界框损失权重               |
| cls             | 0.5                                    | 分类损失权重                 |
| dfl             | 1.5                                    | 分布损失权重                 |
| pose            | 12.0                                   | 姿态估计损失权重             |
| kobj            | 1.0                                    | 目标损失权重                 |
| label_smoothing | 0.0                                    | 标签平滑                     |
| nbs             | 64                                     | 批量大小                     |
| hsv_h           | 0.015                                  | HSV色调变换范围              |
| hsv_s           | 0.7                                    | HSV饱和度变换范围            |
| hsv_v           | 0.4                                    | HSV明度变换范围              |
| degrees         | 0.0                                    | 旋转角度变换                 |
| translate       | 0.1                                    | 平移变换比例                 |
| scale           | 0.5                                    | 缩放变换比例                 |
| shear           | 0.0                                    | 剪切角度变换                 |
| perspective     | 0.0                                    | 透视变换强度                 |
| flipud          | 0.0                                    | 上下翻转概率                 |
| fliplr          | 0.5                                    | 左右翻转概率                 |
| bgr             | 0.0                                    | RGB/BGR格式转换概率          |
| mosaic          | 1.0                                    | Mosaic数据增强               |
| mixup           | 0.0                                    | Mixup数据增强                |
| copy_paste      | 0.0                                    | Copy-Paste数据增强           |
| copy_paste_mode | flip                                   | Copy-Paste翻转模式           |
| auto_augment    | randaugment                            | 自动数据增强                 |
| erasing         | 0.4                                    | 随机擦除概率                 |
| crop_fraction   | 1.0                                    | 裁剪比例                     |
| cfg             | None                                   | 额外模型配置                 |
| tracker         | botsort.yaml                           | 对象跟踪配置                 |
| save_dir        | runs\train\exp4                        | 训练结果保存目录             |
