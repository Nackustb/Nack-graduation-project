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

