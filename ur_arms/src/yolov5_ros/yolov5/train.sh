conda activate yolo
python3 train.py --weights weights/yolov5s.pt  --cfg models/yolov5s.yaml  --data data/1.yaml --epoch 100 --batch-size 16 --img 640
