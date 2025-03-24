## 奥比中光 Femto Bolt标定 --ROS2

开启相机

```
ros2 launch orbbec_camera femto_bolt.launch.py
```

标定rgb

```
ros2 run camera_calibration cameracalibrator --size 16x12 --square 0.024 --no-service-check image:=/camera/color/image_raw camera:=/camera/color
```

标定ir

```
ros2 run camera_calibration cameracalibrator --size 16x12 --square 0.024 --no-service-check image:=/camera/ir/image_raw camera:=/camera/ir
```

标定depth rgb 左右分辨率不一致，未果。

```
ros2 run camera_calibration cameracalibrator --size 16x12 --square 0.024 --no-service-check --ros-args --remap image:=/camera/ir/image_raw --remap left:=/camera/ir/image_raw --remap left_camera:=/camera/ir --remap right:=/camera/color/image_raw --remap right_camera:=/camera/color
```

