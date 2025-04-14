# Some problems encountered during the learning process and Changelog

## 2024.2.29
```xml
xacro：
<hardwareInterface>hardware_interface/EffortJointInterface</hardwareInterface>
 <!-- <hardwareInterface>hardware_interface/PositionJointInterface</hardwareInterface> -->
```

这块要注意！卡了好久没看出来.......

## 2024.3.01
给机械臂发布一个指令
```shell
rostopic pub -1 /urbot/wrist_1_joint_position_controller/command std_msgs/Float64 "data: -0.5" 
```

使用moveit建立运动学模型
‵‵‵xml
roslaunch moveit_setup_assistant setup_assistant.launch
```

以上工作参考：https://zhuanlan.zhihu.com/p/573972863