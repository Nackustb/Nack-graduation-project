## TF坐标系

```
# 绘制5s坐标系
ros2 run tf2_tools view_frames
# 展示两坐标系之间位置关系
ros2 run tf_tools ttf2_echo tf1 tf2
# rviz2 可视化
```



## ur5

#### ros2启动

```
ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur5 launch_rviz:=true
```

#### 仿真启动

```
ros2 run ur_robot_driver start_ursim.sh -m ur5
```

```
ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5 robot_ip:=192.168.56.101 launch_rviz:=true
```

#### 文件类型区别

| 文件类型 | 全称                              | 作用                      | 举例内容                      | 主要用于             |
| -------- | --------------------------------- | ------------------------- | ----------------------------- | -------------------- |
| `.urdf`  | Unified Robot Description Format  | 描述机器人结构            | link、joint、mesh             | Gazebo、Rviz、MoveIt |
| `.srdf`  | Semantic Robot Description Format | 语义信息（规划组、末端）  | groups、end_effectors         | MoveIt               |
| `.xacro` | XML Macro                         | 用宏构造 URDF，增强复用性 | `<xacro:macro>` `<xacro:arg>` | 生成 URDF            |
