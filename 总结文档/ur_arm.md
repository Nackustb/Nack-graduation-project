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

