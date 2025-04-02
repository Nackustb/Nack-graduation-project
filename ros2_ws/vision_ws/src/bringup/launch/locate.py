import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # 获取 launch 文件路径
    orbbec_launch = os.path.join(
        os.getenv("HOME"), "vision_ws/src/orbbec_camera/launch/femto_bolt.launch.py"
    )
    yolo_launch = os.path.join(
        os.getenv("HOME"), "vision_ws/src/yolo_bringup/launch/yolov11.launch.py"
    )
    
    # 获取 RViz 配置文件路径
    rviz_config_file = os.path.join(
        os.getenv("HOME"), "/home/nack/.rviz2/default.rviz"
    )

    return LaunchDescription([
        # 启动 Orbbec 深度相机
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(orbbec_launch)
        ),

        # 启动 YOLOv11
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(yolo_launch)
        ),

        # 启动 RViz2 并加载默认配置
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_file],
            output='screen'
        ),
    ])
