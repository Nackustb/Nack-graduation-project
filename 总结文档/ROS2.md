## 📦 安装依赖

使用 `rosdepc` 自动安装 `src` 中所有功能包的依赖（推荐在首次构建前使用）：

```
cd your_workspace
rosdepc install -i --from-path src --rosdistro humble -y
```

------

## 🛠 构建工作空间

使用 `colcon` 构建整个 ROS2 工作空间：

```
cd your_workspace
colcon build
```

------

## 📍 配置环境（source）

每次终端新开后都需要 `source` 一下安装环境：

```
bash
source install/local_setup.sh
```

建议写入 `~/.bashrc`：

```
bash
echo "source ~/your_workspace/install/local_setup.sh" >> ~/.bashrc
```

------

## 📁 创建功能包

### ➕ 创建 C++ 包（`ament_cmake`）

```
cd your_workspace/src
ros2 pkg create --build-type ament_cmake cpp_name
```

### ➕ 创建 Python 包（`ament_python`）

```
cd your_workspace/src
ros2 pkg create --build-type ament_python python_name
```

------

## 🗂 示例目录结构（Python 功能包）

```
nack@nack:~/dev_ws/src$ tree
.
└── learning_node
    ├── learning_node
    │   ├── __init__.py
    │   ├── node_hello_class.py
    │   └── node_helloworld.py
    ├── package.xml
    ├── resource
    │   └── learning_node
    ├── setup.cfg
    ├── setup.py
    └── test
        ├── test_copyright.py
        ├── test_flake8.py
        └── test_pep257.py
```

------

## ✍️ 修改 `setup.py`

一个典型 Python ROS2 节点包的 `setup.py` 如下所示：

```
python复制编辑from setuptools import setup

package_name = 'learning_node'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='Example Python node',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'node_helloworld = learning_node.node_helloworld:main',
            'node_hello_class = learning_node.node_hello_class:main',
        ],
    },
)
```

📌 注意：

- `console_scripts` 中的格式是：`命令名称 = 模块路径:入口函数`
- 必须保证 `learning_node/__init__.py` 存在（使其为 Python 包）
- `setup.cfg` 和 `resource/learning_node` 目录是标准规范要求，用于注册资源

![image-20250321195740052](https://nack-1316646329.cos.ap-nanjing.myqcloud.com/image-20250321195740052.png)



# ROS2 Launch 文件编写教程（基于 Python Launch）

------

## 📦 一、创建功能包

### 1. 创建一个功能包（支持 Launch）

```
ros2 pkg create bringup --build-type ament_python --dependencies rclpy launch launch_ros
```

> - `ament_python`：表示这是 Python 包（支持写 launch 脚本）
> - `--dependencies`：自动加入依赖到 `package.xml` 和 `setup.py`

------

### 2. 目录结构说明

```
my_robot_launch/
├── my_robot_launch/
│   ├── __init__.py
│   └── my_launch.py           # ✅ Python launch 文件
├── launch/                    # ✅ 推荐：单独存放 launch 脚本
│   └── my_launch.py
├── package.xml
├── setup.py
├── setup.cfg
```

> `launch/` 文件夹存放 launch 文件更清晰，也方便 `colcon build` 安装。

------

## ✏️ 二、编写 Launch 文件（`launch/my_launch.py`）

```
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='demo_nodes_cpp',         # 节点所属包名
            executable='talker',              # 可执行文件名
            name='talker_node',               # ROS 节点名
            output='screen'                   # 输出到终端
        ),
        Node(
            package='demo_nodes_py',
            executable='listener',
            name='listener_node',
            output='screen'
        )
    ])
```

------

## 🛠️ 三、修改 `setup.py` 以安装 launch 文件

```
from setuptools import setup

package_name = 'my_robot_launch'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
        # 👇 安装 launch 文件
        ('share/' + package_name + '/launch', ['launch/my_launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your@email.com',
    description='A launch example package',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [],
    },
)
```

------

## 🧱 四、构建并运行

### 1. 构建功能包

```
colcon build --packages-select my_robot_launch
```

记得在构建后执行：

```
source install/setup.bash
```

### 2. 启动 Launch 文件

```
ros2 launch my_robot_launch my_launch.py
```

------

## 🔄 五、传参、命名空间、组合节点（进阶）

### 示例：带参数和命名空间

```
Node(
    package='your_package',
    executable='your_node',
    name='your_node_name',
    namespace='robot1',
    parameters=[{'use_sim_time': True}],
    remappings=[
        ('/old_topic', '/new_topic')
    ],
    output='screen'
)
```

------

## 🧰 六、常用依赖说明

确保你的 `package.xml` 包含这些依赖：

```
<exec_depend>launch</exec_depend>
<exec_depend>launch_ros</exec_depend>
<exec_depend>rclpy</exec_depend>
```

------

## ✅ 七、完整案例仓库模板

可以使用如下命令快速生成一个 Launch 包模板：

```
ros2 pkg create my_launch_pkg --build-type ament_python --dependencies launch launch_ros rclpy
```

然后添加 `launch/your_launch.py` 文件即可。





# 🧠 ROS 2 Launch 启动系统笔记（Python版）

ROS 2 使用 Launch 文件来组织和启动多个节点与配置，尤其在多节点、仿真、视觉系统等场景中非常常用。以下为 Python 版 Launch 文件的整理笔记。

------

## 📦 基础模块导入

```
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
```

| 模块                       | 功能                           |
| -------------------------- | ------------------------------ |
| `LaunchDescription`        | 定义整个启动系统               |
| `DeclareLaunchArgument`    | 声明参数（可从命令行传入）     |
| `IncludeLaunchDescription` | 引入其他 `.launch.py` 文件     |
| `OpaqueFunction`           | 动态执行一个函数以生成启动项   |
| `LaunchConfiguration`      | 获取命令行/声明参数的值        |
| `FindPackageShare`         | 查找某个 ROS 包的 `share` 路径 |

------

## 🛠 基本结构

### 1. 启动函数（实际启动内容）

```
def launch_setup(context, *args, **kwargs):
    # 组织你要启动的节点、launch 文件
    return [ ... ]
```

使用 `OpaqueFunction` 执行这个函数，支持上下文传参（如 launch 参数）。

------

### 2. 生成总启动描述

```
def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument("param_name", default_value="value"),
        ...
    ]
    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
```

------

## 🧩 Include Launch Description 示例

```
IncludeLaunchDescription(
    PythonLaunchDescriptionSource([
        FindPackageShare("your_package"), "/launch/xxx.launch.py"
    ]),
    launch_arguments={
        "use_sim_time": "true"
    }.items()
)
```

### ✅ 实现功能：

- 包路径拼接（跨包引用 launch）
- 动态传参（如是否启用仿真时间）

------

## 📌 常见用途示例

### ✅ 启动 MoveIt 控制

```
IncludeLaunchDescription(
    PythonLaunchDescriptionSource([
        FindPackageShare("your_moveit_pkg"), "/launch/moveit.launch.py"
    ]),
    launch_arguments={"use_sim_time": "true"}.items()
)
```

------

### ✅ 启动 Gazebo + 控制器

```
IncludeLaunchDescription(
    PythonLaunchDescriptionSource([
        FindPackageShare("robot_description_pkg"), "/launch/robot_sim_control.launch.py"
    ]),
    launch_arguments={"launch_rviz": "true"}.items()
)
```

------

### ✅ 启动自定义视觉节点

```
IncludeLaunchDescription(
    PythonLaunchDescriptionSource([
        FindPackageShare("vision_pkg"), "/launch/seg_and_det.launch.py"
    ])
)
```

------

## 🧠 Declare Launch Argument 示例

```
DeclareLaunchArgument(
    "use_sim_time", default_value="true", description="Use simulation time"
)
```

在终端运行时可用：

```
ros2 launch your_pkg your_launch.py use_sim_time:=false
```

------

## 🚀 示例完整结构

```
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

def launch_setup(context, *args, **kwargs):
    node_1 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare("my_pkg"), "/launch/robot.launch.py"
        ])
    )
    return [node_1]

def generate_launch_description():
    return LaunchDescription([OpaqueFunction(function=launch_setup)])
```

------

## ✅ 调试建议

- 使用 `--show-args` 查看参数：

  ```
  ros2 launch your_pkg your_launch.py --show-args
  ```

- 加日志输出追踪问题：

  ```
  import os
  print("Launching from:", os.getcwd())
  ```
