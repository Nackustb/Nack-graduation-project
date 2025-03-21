## ROS2

rosdepc安装依赖

```
cd your_workspace
rosdepc install -i --from-path src --rosdistro humble -y
```

编译

```
cd your_workspace
colcon build
```

source

```
source install/local_setup.sh
```

创建功能包

```
cd you_workspace/src
ros2 pkg create --build-type ament_cmake cpp_name
ros2 pkg create --build-type ament_python python_name
```

src 的目录结构

```
nack@nack:~/dev_ws$ cd src/
nack@nack:~/dev_ws/src$ tree
.
└── learning_node
    ├── learning_node
    │   ├── __init__.py
    │   ├── node_hello_class.py
    │   └── node_helloworld.py
    ├── package.xml
    ├── resource
    │   └── learning_node
    ├── setup.cfg
    ├── setup.py
    └── test
        ├── test_copyright.py
        ├── test_flake8.py
        └── test_pep257.py

```

修改setup.py

![image-20250321195740052](https://nack-1316646329.cos.ap-nanjing.myqcloud.com/image-20250321195740052.png)



