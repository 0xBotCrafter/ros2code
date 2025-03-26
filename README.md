# ROS 2 系统状态监控项目

这是一个基于ROS 2的系统状态监控项目，用于收集、发布和显示系统状态信息（如CPU使用率、内存使用率、网络流量等）。

## 项目结构

本项目包含以下几个功能包：

- **chapt3_status_interfaces**: 定义系统状态消息接口
- **chapt3_status_publisher**: Python实现的系统状态发布节点
- **chapt3_status_display**: C++实现的系统状态显示节点，使用Qt5构建GUI界面
- **chapt2_3_pkg_cpp**: C++示例节点
- **chapt2_2_pkg_py**: Python示例节点
- **chapt2_1_first_node**: 第一个ROS 2节点示例

## 功能包详情

### chapt3_status_interfaces

定义了用于系统状态监控的消息接口：

- `SystemStatus.msg`: 包含时间戳、主机名、CPU使用率、内存使用率、内存总量、可用内存、网络发送和接收数据量等信息。

### chapt3_status_publisher

Python实现的系统状态发布节点，负责收集系统状态并发布到ROS 2话题。

### chapt3_status_display

C++实现的系统状态显示节点，使用Qt5构建图形界面，订阅系统状态话题并显示系统状态信息。

## 构建与运行

### 构建项目

```bash
# 在工作空间根目录下
colcon build
source install/setup.bash
```

### 运行系统状态发布节点

```bash
ros2 run chapt3_status_publisher sys_status_pub
```

### 运行系统状态显示节点

```bash
ros2 run chapt3_status_display sys_status_display
```

## 依赖

- ROS 2
- Python 3
- Qt5
- 系统状态监控相关Python库

## 注意事项

- 请确保已安装所有必要的依赖项
- 构建前请先确保ROS 2环境已正确设置

## 许可证

Apache-2.0

## 维护者

- 姓名：zgzhou
- 邮箱：zhouzge@foxmail.com