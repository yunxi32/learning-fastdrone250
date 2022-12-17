# learning-fastdrone250

## 入门

整个项目的软件部分基于[ZJU-FAST-Lab/Fast-Drone-250: hardware and software design of the 250mm autonomous drone (github.com)](https://github.com/ZJU-FAST-Lab/Fast-Drone-250)

完整学习视频[第一课：课程介绍_哔哩哔哩_bilibili](https://www.bilibili.com/video/BV1WZ4y167me/?p=1)

并在此基础上依据需求做出了相应调整。

需要完全掌握并会使用以下模块：

[Fast-Drone-250/src/realflight_modules/px4ctrl at master · ZJU-FAST-Lab/Fast-Drone-250 (github.com)](https://github.com/ZJU-FAST-Lab/Fast-Drone-250/tree/master/src/realflight_modules/px4ctrl)

需要学会使用以下模块：

[Fast-Drone-250/src/planner/plan_manage/launch at master · ZJU-FAST-Lab/Fast-Drone-250 (github.com)](https://github.com/ZJU-FAST-Lab/Fast-Drone-250/tree/master/src/planner/plan_manage/launch)其中包含了ego-planer的仿真和试飞启动文件

[hku-mars/FAST_LIO: A computationally efficient and robust LiDAR-inertial odometry (LIO) package (github.com)](https://github.com/hku-mars/FAST_LIO)激光slam（我们采用的方案）

[HKUST-Aerial-Robotics/VINS-Fusion: An optimization-based multi-sensor state estimator (github.com)](https://github.com/HKUST-Aerial-Robotics/VINS-Fusion)视觉slam（原版fastdrone250采用的方案）

[WongKinYiu/yolov7: Implementation of paper - YOLOv7: Trainable bag-of-freebies sets new state-of-the-art for real-time object detectors (github.com)](https://github.com/WongKinYiu/yolov7)目标检测模块

## 无人机硬件部分

需要购买的主要硬件以及组装见[learning-fastdrone250/composition.md at main · KayatoDQY/learning-fastdrone250 (github.com)](https://github.com/KayatoDQY/learning-fastdrone250/blob/main/hardware/composition.md)

组装完成后设置飞控见[learning-fastdrone250/set_uav.md at main · KayatoDQY/learning-fastdrone250 (github.com)](https://github.com/KayatoDQY/learning-fastdrone250/blob/main/hardware/set_uav.md)

## 无人机软件部分安装

### 系统安装

#### Ubuntu20.04安装

#### Ubuntu18.04安装（用于仿真）

### 常用软件安装

clash

nomechine

vscode

ssh

### 代码部署和环境配置

ros

#### fastdrone250安装配置

#### fastlio安装配置

#### yolov7训练与部署

##### 训练环境安装

在windows上进行

##### 部署到ros上

### 仿真环境配置

需在Ubuntu18.04上配置

## 实验

### slam测试

### slam试飞

### egoplaner仿真

### egoplaner试飞

### yolov7训练

### yolov7在ros上测试





