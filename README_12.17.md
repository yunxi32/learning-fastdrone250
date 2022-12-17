# 工训赛无人机

## 启动指令

时间对齐

```
sudo ptpd -M -i enp88s0 -C
```

启动mavros

```
roslaunch mavros px4.launch
```

查IMU的频率是否接近200

```
rostopic hz /mavros/imu/data
```

开启雷达

```
roslaunch livox_ros_driver livox_lidar_msg.launch
```

启动fast_lio

```
roslaunch fast_lio mapping_avia.launch
```

连接slam,里程计和飞控

```
roslaunch px4_realsence_bridge bridge.launch
```





## 安装

###  imu内参

安装code_utils需要安装ceres库

首先安装code_utils库

安装依赖

```
sudo apt-get install libdw-dev
```

在src中下载code_utils

```
git clone https://github.com/gaowenliang/code_utils
```

在工作空间中编译

```
cd catkin_ws
catkin_make
```

编译中可能遇到问题，库路径不对和OPENCV版本问题

### 雷达和imu的外参

使用官方的标定包



## 调参

### imu内参

录包

```
rosbag record -O imu_bagname /mavros/imu/data_raw 
```

#TODO

### 雷达和imu的外参

#TODO



## QGC相关参数更改

`dshort_config` 调整电机

`CBRK_USB_CHK` 为了让插了USB之后还可以解锁

`CBRK_IO_SAFETY` 跳过安全开关

`SYS_TEL1_BAUD`改串口波特率

![](C:\Users\31919\Desktop\QQ图片20221216183141.jpg)



## 怎么调电机顺序

## QGC确定电机顺序,和顺逆时针

![](img/444.jpg)

白色是地线，下面是信号线，插好后是蓝色

电机顺序如果反了，就调信号线

![](img/333.jpg)

*顺逆时针反了就在QGC中输命令*

命令1是调顺逆时针，1可替换未2 3 4

命令二是保存

![](img/222.jpg)

![](img/111.jpg)



## CLASH FOR WINDOWS 如何在Linux上使用

### # 1. 安装clash for windows

点开链接,选择clash for windows的linux版本

[clash]: https://github.com/Fndroid/clash_for_windows_pkg/releases/tag/0.17.

![](img/QQ图片20221216185120.jpg)

### 打开clash

![](img/QQ图片20221216185124.jpg)

`cd clash`

`./cfw`

设置clah

![](../../25895336-9667850b01c82b6c.webp)

### 修改配置文件

`sudo chmod 666 /etc/environment`

`sodo gedit /etc/environment`

### 添加以下内容并保存

    `http_proxy=http://127.0.0.1:7890/ https_proxy=http://127.0.0.1:7890/ ftp_proxy=http://127.0.0.1:7890/ HTTP_PROXY=http://127.0.0.1:7890/ HTTPS_PROXY=http://127.0.0.1:7890/ FTP_PROXY=http://127.0.0.1:7890/`

### 保存

`sudo chmod 444 /etc/environment`



## 修改及原因

## 当前问题

### 11月20日

#### 起飞不稳

做了imu和雷达标定后，所有部分正常开启，SlAM非常稳定，启动OFFBOARD代码后，无人机会朝着左后方撞击。

终端出现如下报错

PositionTargetGlobal failed because no origin

#### **解决办法**

要在QGC里EKF2MASK里GPS相关参数关掉



#### 降落时会直接断电摔下

手飞时会突然断电摔下，自动飞时调到land模式会直接摔下

上图可见油门值和电机的输出值，在一瞬间突然掉零。该问题导致一电机烧毁，解决办法需要和电机的商家沟通

#### 原因分析

无人机动力系统问题