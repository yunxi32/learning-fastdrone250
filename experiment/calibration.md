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