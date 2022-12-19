# fastdrone250安装配置

## realsense驱动安装

这是Inter相机的驱动，在源项目中，用这款相机运行vins-fusion

```shell
sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-key  F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key  F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE
```

```shell
sudo add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" -u
```

```shell
sudo apt-get install librealsense2-dkms
sudo apt-get install librealsense2-utils
sudo apt-get install librealsense2-dev
sudo apt-get install librealsense2-dbg
```

**测试**

```shell
realsense-viewer
```

## mavros

这是ros与无人机飞控的通信包，必装

```shell
sudo apt-get install ros-noetic-mavros
```

这一步需要保证前面的clash安装成功

```shell
cd /opt/ros/noetic/lib/mavros
sudo ./install_geographiclib_datasets.sh
```

## ceres与glog与ddyanmic-reconfigure

这些是一些数学计算的依赖安装，在这里装了之后，后面操作一定不能动这个ceres，极其容易破坏环境。

用的fastdrone250项目中提供的安装包

```shell
 sudo apt-get install git
 git clone  https://github.com/ZJU-FAST-Lab/Fast-Drone-250.git
```

在克隆的文件中解压`3rd_party.zip`压缩包

```shell
cd glog
sudo chmod 777./autogen.sh
sudo chmod 777 ./configure
./autogen.sh && ./configure && make && sudo make install
sudo apt-get install liblapack-dev libsuitesparse-dev libcxsparse3.1.2 libgflags-dev libgoogle-glog-dev libgtest-dev
```

```shell
cd ceres
mkdir build
cd build
cmake ..
sudo make -j4
sudo make install
```

```shell
sudo apt-get install ros-noetic-ddynamic-reconfigure
```

## 编译

```shell
cd Fast-Drone-250
catkin_make
```

可以在`~/.bashrc`中写入环境，也可以采用临时添加环境的指令

```shell
source devel/setup.bash
```

ego算法仿真

```shell
roslaunch ego_planner single_run_in_sim.launch
```

在rviz内按键盘G为无人机选定目标点