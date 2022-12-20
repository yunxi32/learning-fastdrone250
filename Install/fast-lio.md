# fast-lio安装配置

## 克隆

新建工作空间，进入src，先安装雷达的驱动

```shell
git clone https://github.com/Livox-SDK/livox_ros_driver.git
cd ..
catkin_make
```
下载编译fastlio
```shell
cd src
git clone https://github.com/hku-mars/FAST_LIO.git
cd FAST_LIO
git submodule update --init
cd ../..
catkin_make
source devel/setup.bash
```

