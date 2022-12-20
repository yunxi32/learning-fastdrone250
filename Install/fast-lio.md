# fast-lio安装配置

## 克隆

新建工作空间，进入src

```
git clone https://github.com/Livox-SDK/livox_ros_driver.git
cd ..
catkin_make
cd src
git clone https://github.com/hku-mars/FAST_LIO.git
cd FAST_LIO
git submodule update --init
cd ../..
catkin_make
source devel/setup.bash
```

