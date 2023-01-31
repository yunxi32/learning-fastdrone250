# 仿真环境搭建流程

主要参考官网：

[px4官网手册]: 

[Ubuntu Development Environment | PX4 User Guide](https://docs.px4.io/main/en/dev_setup/dev_env_linux_ubuntu.html)

## 一、安装Ubuntu18.04并装好ros

步骤省略

## 二、安装PX4

#### 1. 下载ubuntu.sh和requirements.txt

```
wget https://raw.githubusercontent.com/PX4/Firmware/master/Tools/setup/ubuntu.sh 

wget https://raw.githubusercontent.com/PX4/Firmware/master/Tools/setup/requirements.txt

```

```
source ubuntu.sh
```

#### 2.下载PX4源码

```
git clone https://github.com/PX4/Firmware.git
```

#### 3.运行gazebo，打开终端到`Firmware`文件夹，运行命令

```
make px4_sitl gazebo_iris
```

#### 安装问题解决

##### 1.python包安装过慢：打开下载的`ubuntu.sh`，找到下面几行

```
# Python3 dependencies
echo
echo "Installing PX4 Python3 dependencies"
sudo python3 -m pip install --upgrade pip setuptools wheel
sudo python3 -m pip install -r ${DIR}/requirements.txt


# Python2 dependencies
echo
echo "Installing PX4 Python2 dependencies"
sudo python2 -m pip install --upgrade pip setuptools wheel
sudo python2 -m pip install -r ${DIR}/requirements.txt
```

修改为

```
# Python3 dependencies
echo
echo "Installing PX4 Python3 dependencies"
sudo python3 -m pip install --upgrade -i https://pypi.tuna.tsinghua.edu.cn/simple pip setuptools wheel
sudo python3 -m pip install -i https://pypi.tuna.tsinghua.edu.cn/simple -r ${DIR}/requirements.txt


# Python2 dependencies
echo
echo "Installing PX4 Python2 dependencies"
sudo python2 -m pip install --upgrade -i https://pypi.tuna.tsinghua.edu.cn/simple pip setuptools wheel
sudo python2 -m pip install -i https://pypi.tuna.tsinghua.edu.cn/simple -r ${DIR}/requirements.txt
```

运行

```
source ubuntu.sh
```

## 三、mavros 安装

### 1.安装环境依赖

```
wget https://raw.githubusercontent.com/PX4/Devguide/master/build_scripts/ubuntu_sim_common_deps.sh
```

```
./ubuntu_sim_common_deps.sh
```

```
chmod +x ubuntu_sim_common_deps.sh
```

### 2.下载编译

```
sudo apt-get install python-catkin-tools python-rosinstall-generator -y
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
catkin init
wstool init src
rosinstall_generator --rosdistro kinetic mavlink | tee /tmp/mavros.rosinstall
rosinstall_generator --upstream mavros | tee -a /tmp/mavros.rosinstall
wstool merge -t src /tmp/mavros.rosinstall
wstool update -t src

rosdep install --from-paths src --ignore-src -y

wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh

./install_geographiclib_datasets.sh

catkin build
```

```
echo "source ~/catkin_ws/devel/setup.bash >> ~/.bashrc
source ~/catkin_ws/devel/setup.bash
```

### 3.offboard设置

打开官方[MAVROS *Offboard* control example](https://dev.px4.io/master/en/ros/mavros_offboard.html)链接，然后在catkin_ws目录中，运行命令

```bash
catkin_create_pkg offboard_pkg roscpp std_msgs geometry_msgs mavros_msgs
```

然后定位到目录`~/catkin_ws/src/offboard_pkg/src/`，新建一个文件`offboard_node.cpp`。

将代码复制进去(官方示例):

```c++
/**
 * @file offb_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo SITL
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 2;

    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    while(ros::ok()){
        if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }

        local_pos_pub.publish(pose);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
```

然后打开目录`~/catkin_ws/src/offboard_pkg/`下的`CMakeLists.txt`添加下面的两行：

```cmake
add_executable(offboard_node src/offboard_node.cpp)
target_link_libraries(offboard_node ${catkin_LIBRARIES})
```

然后到目录`~/catkin_ws`下，运行命令

```bash
catkin build
```

等待编译完成后，如果你要在`gazebo`中仿真，运行命令

```bash
make px4_sitl gazebo_iris
```

打开`QGroundControl`。

然后在终端下运行命令：

```bash
roslaunch mavros px4.launch fcu_url:="udp://:14540@127.0.0.1:14557"
```

启动`PX4`与`Mavros`之间的连接，然后运行命令

```bash
rosrun offboard_pkg offboard_node
```

然后进入`gazebo`中进行观察。

## 四、下载QGC

链接：

[QGC下载]: https://docs.qgroundcontrol.com/master/en/getting_started/download_and_install.html

## 五、实现offboard模式sin路线飞行控制：

### 1.找到PX4的安装路径，打开gazebo仿真并启动PX4：

```
cd Firmware
make px4_sitl gazebo_iris
```

### 2.打开QGC地面站

### 3.建立mavros与PX4的连接

```
roslaunch mavros px4.launch fcu_url:="udp://:14540@127.0.0.1:14557"
```

### 4.找到ROS工作空间，创建功能包

```
catkin_create_pkg offboard_sin roscpp std_msgs geometry_msgs mavros_msgs
```


在功能包的src路径下创建offboard_sin_node.cpp文件写入如下代码

（根据官方offboard代码示例改写，链接：https://docs.px4.io/master/en/ros/mavros_offboard.html）

```c++
/**

 * @file offb_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo SITL
   */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#define PI acos(-1)mavros_msgs::State current_state;
geometry_msgs::PoseStamped current_position;void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}
void getpointfdb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    ROS_INFO("x: [%f]", msg->pose.position.x);
    ROS_INFO("y: [%f]", msg->pose.position.y);
    ROS_INFO("z: [%f]", msg->pose.position.z);
    current_position = *msg;
}int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
        ("mavros/state", 10, state_cb);
        
ros::Subscriber get_point = nh.subscribe<geometry_msgs::PoseStamped>
        ("mavros/local_position/pose", 10, getpointfdb);
        
ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
        ("mavros/setpoint_position/local", 10);
ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
        ("mavros/cmd/arming");
ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
        ("mavros/set_mode");

//the setpoint publishing rate MUST be faster than 2Hz
ros::Rate rate(20.0f);

// wait for FCU connection
while(ros::ok() && !current_state.connected){
    ros::spinOnce();
    rate.sleep();
}

geometry_msgs::PoseStamped pose;
pose.pose.position.x = 0;
pose.pose.position.y = 0;
pose.pose.position.z = 3;
    //send a few setpoints before starting
for(int i = 100; ros::ok() && i > 0; --i){
    local_pos_pub.publish(pose);
    ros::spinOnce();
    rate.sleep();
}

mavros_msgs::SetMode offb_set_mode;
offb_set_mode.request.custom_mode = "OFFBOARD";

mavros_msgs::CommandBool arm_cmd;
arm_cmd.request.value = true;

ros::Time last_request = ros::Time::now();

while(ros::ok()){
    if( current_state.mode != "OFFBOARD" &&
        (ros::Time::now() - last_request > ros::Duration(5.0f))){
        if( set_mode_client.call(offb_set_mode) &&
            offb_set_mode.response.mode_sent){
            ROS_INFO("Offboard enabled");
        }
        last_request = ros::Time::now();
    } else {
        if( !current_state.armed &&
            (ros::Time::now() - last_request > ros::Duration(5.0f))){
            if( arming_client.call(arm_cmd) &&
                arm_cmd.response.success){
                ROS_INFO("Vehicle armed");
            }
            last_request = ros::Time::now();
        }
    }
    
    if((abs(current_position.pose.position.x-pose.pose.position.x)<0.5f)&&(abs(current_position.pose.position.y-pose.pose.position.y)<0.5f)&&(abs(current_position.pose.position.y-pose.pose.position.y)<0.5f))
    {
        pose.pose.position.x += 5;
        pose.pose.position.y = 20*sin(pose.pose.position.x/40*PI);
        pose.pose.position.z = 3;
    }

    local_pos_pub.publish(pose);

    ros::spinOnce();
    rate.sleep();
}

return 0;
    }
```


其中通过订阅mavros发布的的mavros/local_position/pose话题得到当前无人机的位置反馈

修改CMakeLists.txt文件，在最后加入下面两行：

```
add_executable(offboard_node src/offboard_node.cpp)
target_link_libraries(offboard_node ${catkin_LIBRARIES})
```


