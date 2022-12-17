#  px4ctrl

## 文件组成

```
C:.
│  CMakeLists.txt
│  LICENSE
│  package.xml
│  README.md
│
├─config
│      ctrl_param_fpv.yaml
│
├─launch
│      run_ctrl.launch
│      thrust_calibrate.launch
│
├─src
│      controller.cpp
│      controller.h
│      input.cpp
│      input.h
│      PX4CtrlFSM.cpp
│      PX4CtrlFSM.h
│      PX4CtrlParam.cpp
│      PX4CtrlParam.h
│      px4ctrl_node.cpp
│
└─thrust_calibrate_scrips
        record.sh
        thrust_calibrate.py
```

## launch

### run_ctrl.launch

```xml
<launch>

	<node pkg="px4ctrl" type="px4ctrl_node" name="px4ctrl" output="screen">
        	<!-- <remap from="~odom" to="/vicon_imu_ekf_odom" /> -->
			
			<remap from="~odom" to="/vins_fusion/imu_propagate" />

		<remap from="~cmd" to="/position_cmd" />

        <rosparam command="load" file="$(find px4ctrl)/config/ctrl_param_fpv.yaml" />
	</node>
 
</launch>

```

启动px4ctrl节点，将`/vins_fusion/imu_propagate`映射为`/odom`，将`/position_cmd`映射为`/cmd`，加载参数文件

### thrust_calibrate.launch

```xml
<launch>

	<node pkg="px4ctrl" name="thrust_calibrate" type="thrust_calibrate.py" output="screen">
		<param name="time_interval" value="1.0" />
		<param name="mass_kg" value="10.7" />
	</node>
 
</launch>

```

参数估计节点`TODO`

## config

### ctrl_param_fpv.yaml

```yaml
#########################################################################
# ⭐⭐⭐                仔细设置参数以获得满意的性能！               ⭐⭐⭐ #
#########################################################################

mass        : 1.2 			# 无人质量 
gra         : 9.81			# 重力加速度
pose_solver : 1     		# 姿态求解器 0:From ZhepeiWang (drag & less singular) 1:From ZhepeiWang, 2:From rotor-drag    
ctrl_freq_max   : 100.0		# 控制的最大频率
use_bodyrate_ctrl: false	# 使用bodyrate控制
max_manual_vel: 1.0			# 最大手动速度？？
max_angle: 30  				# 以度为单位的姿态角限制。负值意味着没有限制
low_voltage: 13.2 			# 电池电压

rc_reverse:  				# 是否接受遥控器的4个摇杆值
    roll: false
    pitch: false
    yaw: false
    throttle: false

auto_takeoff_land:
    enable: true   			# 是否允许自动起飞降落
    enable_auto_arm: true	# 是否允许自动解锁
    no_RC: false			# 是否没有遥控器
    takeoff_height: 1.0   	# 起飞高度
    takeoff_land_speed: 0.3 # 起飞降落速度

```

pid参数

```yaml
gain: 
    # 级联 PID 控制器。建议阅读代码。
    Kp0: 1.5
    Kp1: 1.5 
    Kp2: 1.5
    Kv0: 1.5
    Kv1: 1.5
    Kv2: 1.5
    # ↓↓↓ 现在不使用 --
    Kvi0: 0.0
    Kvi1: 0.0
    Kvi2: 0.0
    Kvd0: 0.0
    Kvd1: 0.0
    Kvd2: 0.0
    # ↓↓↓ 仅在速度控制模式下使用。
    KAngR: 20.0
    KAngP: 20.0
    KAngY: 20.0

```

推力控制模型参数

```yaml
thrust_model: 				# The model that maps thrust signal u(0~1) to real thrust force F(Unit:N): F=K1*Voltage^K2*(K3*u^2+(1-K3)*u). 
    # 将归一化推力信号映射到实际推力的模型（单位N）
    print_value: false 		# display the value of “thr_scale_compensate” or “hover_percentage” during thrust model estimating.
    # 是否在运行推力模型时显示值
    accurate_thrust_model: false  
    # 精确的推力模型
    # This can always enabled if don't require accurate control performance :-)
    # 如果不需要精确控制性能可以始终启用该模型
    # accurate thrust mapping parameters
    # 精确的推力映射参数
    K1: 0.7583 				# 需要精确校准！
    K2: 1.6942 				# 需要精确校准！
    K3: 0.6786 				# 需要精确校准！ K3 等于 THR_MDL_FAC 在https://docs.px4.io/master/en/config_mc/pid_tuning_guide_multicopter.html.
    
    # 近似推力映射参数
    hover_percentage: 0.30  # 推力参数在自稳模式的百分比 # *

```

电机阻力参数

```yaml
rotor_drag:  				# 转子阻力	
    x: 0.0  				# The reduced acceleration on each axis caused by rotor drag. Unit:(m*s^-2)/(m*s^-1).
    # 转子阻力引起的每个轴上的加速度降低。	
    y: 0.0  				# Same as above
    z: 0.0  				# Same as above
    k_thrust_horz: 0.0		# 推荐设置为0 --

```

其他

```yaml
msg_timeout:				# 消息超时判断时间
    odom: 0.5
    rc:   0.5
    cmd:  0.5
    imu:  0.5
    bat:  0.5
```



## thrust_calibrate_scrips

推力估计脚本

```sh
rosbag record --tcpnodelay /mavros/battery /mavros/setpoint_raw/attitude /traj_start_trigger
```

`TODO`

```python
#!/usr/bin/env python2
# -*- coding: utf-8 -*-

import numpy as np
import rospy
import csv
import time
import rospkg
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import BatteryState
from mavros_msgs.msg import AttitudeTarget


class ThrustCalibration:

    def __init__(self):

        # Load parameters
        self.time_average_interval = float(
            rospy.get_param('~time_interval', 1.0))
        self.vbat_min = float(
            rospy.get_param('~min_battery_voltage', 13.2))
        self.mass_kg = float(
            rospy.get_param('~mass_kg', 1.0))

        # store measurements of vbat and commanded thrusts
        self.volt_buf = np.array([])
        self.volt_records = np.array([])
        self.cmd_buf = np.array([])
        self.cmd_records = np.array([])
        self.last_records_t = rospy.get_rostime()

        # Some flags
        self.count_start_trigger_received = 0

        # Subscribe
        self.bat_sub = rospy.Subscriber(
            "/mavros/battery", BatteryState, self.battery_voltage_cb)
        rospy.Subscriber(
            "/mavros/setpoint_raw/attitude", AttitudeTarget, self.thrust_commands_cb)
        rospy.Subscriber(
            "/traj_start_trigger", PoseStamped, self.start_trigger_cb)

    def battery_voltage_cb(self, msg):

        if (self.count_start_trigger_received == 0):
            return

        self.volt_buf = np.append(
            self.volt_buf, np.array([msg.voltage]), axis=0)

        # Record filted data
        cur_t = rospy.get_rostime()
        if ((cur_t - self.last_records_t).to_sec() > self.time_average_interval):
            self.last_records_t = cur_t
            # print self.volt_records.shape
            # print self.volt_buf.shape
            # print
            # print np.mean(self.volt_buf)
            self.volt_records = np.append(
                self.volt_records, np.array([np.mean(self.volt_buf)]), axis=0)
            self.volt_buf = np.array([])
            self.cmd_records = np.append(
                self.cmd_records, np.array([np.mean(self.cmd_buf)]), axis=0)
            self.cmd_buf = np.array([])
            print "volt=", self.volt_records[-1], " thr=", self.cmd_records[-1]

        # Calculate calibration parameters
        if ((self.count_start_trigger_received >= 2) or 
            (self.volt_records.size != 0 and self.volt_records[-1] < self.vbat_min)):
            self.cal_and_save_data()
            self.bat_sub.unregister()

    def thrust_commands_cb(self, msg):

        if (self.count_start_trigger_received == 0):
            return

        self.cmd_buf = np.append(self.cmd_buf, np.array([msg.thrust]), axis=0)
        print msg.thrust

    def start_trigger_cb(self, msg):

        self.count_start_trigger_received += 1
        if(self.count_start_trigger_received == 1):
            rospy.loginfo("Start recording.")
        if(self.count_start_trigger_received > 1):
            rospy.loginfo("Stop recording.")

    def cal_and_save_data(self):
        rospy.loginfo("Data storing.")

        data_stack = np.vstack((self.cmd_records, self.volt_records))
        data = [tuple(x) for x in data_stack.tolist()]

        rospack = rospkg.RosPack()
        filedir = rospack.get_path(
            'px4ctrl')+'/thrust_calibrate_scrips/data.csv'
        f = open(filedir, 'a')
        writer = csv.writer(f)
        writer.writerow(((time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())),
                         'mass(kg):', self.mass_kg, 'commands', 'voltage'))
        writer.writerows(data)
        f.close()

        rospy.loginfo("Stored to " + filedir)


if __name__ == '__main__':

    try:
        rospy.init_node("thrust_calibration")

        thrust_calibration = ThrustCalibration()
        rospy.loginfo("Waiting for trigger.")

        rospy.spin()

    except rospy.ROSInterruptException:
        pass
```

## `src`

### px4ctrl_node.cpp

```c++
#include <ros/ros.h>
#include "PX4CtrlFSM.h"
#include <signal.h>
```

```c++
void mySigintHandler(int sig)
{
    ROS_INFO("[PX4Ctrl] exit...");
    ros::shutdown();
}
```

#### 主函数部分

```c++
    ros::init(argc, argv, "px4ctrl");
    ros::NodeHandle nh("~");
```

ros节点初始化

```c++
	signal(SIGINT, mySigintHandler);
    ros::Duration(1.0).sleep();
```

信号捕捉函数，全局有效（多线程）。此时按ctrl+c将不再强制中断程序，而是由程序捕捉到中断信号`SIGINT`，触发软中断，关闭ros节点。延时1s

```c++
	Parameter_t param;
    param.config_from_ros_handle(nh);
```

加载参数

```c++
	LinearControl controller(param);
    PX4CtrlFSM fsm(param, controller);
```

实列化控制器



创建ros订阅者

```c++
	//订阅飞控当前状态,回调State_Data_t::feed，传入state_data和订阅订阅消息

	ros::Subscriber state_sub =
        nh.subscribe<mavros_msgs::State>("/mavros/state",
                                         10,
                                         boost::bind(&State_Data_t::feed, &fsm.state_data, _1));
	//订阅飞控扩展状态，着陆探测器和垂直起降状态。
    ros::Subscriber extended_state_sub =
        nh.subscribe<mavros_msgs::ExtendedState>("/mavros/extended_state",
                                                 10,
                                                 boost::bind(&ExtendedState_Data_t::feed, &fsm.extended_state_data, _1));
	//订阅里程计信息，此时由launch中将vins映射为odom
    ros::Subscriber odom_sub =
        nh.subscribe<nav_msgs::Odometry>("odom",
                                         100,
                                         boost::bind(&Odom_Data_t::feed, &fsm.odom_data, _1),
                                         ros::VoidConstPtr(),
                                         ros::TransportHints().tcpNoDelay());
	//订阅控制指令
    ros::Subscriber cmd_sub =
        nh.subscribe<quadrotor_msgs::PositionCommand>("cmd",
                                                      100,
                                                      boost::bind(&Command_Data_t::feed, &fsm.cmd_data, _1),
                                                      ros::VoidConstPtr(),
                                                      ros::TransportHints().tcpNoDelay());
	//订阅imu数据，不能使用原始数据
    ros::Subscriber imu_sub =
        nh.subscribe<sensor_msgs::Imu>("/mavros/imu/data", // Note: do NOT change it to /mavros/imu/data_raw !!!
                                       100,
                                       boost::bind(&Imu_Data_t::feed, &fsm.imu_data, _1),
                                       ros::VoidConstPtr(),
                                       ros::TransportHints().tcpNoDelay());
	//如果参数中声明有遥控器，则创建rc订阅
    ros::Subscriber rc_sub;
    if (!param.takeoff_land.no_RC) // mavros will still publish wrong rc messages although no RC is connected mavros
     							   //将发出没有遥控器连接的警告
    {
        rc_sub = nh.subscribe<mavros_msgs::RCIn>("/mavros/rc/in",
                                                 10,
                                                 boost::bind(&RC_Data_t::feed, &fsm.rc_data, _1));
    }
	//订阅飞控电池状态
    ros::Subscriber bat_sub =
        nh.subscribe<sensor_msgs::BatteryState>("/mavros/battery",
                                                100,
                                                boost::bind(&Battery_Data_t::feed, &fsm.bat_data, _1),
                                                ros::VoidConstPtr(),
                                                ros::TransportHints().tcpNoDelay());
	//订阅起飞和降落指令
    ros::Subscriber takeoff_land_sub =
        nh.subscribe<quadrotor_msgs::TakeoffLand>("takeoff_land",
                                                  100,
                                                  boost::bind(&Takeoff_Land_Data_t::feed, &fsm.takeoff_land_data, _1),
                                                  ros::VoidConstPtr(),
                                                  ros::TransportHints().tcpNoDelay());

```

_1 占位符，替换参数

boost::bind(调用函数地址，参数1，参数2。。。),返回调用函数函数地址

ros::VoidConstPtr(),ros::TransportHints().tcpNoDelay());采用tcp传输，减小延时？？？



创建服务

```c++
    //创建模式控制服务
	fsm.set_FCU_mode_srv = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
	//创建上/解锁服务
    fsm.arming_client_srv = nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
	//创建自定义mavlinkzhi
    fsm.reboot_FCU_srv = nh.serviceClient<mavros_msgs::CommandLong>("/mavros/cmd/command");
	//延时0.5s
    ros::Duration(0.5).sleep();
```

处理遥控器逻辑

```c++
	//如果遥控器不存在，则在终端输出警告
	if (param.takeoff_land.no_RC)
    {
        ROS_WARN("PX4CTRL] Remote controller disabled, be careful!");
    }
    else
    {
        ROS_INFO("PX4CTRL] Waiting for RC");
        while (ros::ok())
        {
            ros::spinOnce();
            //每隔0.1s，判断一次遥控器是否连接，判断时间同步
            if (fsm.rc_is_received(ros::Time::now()))
            {
                ROS_INFO("[PX4CTRL] RC received.");
                break;
            }
            ros::Duration(0.1).sleep();
        }
    }
```

连接飞控

```c++
	int trials = 0;
    while (ros::ok() && !fsm.state_data.current_state.connected)//判断是否连接
    {
        ros::spinOnce();
        ros::Duration(1.0).sleep();
        if (trials++ > 5)
            //延时五秒后若未能连接，则抛出错误
            ROS_ERROR("Unable to connnect to PX4!!!");
    }
```

```c++
	//以最大控制频率为循环频率
	ros::Rate r(param.ctrl_freq_max);
    while (ros::ok())
    {
        r.sleep();
        ros::spinOnce();
        //开始控制
        fsm.process(); // We DO NOT rely on feedback as trigger, since there is no significant performance difference through our test.
        				//我们不依赖反馈作为触发器，因为通过我们的测试没有显着的性能差异。
    }
```

### PX4CtrlParam.h

参数读取

```c++
#include <ros/ros.h>
```

参数存储类

```c++
class Parameter_t
{
public:
    //参数文件中的对应参数
	struct Gain
	{
		double Kp0, Kp1, Kp2;
		double Kv0, Kv1, Kv2;
		double Kvi0, Kvi1, Kvi2;
		double Kvd0, Kvd1, Kvd2;
		double KAngR, KAngP, KAngY;
	};

	struct RotorDrag
	{
		double x, y, z;
		double k_thrust_horz;
	};

	struct MsgTimeout
	{
		double odom;
		double rc;
		double cmd;
		double imu;
		double bat;
	};

	struct ThrustMapping
	{
		bool print_val;
		double K1;
		double K2;
		double K3;
		bool accurate_thrust_model;
		double hover_percentage;
	};

	struct RCReverse
	{
		bool roll;
		bool pitch;
		bool yaw;
		bool throttle;
	};

	struct AutoTakeoffLand
	{
		bool enable;
		bool enable_auto_arm;
		bool no_RC;
		double height;
		double speed;
	};
	//实列化对应结构体
	Gain gain;
	RotorDrag rt_drag;
	MsgTimeout msg_timeout;
	RCReverse rc_reverse;
	ThrustMapping thr_map;
	AutoTakeoffLand takeoff_land;
	//参数
	int pose_solver;
	double mass;
	double gra;
	double max_angle;
	double ctrl_freq_max;
	double max_manual_vel;
	double low_voltage;

	bool use_bodyrate_ctrl;
	// bool print_dbg;
	//成员函数
	Parameter_t();//构造函数
	void config_from_ros_handle(const ros::NodeHandle &nh);
	void config_full_thrust(double hov);

private:
	template <typename TName, typename TVal>//使得函数能够传入多种类型参数
	void read_essential_param(const ros::NodeHandle &nh, const TName &name, TVal &val)
	{
        //如果能读取到这个参数
		if (nh.getParam(name, val))
		{
			// pass
		}
        //否则抛出错误
		else
		{
			ROS_ERROR_STREAM("Read param: " << name << " failed.");
			ROS_BREAK();
		}
	};
};
```

### PX4CtrlParam.cpp

```c++
#include "PX4CtrlParam.h"
```

构造函数初始化这个类

```
Parameter_t::Parameter_t()
{
}
```

读取参数

```c++
void Parameter_t::config_from_ros_handle(const ros::NodeHandle &nh)
{
	//直接从ros参数表中读取参数
	read_essential_param(nh, "gain/Kp0", gain.Kp0);
	read_essential_param(nh, "gain/Kp1", gain.Kp1);
	read_essential_param(nh, "gain/Kp2", gain.Kp2);
	read_essential_param(nh, "gain/Kv0", gain.Kv0);
	read_essential_param(nh, "gain/Kv1", gain.Kv1);
	read_essential_param(nh, "gain/Kv2", gain.Kv2);
	read_essential_param(nh, "gain/Kvi0", gain.Kvi0);
	read_essential_param(nh, "gain/Kvi1", gain.Kvi1);
	read_essential_param(nh, "gain/Kvi2", gain.Kvi2);
	read_essential_param(nh, "gain/KAngR", gain.KAngR);
	read_essential_param(nh, "gain/KAngP", gain.KAngP);
	read_essential_param(nh, "gain/KAngY", gain.KAngY);

	read_essential_param(nh, "rotor_drag/x", rt_drag.x);
	read_essential_param(nh, "rotor_drag/y", rt_drag.y);
	read_essential_param(nh, "rotor_drag/z", rt_drag.z);
	read_essential_param(nh, "rotor_drag/k_thrust_horz", rt_drag.k_thrust_horz);

	read_essential_param(nh, "msg_timeout/odom", msg_timeout.odom);
	read_essential_param(nh, "msg_timeout/rc", msg_timeout.rc);
	read_essential_param(nh, "msg_timeout/cmd", msg_timeout.cmd);
	read_essential_param(nh, "msg_timeout/imu", msg_timeout.imu);
	read_essential_param(nh, "msg_timeout/bat", msg_timeout.bat);

	read_essential_param(nh, "pose_solver", pose_solver);
	read_essential_param(nh, "mass", mass);
	read_essential_param(nh, "gra", gra);
	read_essential_param(nh, "ctrl_freq_max", ctrl_freq_max);
	read_essential_param(nh, "use_bodyrate_ctrl", use_bodyrate_ctrl);
	read_essential_param(nh, "max_manual_vel", max_manual_vel);
	read_essential_param(nh, "max_angle", max_angle);
	read_essential_param(nh, "low_voltage", low_voltage);

	read_essential_param(nh, "rc_reverse/roll", rc_reverse.roll);
	read_essential_param(nh, "rc_reverse/pitch", rc_reverse.pitch);
	read_essential_param(nh, "rc_reverse/yaw", rc_reverse.yaw);
	read_essential_param(nh, "rc_reverse/throttle", rc_reverse.throttle);

	read_essential_param(nh, "auto_takeoff_land/enable", takeoff_land.enable);
    read_essential_param(nh, "auto_takeoff_land/enable_auto_arm", takeoff_land.enable_auto_arm);
    read_essential_param(nh, "auto_takeoff_land/no_RC", takeoff_land.no_RC);
	read_essential_param(nh, "auto_takeoff_land/takeoff_height", takeoff_land.height);
	read_essential_param(nh, "auto_takeoff_land/takeoff_land_speed", takeoff_land.speed);

	read_essential_param(nh, "thrust_model/print_value", thr_map.print_val);
	read_essential_param(nh, "thrust_model/K1", thr_map.K1);
	read_essential_param(nh, "thrust_model/K2", thr_map.K2);
	read_essential_param(nh, "thrust_model/K3", thr_map.K3);
	read_essential_param(nh, "thrust_model/accurate_thrust_model", thr_map.accurate_thrust_model);
	read_essential_param(nh, "thrust_model/hover_percentage", thr_map.hover_percentage);
	
	//将角度转换为弧度
	max_angle /= (180.0 / M_PI);
	//如果参数设置中允许自动解锁，但不允许自动降落起飞，则修改为不允许解锁，并抛出错误
	if ( takeoff_land.enable_auto_arm && !takeoff_land.enable )
	{
		takeoff_land.enable_auto_arm = false;
		ROS_ERROR("\"enable_auto_arm\" is only allowd with \"auto_takeoff_land\" enabled.");
	}
    //如果没有遥控器，且不允许自动，则要求必须有遥控器，并抛出错误
	if ( takeoff_land.no_RC && (!takeoff_land.enable_auto_arm || !takeoff_land.enable) )
	{
		takeoff_land.no_RC = false;
		ROS_ERROR("\"no_RC\" is only allowd with both \"auto_takeoff_land\" and \"enable_auto_arm\" enabled.");
	}
	//是否输出参数警告
	if ( thr_map.print_val )
	{
		ROS_WARN("You should disable \"print_value\" if you are in regular usage.");
	}
};
```

计算最大油门

```c++
// void Parameter_t::config_full_thrust(double hov)
// {
// 	full_thrust = mass * gra / hov;
// };
```

### input.h

处理读取数据，`回调函数`

```c++
#include <ros/ros.h>
#include <Eigen/Dense>

#include <sensor_msgs/Imu.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <quadrotor_msgs/TakeoffLand.h>
#include <mavros_msgs/RCIn.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/ExtendedState.h>
#include <sensor_msgs/BatteryState.h>
#include <uav_utils/utils.h>
#include "PX4CtrlParam.h"
```

#### 遥控器数据处理

```c++
class RC_Data_t
{
public:
  double mode;//5通道当前值
  double gear;//6通道当前值
  double reboot_cmd;//8通道当前值
  double last_mode;//上一次
  double last_gear;
  double last_reboot_cmd;
  bool have_init_last_mode{false};//用于判断是否初始化，仅第一次运行时有效
  bool have_init_last_gear{false};
  bool have_init_last_reboot_cmd{false};
  double ch[4];//遥控器遥感的通道值

  mavros_msgs::RCIn msg;//记录遥控器数据
  ros::Time rcv_stamp;//记录时间
  //通道值和阈值比较后的结果，见void RC_Data_t::feed(mavros_msgs::RCInConstPtr pMsg)
  bool is_command_mode;
  bool enter_command_mode;
  bool is_hover_mode;
  bool enter_hover_mode;
  bool toggle_reboot;
  //5，6，8通道的阈值
  static constexpr double GEAR_SHIFT_VALUE = 0.75;
  static constexpr double API_MODE_THRESHOLD_VALUE = 0.75;
  static constexpr double REBOOT_THRESHOLD_VALUE = 0.5;
  static constexpr double DEAD_ZONE = 0.25;		//死区大小

  RC_Data_t();//构造函数
  void check_validity();
  bool check_centered();
  void feed(mavros_msgs::RCInConstPtr pMsg);//遥控器回调函数
  bool is_received(const ros::Time &now_time);
};
```

#### 里程计数据处理

```c++
class Odom_Data_t
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Vector3d p;//姿态信息
  Eigen::Vector3d v;//线速度信息
  Eigen::Quaterniond q;//四元数
  Eigen::Vector3d w;//角速度信息

  nav_msgs::Odometry msg;//里程计消息
  ros::Time rcv_stamp;//记录时间
  bool recv_new_msg;//记录是否收到新的消息

  Odom_Data_t();
  void feed(nav_msgs::OdometryConstPtr pMsg);//里程计回调函数
};
```

#### imu数据处理

```c++
class Imu_Data_t
{
public:
  Eigen::Quaterniond q;//姿态四元数
  Eigen::Vector3d w;//角速度
  Eigen::Vector3d a;//线加速度

  sensor_msgs::Imu msg;//记录imu消息
  ros::Time rcv_stamp;//记录时间

  Imu_Data_t();
  void feed(sensor_msgs::ImuConstPtr pMsg);//imu回调函数
};
```

#### 状态数据处理

```c++
class State_Data_t
{
public:
  mavros_msgs::State current_state;//当前状态
  mavros_msgs::State state_before_offboard;

  State_Data_t();
  void feed(mavros_msgs::StateConstPtr pMsg);
};
```

#### 拓展状态处理

```c++
class ExtendedState_Data_t
{
public:
  mavros_msgs::ExtendedState current_extended_state;//当前状态

  ExtendedState_Data_t();
  void feed(mavros_msgs::ExtendedStateConstPtr pMsg);
};
```

#### 飞行控制指令数据处理

```c++
class Command_Data_t
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Vector3d p;
  Eigen::Vector3d v;
  Eigen::Vector3d a;
  Eigen::Vector3d j;
  double yaw;
  double yaw_rate;

  quadrotor_msgs::PositionCommand msg;
  ros::Time rcv_stamp;

  Command_Data_t();
  void feed(quadrotor_msgs::PositionCommandConstPtr pMsg);
};
```

#### 电池数据处理

```c++
class Battery_Data_t
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  double volt{0.0};//电压
  double percentage{0.0};//dian'l

  sensor_msgs::BatteryState msg;
  ros::Time rcv_stamp;

  Battery_Data_t();
  void feed(sensor_msgs::BatteryStateConstPtr pMsg);
};
```

#### 起飞降落信号处理

```c++
class Takeoff_Land_Data_t
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  bool triggered{false};//标记
  uint8_t takeoff_land_cmd; // see TakeoffLand.msg for its defination //起飞降落话题

  quadrotor_msgs::TakeoffLand msg;
  ros::Time rcv_stamp;

  Takeoff_Land_Data_t();
  void feed(quadrotor_msgs::TakeoffLandConstPtr pMsg);
};
```

### input.cpp

#### 遥控器数据处理

##### RC_Data_t::RC_Data_t()

构造函数

```c++
RC_Data_t::RC_Data_t()
{
    rcv_stamp = ros::Time(0);

    last_mode = -1.0;
    last_gear = -1.0;

    // Parameter initilation is very important in RC-Free usage!
    // 参数初始化在无RC使用中非常重要！
    is_hover_mode = true;
    enter_hover_mode = false;
    is_command_mode = true;
    enter_command_mode = false;
    toggle_reboot = false;
    for (int i = 0; i < 4; ++i)
    {
        ch[i] = 0.0;
    }
}
```

`ros::time(0)和ros::time::now的区别`

![image-20221014114408902](C:\Users\20826\AppData\Roaming\Typora\typora-user-images\image-20221014114408902.png)

##### void RC_Data_t::feed(mavros_msgs::RCInConstPtr pMsg)

遥控器接收回调函数

```c++
void RC_Data_t::feed(mavros_msgs::RCInConstPtr pMsg)
{
    msg = *pMsg;
    rcv_stamp = ros::Time::now();

    for (int i = 0; i < 4; i++)
    {
        //msg.channels[i]值位于1000到2000之间
        //转换到-1到1之间
        ch[i] = ((double)msg.channels[i] - 1500.0) / 500.0;
        //均匀化死区外的数值
        if (ch[i] > DEAD_ZONE)
            ch[i] = (ch[i] - DEAD_ZONE) / (1 - DEAD_ZONE);
        else if (ch[i] < -DEAD_ZONE)
            ch[i] = (ch[i] + DEAD_ZONE) / (1 - DEAD_ZONE);
        //死区内的为0
        else
            ch[i] = 0.0;
    }
	//5,6,8通道归一化
    //5通道代表模式,6通道代表???,8通道代表重启
    mode = ((double)msg.channels[4] - 1000.0) / 1000.0;
    gear = ((double)msg.channels[5] - 1000.0) / 1000.0;
    reboot_cmd = ((double)msg.channels[7] - 1000.0) / 1000.0;
	//TODO
    check_validity();
	//是否初始化上一模式
    if (!have_init_last_mode)
    {
        have_init_last_mode = true;
        //初始化上一模式的值
        last_mode = mode;
    }
    //同上初始化 gear????
    if (!have_init_last_gear)
    {
        have_init_last_gear = true;
        last_gear = gear;
    }
    //初始化重启信号
    if (!have_init_last_reboot_cmd)
    {
        have_init_last_reboot_cmd = true;
        last_reboot_cmd = reboot_cmd;
    }

    // 1
    //如果5通道当前值大于阈值，上一次的值小于阈值，记录
    if (last_mode < API_MODE_THRESHOLD_VALUE && mode > API_MODE_THRESHOLD_VALUE)
        enter_hover_mode = true;
    else
        enter_hover_mode = false;
	//如果5通道当前值大于阈值，记录
    if (mode > API_MODE_THRESHOLD_VALUE)
        is_hover_mode = true;
    else
        is_hover_mode = false;
	
    // 2
    //如过5通道当前值大于阈值
    if (is_hover_mode)
    {
        //如果6通道当前值大于阈值，上一次的值小于阈值，记录
        if (last_gear < GEAR_SHIFT_VALUE && gear > GEAR_SHIFT_VALUE)
            enter_command_mode = true;
        else if (gear < GEAR_SHIFT_VALUE)
            enter_command_mode = false;
		//如果6通道当前值大于阈值，记录
        if (gear > GEAR_SHIFT_VALUE)
            is_command_mode = true;
        else
            is_command_mode = false;
    }

    // 3
    //如过5、6通道当前值均小于阈值
    if (!is_hover_mode && !is_command_mode)
    {
        //如果8通道当前值大于阈值，上一次的值小于阈值，记录
        if (last_reboot_cmd < REBOOT_THRESHOLD_VALUE && reboot_cmd > REBOOT_THRESHOLD_VALUE)
            toggle_reboot = true;
        else
            toggle_reboot = false;
    }
    else
        toggle_reboot = false;
	//将这一次值记录，便于下次使用
    last_mode = mode;
    last_gear = gear;
    last_reboot_cmd = reboot_cmd;
}
```

##### void RC_Data_t::check_validity()

判断遥控器输入数据是否在正常范围内，如果不在正常范围内，抛出错误，并输出对应值

```c++
void RC_Data_t::check_validity()
{
    if (mode >= -1.1 && mode <= 1.1 && gear >= -1.1 && gear <= 1.1 && reboot_cmd >= -1.1 && reboot_cmd <= 1.1)
    {
        // pass
    }
    else
    {
        ROS_ERROR("RC data validity check fail. mode=%f, gear=%f, reboot_cmd=%f", mode, gear, reboot_cmd);
    }
}
```

##### bool RC_Data_t::check_centered()

检测拨杆的值是否在中心（可能有问题）

```c++
bool RC_Data_t::check_centered()
{
    bool centered = abs(ch[0]) < 1e-5 && abs(ch[0]) < 1e-5 && abs(ch[0]) < 1e-5 && abs(ch[0]) < 1e-5;
    return centered;
}
```

#### 里程计数据处理

##### Odom_Data_t::Odom_Data_t()

构造函数里程计数据

```c++
Odom_Data_t::Odom_Data_t()
{
    rcv_stamp = ros::Time(0); //初始化时间
    q.setIdentity();//初始化四元数组
    recv_new_msg = false;//标记未收到消息
};
```

##### void Odom_Data_t::feed(nav_msgs::OdometryConstPtr pMsg)

```c++
void Odom_Data_t::feed(nav_msgs::OdometryConstPtr pMsg)
{
    ros::Time now = ros::Time::now();
	//记录新的消息
    msg = *pMsg;
    //记录当前时间
    rcv_stamp = now;
    //标记收到了新的消息
    recv_new_msg = true;
	//取出消息中的，线加速度，线速度，四元数，角速度
    uav_utils::extract_odometry(pMsg, p, v, q, w);

#define VEL_IN_BODY 
#ifdef VEL_IN_BODY /* Set to 1 if the velocity in odom topic is relative to current body frame, not to world frame.*/
    //如果 odom 主题中的速度相对于当前正文帧，而不是相对于世界帧，则设置为 1。
    //取出消息中的姿态四元数
    Eigen::Quaternion<double> wRb_q(msg.pose.pose.orientation.w, msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z);
    //将四元数转换为旋转矩阵
    Eigen::Matrix3d wRb = wRb_q.matrix();
    //将局部坐标系的速度转换为全局坐标系的速度
    v = wRb * v;
    //提示速度时局部坐标系
    static int count = 0;
    if (count++ % 500 == 0)
        ROS_WARN("VEL_IN_BODY!!!");
#endif

    // check the frequency
    //检测频率
    static int one_min_count = 9999;
    //记录开始检测的时间
    static ros::Time last_clear_count_time = ros::Time(0.0);
    //如果开始检测的时间过去了1秒
    if ( (now - last_clear_count_time).toSec() > 1.0 )
    {
        //如果一秒内触发回调函数的次数小于100，给出里程计频率过低的警告
        if ( one_min_count < 100 )
        {
            ROS_WARN("ODOM frequency seems lower than 100Hz, which is too low!");
        }
        //开始下一秒检测
        one_min_count = 0;
        last_clear_count_time = now;
    }
    //每次触发回调函数加1
    one_min_count ++;
}
```

#### imu数据处理

##### Imu_Data_t::Imu_Data_t()

```c++
Imu_Data_t::Imu_Data_t()
{
    rcv_stamp = ros::Time(0);//初始化时间
}
```

##### void Imu_Data_t::feed(sensor_msgs::ImuConstPtr pMsg)

```c++
void Imu_Data_t::feed(sensor_msgs::ImuConstPtr pMsg)
{
    ros::Time now = ros::Time::now();
	//记录新消息
    msg = *pMsg;
    //记录时间
    rcv_stamp = now;
	//记录角速度
    w(0) = msg.angular_velocity.x;
    w(1) = msg.angular_velocity.y;
    w(2) = msg.angular_velocity.z;
	//记录线加速度
    a(0) = msg.linear_acceleration.x;
    a(1) = msg.linear_acceleration.y;
    a(2) = msg.linear_acceleration.z;
	//记录姿态四元数
    q.x() = msg.orientation.x;
    q.y() = msg.orientation.y;
    q.z() = msg.orientation.z;
    q.w() = msg.orientation.w;
	//检查频率，同里程计
    // check the frequency
    static int one_min_count = 9999;
    static ros::Time last_clear_count_time = ros::Time(0.0);
    if ( (now - last_clear_count_time).toSec() > 1.0 )
    {
        if ( one_min_count < 100 )
        {
            ROS_WARN("IMU frequency seems lower than 100Hz, which is too low!");
        }
        one_min_count = 0;
        last_clear_count_time = now;
    }
    one_min_count ++;
}
```

#### 状态数据处理

```c++
State_Data_t::State_Data_t()
{
}

void State_Data_t::feed(mavros_msgs::StateConstPtr pMsg)
{
    current_state = *pMsg;//记录消息
}
```

#### 拓展状态处理

```c++
ExtendedState_Data_t::ExtendedState_Data_t()
{
}

void ExtendedState_Data_t::feed(mavros_msgs::ExtendedStateConstPtr pMsg)
{
    current_extended_state = *pMsg;//记录消息
}
```

#### 飞行控制指令数据处理

```c++
Command_Data_t::Command_Data_t()
{
    rcv_stamp = ros::Time(0);//初始化时间
}

void Command_Data_t::feed(quadrotor_msgs::PositionCommandConstPtr pMsg)
{
	
    msg = *pMsg;
    rcv_stamp = ros::Time::now();//记录当前时间
	//记录位置
    p(0) = msg.position.x;
    p(1) = msg.position.y;
    p(2) = msg.position.z;
	//记录速度
    v(0) = msg.velocity.x;
    v(1) = msg.velocity.y;
    v(2) = msg.velocity.z;
	//记录线加速度
    a(0) = msg.acceleration.x;
    a(1) = msg.acceleration.y;
    a(2) = msg.acceleration.z;
	//？？？？
    j(0) = msg.jerk.x;
    j(1) = msg.jerk.y;
    j(2) = msg.jerk.z;

    // std::cout << "j1=" << j.transpose() << std::endl;
	//记录偏航角
    yaw = uav_utils::normalize_angle(msg.yaw);
    //记录偏航角速度
    yaw_rate = msg.yaw_dot;
}

```

#### 电池数据处理

```c++
Battery_Data_t::Battery_Data_t()
{
    rcv_stamp = ros::Time(0);//初始化时间
}

void Battery_Data_t::feed(sensor_msgs::BatteryStateConstPtr pMsg)
{

    msg = *pMsg;
    rcv_stamp = ros::Time::now();//记录当前时间

    double voltage = 0;//电压
    //取出每一块电池电压求和
    for (size_t i = 0; i < pMsg->cell_voltage.size(); ++i)
    {
        voltage += pMsg->cell_voltage[i];
    }
    //一阶低通滤波器，减小电压误差带来的波动
    volt = 0.8 * volt + 0.2 * voltage; // Naive LPF, cell_voltage has a higher frequency

    // volt = 0.8 * volt + 0.2 * pMsg->voltage; // Naive LPF
    //获取电量百分比
    percentage = pMsg->percentage;

    static ros::Time last_print_t = ros::Time(0);
    //如果电量百分比大于无，每隔10s输出一次数据
    if (percentage > 0.05)
    {
        if ((rcv_stamp - last_print_t).toSec() > 10)
        {
            ROS_INFO("[px4ctrl] Voltage=%.3f, percentage=%.3f", volt, percentage);
            last_print_t = rcv_stamp;
        }
    }
    //否则每个1秒输入电压过低的警告
    else
    {
        if ((rcv_stamp - last_print_t).toSec() > 1)
        {
            // ROS_ERROR("[px4ctrl] Dangerous! voltage=%.3f, percentage=%.3f", volt, percentage);
            last_print_t = rcv_stamp;
        }
    }
}
```

#### 起飞降落信号处理

```c++
Takeoff_Land_Data_t::Takeoff_Land_Data_t()
{
    rcv_stamp = ros::Time(0);//初始化时间
}

void Takeoff_Land_Data_t::feed(quadrotor_msgs::TakeoffLandConstPtr pMsg)
{

    msg = *pMsg;
    rcv_stamp = ros::Time::now();//记录当前时间

    triggered = true;//是否触发过起飞降落
    takeoff_land_cmd = pMsg->takeoff_land_cmd;//记录指令
}
```

### controller.h

```c++
#include <mavros_msgs/AttitudeTarget.h>
#include <quadrotor_msgs/Px4ctrlDebug.h>
#include <queue>

#include "input.h"
#include <Eigen/Dense>
```

创建期望状态结构体

```c++
struct Desired_State_t
{
    //存储各种姿态
	Eigen::Vector3d p;
	Eigen::Vector3d v;
	Eigen::Vector3d a;
	Eigen::Vector3d j;
	Eigen::Quaterniond q;
	double yaw;
	double yaw_rate;
	//声明结构体的构造函数
	Desired_State_t(){};
	//初始值通过input.h中的里程计数据获取,其余状态均设置为0
	Desired_State_t(Odom_Data_t &odom)
		: p(odom.p),
		  v(Eigen::Vector3d::Zero()),
		  a(Eigen::Vector3d::Zero()),
		  j(Eigen::Vector3d::Zero()),
		  q(odom.q),
		  yaw(uav_utils::get_yaw_from_quaternion(odom.q)),
		  yaw_rate(0){};
};
```

创建控制器的输出结构体

```c++
struct Controller_Output_t
{

	// Orientation of the body frame with respect to the world frame
    //机体坐标系相对于世界坐标系的旋转四元数
	Eigen::Quaterniond q;

	// Body rates in body frame
    //机体坐标系下三轴的旋转速度
	Eigen::Vector3d bodyrates; // [rad/s]

	// Collective mass normalized thrust
    //归一化的推力
	double thrust;

	//Eigen::Vector3d des_v_real;
};
```

创建线性控制器的类

```c++
class LinearControl
{
public:
  LinearControl(Parameter_t &);//线性控制器的构造函数,输入为参数文件导入
  //控制部分输入为里程计和imu，以及声明的两个结构体
  quadrotor_msgs::Px4ctrlDebug calculateControl(const Desired_State_t &des,
      const Odom_Data_t &odom,
      const Imu_Data_t &imu, 
      Controller_Output_t &u);
  //估计油门的模块
  bool estimateThrustModel(const Eigen::Vector3d &est_v,
      const Parameter_t &param);
  //重置推力
  void resetThrustMapping(void);

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
  //参数
  Parameter_t param_;
  quadrotor_msgs::Px4ctrlDebug debug_msg_;
  std::queue<std::pair<ros::Time, double>> timed_thrust_;
  static constexpr double kMinNormalizedCollectiveThrust_ = 3.0;

  // Thrust-accel mapping params
  const double rho2_ = 0.998; // do not change
  double thr2acc_;
  double P_;

  double computeDesiredCollectiveThrustSignal(const Eigen::Vector3d &des_acc);
  //从四元数到偏航角
  double fromQuaternion2yaw(Eigen::Quaterniond q);
};
```

### controller.cpp

```c++
double LinearControl::fromQuaternion2yaw(Eigen::Quaterniond q)//从四元数转为偏航角
{
  double yaw = atan2(2 * (q.x()*q.y() + q.w()*q.z()), q.w()*q.w() + q.x()*q.x() - q.y()*q.y() - q.z()*q.z());
  return yaw;
}
```

```c++
LinearControl::LinearControl(Parameter_t &param) : param_(param)//构造函数，通过param_初始化参数
{
  resetThrustMapping();
}
void 
LinearControl::resetThrustMapping(void)
{
  thr2acc_ = param_.gra / param_.thr_map.hover_percentage;//重力加速度/悬停油门百分比
  P_ = 1e6;
}
```

```c++
/* 
  compute u.thrust and u.q, controller gains and other parameters are in param_
  计算参数
*/
quadrotor_msgs::Px4ctrlDebug
LinearControl::calculateControl(const Desired_State_t &des,
    const Odom_Data_t &odom,
    const Imu_Data_t &imu, 
    Controller_Output_t &u)
{
  /* WRITE YOUR CODE HERE */
      //compute disired acceleration
      //计算所需加速度
      Eigen::Vector3d des_acc(0.0, 0.0, 0.0);//初始化加速矩阵
      Eigen::Vector3d Kp,Kv;
      Kp << param_.gain.Kp0, param_.gain.Kp1, param_.gain.Kp2;
      Kv << param_.gain.Kv0, param_.gain.Kv1, param_.gain.Kv2;//初始化Kp，Kv
      des_acc = des.a + Kv.asDiagonal() * (des.v - odom.v) + Kp.asDiagonal() * (des.p - odom.p);
      des_acc += Eigen::Vector3d(0,0,param_.gra);//z轴加速度还需要加上重力加速度

      u.thrust = computeDesiredCollectiveThrustSignal(des_acc);//将加速度计算为油门
      double roll,pitch,yaw,yaw_imu;
      double yaw_odom = fromQuaternion2yaw(odom.q);//从里程计数据中获取偏航角
      double sin = std::sin(yaw_odom);
      double cos = std::cos(yaw_odom);
      roll = (des_acc(0) * sin - des_acc(1) * cos )/ param_.gra;
      pitch = (des_acc(0) * cos + des_acc(1) * sin )/ param_.gra;
      // yaw = fromQuaternion2yaw(des.q);
      yaw_imu = fromQuaternion2yaw(imu.q);
      // Eigen::Quaterniond q = Eigen::AngleAxisd(yaw,Eigen::Vector3d::UnitZ())
      //   * Eigen::AngleAxisd(roll,Eigen::Vector3d::UnitX())
      //   * Eigen::AngleAxisd(pitch,Eigen::Vector3d::UnitY());
      Eigen::Quaterniond q = Eigen::AngleAxisd(des.yaw,Eigen::Vector3d::UnitZ())
        * Eigen::AngleAxisd(pitch,Eigen::Vector3d::UnitY())
        * Eigen::AngleAxisd(roll,Eigen::Vector3d::UnitX());
      u.q = imu.q * odom.q.inverse() * q;
	  //计算预计旋转

  /* WRITE YOUR CODE HERE */

  //used for debug
  // debug_msg_.des_p_x = des.p(0);
  // debug_msg_.des_p_y = des.p(1);
  // debug_msg_.des_p_z = des.p(2);
  
  debug_msg_.des_v_x = des.v(0);
  debug_msg_.des_v_y = des.v(1);
  debug_msg_.des_v_z = des.v(2);
  
  debug_msg_.des_a_x = des_acc(0);
  debug_msg_.des_a_y = des_acc(1);
  debug_msg_.des_a_z = des_acc(2);
  
  debug_msg_.des_q_x = u.q.x();
  debug_msg_.des_q_y = u.q.y();
  debug_msg_.des_q_z = u.q.z();
  debug_msg_.des_q_w = u.q.w();
  
  debug_msg_.des_thr = u.thrust;
  
  // Used for thrust-accel mapping estimation
  //保存推力和对应时间
  timed_thrust_.push(std::pair<ros::Time, double>(ros::Time::now(), u.thrust));
  //如果队列里数量超过100则将以前的出队
  while (timed_thrust_.size() > 100)
  {
    timed_thrust_.pop();
  }
  return debug_msg_;
}
```

```c++
double 
LinearControl::computeDesiredCollectiveThrustSignal(
    const Eigen::Vector3d &des_acc)
{
  //由加速度计算油门
  double throttle_percentage(0.0);
  
  /* compute throttle, thr2acc has been estimated before */
  throttle_percentage = des_acc(2) / thr2acc_;

  return throttle_percentage;
}
```

```c++
bool 
LinearControl::estimateThrustModel(
    const Eigen::Vector3d &est_a,
    const Parameter_t &param)
{
  ros::Time t_now = ros::Time::now();
  //如果队列里存在数据
  while (timed_thrust_.size() >= 1)
  {
    // Choose data before 35~45ms ago
    //取出队列第一个
    std::pair<ros::Time, double> t_t = timed_thrust_.front();
    
    //计算这一个推力的时间点和现在时间的差值
    double time_passed = (t_now - t_t.first).toSec();
   
    //如果差值大于45ms
    if (time_passed > 0.045) // 45ms
    {
        
      //则出队,表示丢弃过时的推力
      // printf("continue, time_passed=%f\n", time_passed);
      timed_thrust_.pop();
      continue;
    }
    //如果差值小于35ms
    if (time_passed < 0.035) // 35ms
    {
      //表示时间还没有到
      // printf("skip, time_passed=%f\n", time_passed);
      return false;
    }

    /***********************************************************/
    /* Recursive least squares algorithm with vanishing memory */
    /*                具有消失内存的递归最小二乘算法                 */
    /***********************************************************/
    //取出时间后出队
    double thr = t_t.second;
    timed_thrust_.pop();
    
    /***********************************/
    /* Model: est_a(2) = thr1acc_ * thr */
    /***********************************/
    //计算can'shu
    double gamma = 1 / (rho2_ + thr * P_ * thr);
    double K = gamma * P_ * thr;
    thr2acc_ = thr2acc_ + K * (est_a(2) - thr * thr2acc_);
    P_ = (1 - K * thr) * P_ / rho2_;
    //printf("%6.3f,%6.3f,%6.3f,%6.3f\n", thr2acc_, gamma, K, P_);
    //fflush(stdout);

    // debug_msg_.thr2acc = thr2acc_;
    return true;
  }
  return false;
}
```

### PX4CtrlFSM.cpp

```c++
#include "PX4CtrlFSM.h"
#include <uav_utils/converters.h>

using namespace std;
using namespace uav_utils;
```

构造函数，创建控制器

```c++
PX4CtrlFSM::PX4CtrlFSM(Parameter_t &param_, LinearControl &controller_) : param(param_), controller(controller_) /*, thrust_curve(thrust_curve_)*/
{
	state = MANUAL_CTRL;
	hover_pose.setZero();
}
```

控制流程

```c++
/* 
        Finite State Machine

	      system start
	            |
	            |
	            v
	----- > MANUAL_CTRL <-----------------
	|         ^   |    \                 |
	|         |   |     \                |
	|         |   |      > AUTO_TAKEOFF  |
	|         |   |        /             |
	|         |   |       /              |
	|         |   |      /               |
	|         |   v     /                |
	|       AUTO_HOVER <                 |
	|         ^   |  \  \                |
	|         |   |   \  \               |
	|         |	  |    > AUTO_LAND -------
	|         |   |
	|         |   v
	-------- CMD_CTRL

*/
```

#### void PX4CtrlFSM::process()

开始控制进程

```C++
	ros::Time now_time = ros::Time::now();//记录时间
	Controller_Output_t u;//创建控制器输出
	Desired_State_t des(odom_data);//调用构造函数（在controller中）
	bool rotor_low_speed_during_land = false;//标记？？？
```

对state进行判断

##### MANUAL_CTRL

尝试跳出遥控器控制模式

```c++
		//读取遥控器数据，对应如果5通道当前值大于阈值，上一次的值小于阈值。尝试跳转导自动模式
		if (rc_data.enter_hover_mode) // Try to jump to AUTO_HOVER
		{
            //使用当前时间查询里程计是否存在，如果不存在发出警告，并且退出这次switch判断
			if (!odom_is_received(now_time))
			{
				ROS_ERROR("[px4ctrl] Reject AUTO_HOVER(L2). No odom!");
				break;
			}
            //同上查询控制指令
			if (cmd_is_received(now_time))
			{
				ROS_ERROR("[px4ctrl] Reject AUTO_HOVER(L2). You are sending commands before toggling into AUTO_HOVER, which is not allowed. Stop sending commands now!");
				break;
			}
            //判断里程计数据是否在合理的启动速度范围内
			if (odom_data.v.norm() > 3.0)
			{
				ROS_ERROR("[px4ctrl] Reject AUTO_HOVER(L2). Odom_Vel=%fm/s, which seems that the locolization module goes wrong!", odom_data.v.norm());
				break;
			}
			//如果上述条件满足则准备跳转到自动模式
			state = AUTO_HOVER;
            //调用controller重置油门
			controller.resetThrustMapping();
            //？？？
			set_hov_with_odom();
			toggle_offboard_mode(true);
			//给出模式跳转提示
			ROS_INFO("\033[32m[px4ctrl] MANUAL_CTRL(L1) --> AUTO_HOVER(L2)\033[32m");
		}
```

如果跳转自动模式失败，开始尝试跳转到

```c++
		//如参数中允许自动起飞，且input中收到了起飞指令，尝试自动起飞模式
		else if (param.takeoff_land.enable && takeoff_land_data.triggered && takeoff_land_data.takeoff_land_cmd == quadrotor_msgs::TakeoffLand::TAKEOFF) // Try to jump to AUTO_TAKEOFF
		{
            //同上判断里程计消息状态
			if (!odom_is_received(now_time))
			{
				ROS_ERROR("[px4ctrl] Reject AUTO_TAKEOFF. No odom!");
				break;
			}
            //判断控制指令状态
			if (cmd_is_received(now_time))
			{
				ROS_ERROR("[px4ctrl] Reject AUTO_TAKEOFF. You are sending commands before toggling into AUTO_TAKEOFF, which is not allowed. Stop sending commands now!");
				break;
			}
            //判断当前速度是否基本静止
			if (odom_data.v.norm() > 0.1)
			{
				ROS_ERROR("[px4ctrl] Reject AUTO_TAKEOFF. Odom_Vel=%fm/s, non-static takeoff is not allowed!", odom_data.v.norm());
				break;
			}
            //检测是否在地面
			if (!get_landed())
			{
				ROS_ERROR("[px4ctrl] Reject AUTO_TAKEOFF. land detector says that the drone is not landed now!");
				break;
			}
            //检测遥控器是否连接
			if (rc_is_received(now_time)) // Check this only if RC is connected.
			{
                //要求5通道6通道的值大于阈值，且拨杆的值正常（5通道代表自动？6通道代表指令控制）
				if (!rc_data.is_hover_mode || !rc_data.is_command_mode || !rc_data.check_centered())
				{
					ROS_ERROR("[px4ctrl] Reject AUTO_TAKEOFF. If you have your RC connected, keep its switches at \"auto hover\" and \"command control\" states, and all sticks at the center, then takeoff again.");
                    //等待遥控器进入要求状态，会一直卡在此处
					while (ros::ok())
					{
						ros::Duration(0.01).sleep();
						ros::spinOnce();
						if (rc_data.is_hover_mode && rc_data.is_command_mode && rc_data.check_centered())
						{
							ROS_INFO("\033[32m[px4ctrl] OK, you can takeoff again.\033[32m");
							break;
						}
					}
					break;
				}
			}
			//满足以上要求后准备进入自动起飞状态
			state = AUTO_TAKEOFF;
            //重置油门
			controller.resetThrustMapping();
            //使用里程计数据初始化
			set_start_pose_for_takeoff_land(odom_data);
            //在解锁状态前切换到offboard模式
			toggle_offboard_mode(true);				  // toggle on offboard before arm
            //等待0.1秒用于飞控切换模式
			for (int i = 0; i < 10 && ros::ok(); ++i) // wait for 0.1 seconds to allow mode change by FMU // mark
			{
				ros::Duration(0.01).sleep();
				ros::spinOnce();
			}
            //查看参数表中是否允许自动起飞
			if (param.takeoff_land.enable_auto_arm)
			{
                //解锁
				toggle_arm_disarm(true);
			}
            //记录起飞时间
			takeoff_land.toggle_takeoff_land_time = now_time;
			//给出模式跳转提示
			ROS_INFO("\033[32m[px4ctrl] MANUAL_CTRL(L1) --> AUTO_TAKEOFF\033[32m");
		}
```

如果遥控器触发8通道，开始尝试重启

```c++
		if (rc_data.toggle_reboot) // Try to reboot. EKF2 based PX4 FCU requires reboot when its state estimator goes wrong.
		{
            //检测状态中是否上锁，如果不上锁开始重启
			if (state_data.current_state.armed)
			{
				ROS_ERROR("[px4ctrl] Reject reboot! Disarm the drone first!");
				break;
			}
			reboot_FCU();
		}
```

##### AUTO_HOVER

```c++
		//检测遥控器5通道是否高于阈值，里程计是否有数据
		if (!rc_data.is_hover_mode || !odom_is_received(now_time))
		{
            //如果不满足条件则跳出自动模式，并结束offoard模式
			state = MANUAL_CTRL;
			toggle_offboard_mode(false);

			ROS_WARN("[px4ctrl] AUTO_HOVER(L2) --> MANUAL_CTRL(L1)");
		}
		//检测遥控器六通道是否高于阈值，允许指令控制，检测是否接收到指令
		else if (rc_data.is_command_mode && cmd_is_received(now_time))
		{
            //如果满足条件，且处于offoard模式
			if (state_data.current_state.mode == "OFFBOARD")
			{
                //将模式切换到指令控制模式
				state = CMD_CTRL;
				des = get_cmd_des();
				ROS_INFO("\033[32m[px4ctrl] AUTO_HOVER(L2) --> CMD_CTRL(L3)\033[32m");
			}
		}
		//检测起飞标记是否激活，并且接受到起飞降落信号
		else if (takeoff_land_data.triggered && takeoff_land_data.takeoff_land_cmd == quadrotor_msgs::TakeoffLand::LAND)
		{
			//进入自动降落模式，初始化降落位姿
			state = AUTO_LAND;
			set_start_pose_for_takeoff_land(odom_data);

			ROS_INFO("\033[32m[px4ctrl] AUTO_HOVER(L2) --> AUTO_LAND\033[32m");
		}
		//如果以上都没有满足
		else
		{
            //使用遥控器控制悬停
			set_hov_with_rc();
			des = get_hover_des();
            //如果遥控器6通道满足条件，或者当前时间大于起飞时间
			if ((rc_data.enter_command_mode) ||
				(takeoff_land.delay_trigger.first && now_time > takeoff_land.delay_trigger.second))
			{
				takeoff_land.delay_trigger.first = false;
                //发布里程计数据
				publish_trigger(odom_data.msg);
				ROS_INFO("\033[32m[px4ctrl] TRIGGER sent, allow user command.\033[32m");
			}

			// cout << "des.p=" << des.p.transpose() << endl;
		}
```

##### CMD_CTRL

```c++
		//同上模式切换
		if (!rc_data.is_hover_mode || !odom_is_received(now_time))
		{
			state = MANUAL_CTRL;
			toggle_offboard_mode(false);

			ROS_WARN("[px4ctrl] From CMD_CTRL(L3) to MANUAL_CTRL(L1)!");
		}
		else if (!rc_data.is_command_mode || !cmd_is_received(now_time))
		{
			state = AUTO_HOVER;
			set_hov_with_odom();
			des = get_hover_des();
			ROS_INFO("[px4ctrl] From CMD_CTRL(L3) to AUTO_HOVER(L2)!");
		}
		else
		{
			des = get_cmd_des();
		}
		//检测起飞指令是否按要求顺序发布
		if (takeoff_land_data.triggered && takeoff_land_data.takeoff_land_cmd == quadrotor_msgs::TakeoffLand::LAND)
		{
			ROS_ERROR("[px4ctrl] Reject AUTO_LAND, which must be triggered in AUTO_HOVER. \
					Stop sending control commands for longer than %fs to let px4ctrl return to AUTO_HOVER first.",
					  param.msg_timeout.cmd);
		}
```

##### AUTO_TAKEOFF

```c++
		//等待电机初始化
		if ((now_time - takeoff_land.toggle_takeoff_land_time).toSec() < AutoTakeoffLand_t::MOTORS_SPEEDUP_TIME) // Wait for several seconds to warn prople.
		{
			des = get_rotor_speed_up_des(now_time);
		}
		//如果当前高度大于初始起飞高度加目标起飞高度
		else if (odom_data.p(2) >= (takeoff_land.start_pose(2) + param.takeoff_land.height)) // reach the desired height
		{
            //跳转回自动模式
			state = AUTO_HOVER;
			set_hov_with_odom();
			ROS_INFO("\033[32m[px4ctrl] AUTO_TAKEOFF --> AUTO_HOVER(L2)\033[32m");
			//标记起飞记录起飞时间
			takeoff_land.delay_trigger.first = true;
			takeoff_land.delay_trigger.second = now_time + ros::Duration(AutoTakeoffLand_t::DELAY_TRIGGER_TIME);
		}
		//请求起飞
		else
		{
			des = get_takeoff_land_des(param.takeoff_land.speed);
		}
```

##### AUTO_LAND

```c++
		//同上模式切换
		if (!rc_data.is_hover_mode || !odom_is_received(now_time))
		{
			state = MANUAL_CTRL;
			toggle_offboard_mode(false);

			ROS_WARN("[px4ctrl] From AUTO_LAND to MANUAL_CTRL(L1)!");
		}
		else if (!rc_data.is_command_mode)
		{
			state = AUTO_HOVER;
			set_hov_with_odom();
			des = get_hover_des();
			ROS_INFO("[px4ctrl] From AUTO_LAND to AUTO_HOVER(L2)!");
		}
		//检测是否在地面
		else if (!get_landed())
		{
            //如果不在地面，使用起飞速度相反数请求降落
			des = get_takeoff_land_des(-param.takeoff_land.speed);
		}
		else
		{
			rotor_low_speed_during_land = true;
			//只打印一次输出
			static bool print_once_flag = true;
			if (print_once_flag)
			{
				ROS_INFO("\033[32m[px4ctrl] Wait for abount 10s to let the drone arm.\033[32m");
				print_once_flag = false;
			}
			//锁上飞机
			if (extended_state_data.current_extended_state.landed_state == mavros_msgs::ExtendedState::LANDED_STATE_ON_GROUND) // PX4 allows disarm after this
			{
				static double last_trial_time = 0; // Avoid too frequent calls
				if (now_time.toSec() - last_trial_time > 1.0)
				{
                    //如果锁上飞机，推出降落模式
					if (toggle_arm_disarm(false)) // disarm
					{
						print_once_flag = true;
						state = MANUAL_CTRL;
						toggle_offboard_mode(false); // toggle off offboard after disarm
						ROS_INFO("\033[32m[px4ctrl] AUTO_LAND --> MANUAL_CTRL(L1)\033[32m");
					}

					last_trial_time = now_time.toSec();
				}
			}
		}
```

##### 其他控制

```c++
	//估计油门模式
// STEP2: estimate thrust model
	if (state == AUTO_HOVER || state == CMD_CTRL)
	{
		// controller.estimateThrustModel(imu_data.a, bat_data.volt, param);
		controller.estimateThrustModel(imu_data.a,param);

	}
	
	if (rotor_low_speed_during_land) // used at the start of auto takeoff
	{
		motors_idling(imu_data, u);
	}
	else
	{
		debug_msg = controller.calculateControl(des, odom_data, imu_data, u);
		debug_msg.header.stamp = now_time;
		debug_pub.publish(debug_msg);
	}
	//向mavros发布控制指令
	// STEP4: publish control commands to mavros
	if (param.use_bodyrate_ctrl)
	{
		publish_bodyrate_ctrl(u, now_time);
	}
	else
	{
		publish_attitude_ctrl(u, now_time);
	}
	
	// STEP5: Detect if the drone has landed
	land_detector(state, des, odom_data);
	// cout << takeoff_land.landed << " ";
	// fflush(stdout);
	//结束后清楚标志位
	// STEP6: Clear flags beyound their lifetime
	rc_data.enter_hover_mode = false;
	rc_data.enter_command_mode = false;
	rc_data.toggle_reboot = false;
	takeoff_land_data.triggered = false;
```

