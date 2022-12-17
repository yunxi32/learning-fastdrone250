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

## 调参指令

### imu内参

录包

```
rosbag record -O imu_bagname /mavros/imu/data_raw 
```

#TODO

### 雷达和imu的外参

#TODO
