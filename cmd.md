### cmd

```
cd ws_livox
catkin_make
#更新当前 ROS 包环境
source ./devel/setup.sh
# 运行


roslaunch livox_ros_driver livox_lidar_rviz.launch bd_list:="3GGDLCH0020116"
```







### 查看

```
# rostopic
# 查看发布的点云数据
rostopic echo /livox/lidar
# 查看发布频率
rostopic hz /livox/lidar
```





# 记录

```
# 记录数据
rosbag record /livox/lidar
rosbag record --duration=10 /livox/lidar
rosbag record -O my_livox_data.bag /livox/lidar
```

```
# 转换
rosrun pcl_ros bag_to_pcd 2024-09-02-10-33-35.bag /livox/lidar ~/my_output_directory
```



### 播放

```
rosbag play 2024-08-27-11-28-23.bag --topic /velodyne_points

```

   rostopic echo /cloud | grep frame_id





## 使用livox_sdk保存数据

https://github.com/Livox-SDK/Livox-SDK/wiki/How-to-use-lvx-file-under-ros-cn

```
./lidar_lvx_sample -c "3GGDLCH0020116" -t 10
```

