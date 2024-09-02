#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

// 回调函数，用于处理接收到的点云数据
void lidarDataListenerCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg) {
    ROS_INFO("Received point cloud with width: %d, height: %d", cloud_msg->width, cloud_msg->height);
    // 这里可以添加更多的点云处理逻辑
}

int main(int argc, char** argv) {
    // 初始化ROS节点
    ros::init(argc, argv, "lidar_data_listener");
    ros::NodeHandle nh;

    // 订阅点云话题
    ros::Subscriber sub = nh.subscribe("/livox/lidar", 1000, lidarDataListenerCallback);

    // 循环等待回调
    ros::spin();

    return 0;
}
