#include <pcl/conversions.h>
#include <pcl/io/pcd_io.h>  // 包含PCD文件的读写功能
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>  // 包含所需的点云类型定义
#include <pcl_conversions/pcl_conversions.h>  // 添加此头文件
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <chrono>
#include <fstream>
#include <iomanip>  // 用于 std::put_time

void savePointCloud(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
    // 获取当前时间并构建文件名
    std::time_t now = std::time(nullptr);
    std::stringstream fileName;
    fileName << "/home/xnb/rosbag_data/livox/pointcloud_"
             << std::put_time(std::localtime(&now), "%Y-%m-%d_%H-%M-%S")
             << ".pcd";

    // 创建PCL点云对象使用智能指针
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(
        new pcl::PointCloud<pcl::PointXYZ>);

    // 将ROS点云消息转换为PCL点云
    pcl::fromROSMsg(*cloud_msg, *cloud);

    // 保存点云为PCD文件
    if (pcl::io::savePCDFileASCII(fileName.str(), *cloud) == -1)
    {
        ROS_ERROR("Failed to save point cloud data to file: %s",
                  fileName.str().c_str());
        return;
    }

    ROS_INFO("Saved point cloud to file: %s", fileName.str().c_str());
}

// 回调函数，用于处理接收到的点云数据
void lidarDataListenerCallback(
    const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
    ROS_INFO("Received point cloud with width: %d, height: %d",
             cloud_msg->width, cloud_msg->height);

    // 调用保存函数
    savePointCloud(cloud_msg);
}

int main(int argc, char** argv)
{
    // 初始化ROS节点
    ros::init(argc, argv, "lidar_data_listener");
    ros::NodeHandle nh;

    // 订阅点云话题
    ros::Subscriber sub =
        nh.subscribe("/livox/lidar", 1000, lidarDataListenerCallback);

    // 循环等待回调
    ros::spin();

    return 0;
}
