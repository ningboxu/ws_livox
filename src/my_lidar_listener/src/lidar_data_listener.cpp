#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <chrono>
#include <iomanip>
#include <thread>
#include <vector>

// 全局变量
int save_count = 0;
int max_saves;
double save_interval;
std::chrono::time_point<std::chrono::steady_clock> start_time;
pcl::PointCloud<pcl::PointXYZ>::Ptr accumulated_cloud(
    new pcl::PointCloud<pcl::PointXYZ>);

void saveAccumulatedPointCloud()
{
    // 获取当前时间并构建文件名
    std::time_t now = std::time(nullptr);
    std::stringstream fileName;
    fileName << "/home/xnb/rosbag_data/livox/accumulated_pointcloud_"
             << std::put_time(std::localtime(&now), "%Y-%m-%d_%H-%M-%S")
             << ".pcd";

    // 保存累积的点云为PCD文件
    if (pcl::io::savePCDFileASCII(fileName.str(), *accumulated_cloud) == -1)
    {
        ROS_ERROR("Failed to save accumulated point cloud data to file: %s",
                  fileName.str().c_str());
        return;
    }

    ROS_INFO("Saved accumulated point cloud to file: %s",
             fileName.str().c_str());

    // 清空累积的点云
    accumulated_cloud->clear();

    // 更新保存次数
    save_count++;
}
void lidarDataListenerCallback(
    const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
    // 检查时间是否已超出总时间
    auto elapsed = std::chrono::steady_clock::now() - start_time;
    if (std::chrono::duration<double>(elapsed).count() >
        save_interval * max_saves)
    {
        ROS_INFO("Exceeded total time limit. Stopping...");
        ros::shutdown();
        return;
    }

    // 将ROS点云消息转换为PCL点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(
        new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*cloud_msg, *cloud);

    // 输出当前点云的数量
    ROS_INFO("Received point cloud with %lu points", cloud->size());

    // 拼接点云
    *accumulated_cloud += *cloud;

    // 检查是否到达保存间隔
    if (save_count < max_saves &&
        std::chrono::duration<double>(elapsed).count() >=
            save_interval * (save_count + 1))
    {
        saveAccumulatedPointCloud();
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "lidar_data_listener");
    ros::NodeHandle nh;

    // 设置保存间隔（秒）和最大保存次数
    save_interval = 3;  // 每隔1秒保存一次
    max_saves     = 6;  // 最多保存5次

    // 获取开始时间
    start_time = std::chrono::steady_clock::now();

    // 订阅点云话题
    ros::Subscriber sub =
        nh.subscribe("/livox/lidar", 1000, lidarDataListenerCallback);

    // 循环等待回调
    ros::spin();

    return 0;
}
