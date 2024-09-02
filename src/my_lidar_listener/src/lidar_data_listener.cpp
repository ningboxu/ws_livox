#include <pcl/io/pcd_io.h>  // 包含PCD文件的读写功能
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>  // 包含所需的点云类型定义
#include <pcl_conversions/pcl_conversions.h>  // 用于将ROS消息转换为PCL点云
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <chrono>
#include <fstream>
#include <iomanip>  // 用于 std::put_time
#include <thread>   // 用于 std::this_thread::sleep_for

// 全局变量
int save_count = 0;
int max_saves;
double save_interval;
std::chrono::time_point<std::chrono::steady_clock> start_time;

void savePointCloud(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
    if (save_count >= max_saves)
    {
        ROS_INFO("Reached the maximum save limit. Stopping...");
        ros::shutdown();
        return;
    }

    std::time_t now = std::time(nullptr);
    std::stringstream fileName;
    fileName << "/home/xnb/rosbag_data/livox/pointcloud_"
             << std::put_time(std::localtime(&now), "%Y-%m-%d_%H-%M-%S")
             << ".pcd";

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(
        new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*cloud_msg, *cloud);

    if (pcl::io::savePCDFileASCII(fileName.str(), *cloud) == -1)
    {
        ROS_ERROR("Failed to save point cloud data to file: %s",
                  fileName.str().c_str());
        return;
    }

    ROS_INFO("Saved point cloud to file: %s", fileName.str().c_str());

    save_count++;
}

void lidarDataListenerCallback(
    const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
    auto elapsed = std::chrono::steady_clock::now() - start_time;
    if (std::chrono::duration<double>(elapsed).count() >
        save_interval * max_saves)
    {
        ROS_INFO("Exceeded total time limit. Stopping...");
        ros::shutdown();
        return;
    }

    savePointCloud(cloud_msg);
    std::this_thread::sleep_for(std::chrono::duration<double>(save_interval));
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "lidar_data_listener");
    ros::NodeHandle nh;

    save_interval = 1.0;  // 每隔1秒保存一次
    max_saves     = 5;    // 最多保存5次

    start_time = std::chrono::steady_clock::now();

    ros::Subscriber sub =
        nh.subscribe("/livox/lidar", 1000, lidarDataListenerCallback);

    ros::spin();

    return 0;
}
