#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include "pointcloud_processor.h"

PointCloudProcessor processor(10, 3.0);  // 设置最大保存次数和保存间隔

void lidarDataListenerCallback(
    const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(
        new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*cloud_msg, *cloud);

    // 累积点云
    processor.accumulatePointCloud(cloud);
    ROS_INFO("Received point cloud with %lu points", cloud->size());
    // 在保存前对累积的点云进行体素栅格滤波
    processor.applyVoxelGridFilter(0.005f);  // 使用叶大小为 0.05f 的滤波器
    // processor.applyStatisticalOutlierRemoval(10, 1);

    // 检查是否需要保存点云
    if (processor.shouldSave())
    {
        applyStatisticalOutlierRemoval processor.savePointCloud();
    }

    // 检查是否达到最大保存次数
    if (processor.hasReachedMaxSaves())
    {
        ROS_INFO("Reached maximum save count. Shutting down...");
        ros::shutdown();
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "lidar_data_listener");
    ros::NodeHandle nh;

    // 订阅点云话题
    ros::Subscriber sub =
        nh.subscribe("/livox/lidar", 1000, lidarDataListenerCallback);

    // 循环等待回调
    ros::spin();
    return 0;
}
