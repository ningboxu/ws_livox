#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include "pointcloud_processor.h"

PointCloudProcessor processor(11, 10);  // 设置最大保存次数和每组累积10帧点云
int frame_count = 0;

void lidarDataListenerCallback(
    const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(
        new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*cloud_msg, *cloud);

    // 计算发布频率
    processor.calculatePublishFrequency();

    // 累积点云
    processor.accumulatePointCloud(cloud);
    ROS_INFO("Received point cloud with %lu points", cloud->size());

    frame_count++;

    // 如果达到组帧数量，保存点云
    if (frame_count % 10 == 0)
    {
        processor.savePointCloud(frame_count / 10);  // 保存累积的组帧点云
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
