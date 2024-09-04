#include <pcl/io/pcd_io.h>
#include <ros/ros.h>
#include <ctime>
#include <iomanip>
#include <sstream>
#include "pointcloud_processor.h"

PointCloudProcessor::PointCloudProcessor(int max_saves, int group_size)
    : max_saves_(max_saves)
    , group_size_(group_size)
    , save_count_(0)
    , publish_frequency_(0.0)
{
    accumulated_cloud_ =
        pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    last_timestamp_ = std::chrono::steady_clock::now();
}

PointCloudProcessor::~PointCloudProcessor() {}

void PointCloudProcessor::accumulatePointCloud(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
{
    *accumulated_cloud_ += *cloud;
}

void PointCloudProcessor::savePointCloud(int frame_group)
{
    std::time_t now = std::time(nullptr);
    std::stringstream fileName;
    fileName << "/home/xnb/rosbag_data/livox/test_group/pc_group_"
             << frame_group << "_"
             << std::put_time(std::localtime(&now), "%Y-%m-%d_%H-%M-%S")
             << ".pcd";

    if (pcl::io::savePCDFileASCII(fileName.str(), *accumulated_cloud_) == -1)
    {
        ROS_ERROR("Failed to save accumulated point cloud data to file: %s",
                  fileName.str().c_str());
        return;
    }

    ROS_INFO("Saved accumulated point cloud to file: %s",
             fileName.str().c_str());
    clearAccumulatedCloud();  // 清空当前累积的点云
}

bool PointCloudProcessor::hasReachedMaxSaves()
{
    return save_count_ >= max_saves_;
}

void PointCloudProcessor::clearAccumulatedCloud()
{
    accumulated_cloud_->clear();
}

void PointCloudProcessor::calculatePublishFrequency()
{
    auto current_time = std::chrono::steady_clock::now();
    auto duration     = std::chrono::duration_cast<std::chrono::milliseconds>(
                        current_time - last_timestamp_)
                        .count();
    publish_frequency_ = 1000.0 / duration;  // 转换为 Hz
    ROS_INFO("PointCloud Publish Frequency: %.2f Hz", publish_frequency_);
    last_timestamp_ = current_time;  // 更新时间戳
}
