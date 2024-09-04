#include <pcl/io/pcd_io.h>
#include <ros/ros.h>
#include <ctime>
#include <iomanip>
#include <sstream>
#include "pointcloud_processor.h"

PointCloudProcessor::PointCloudProcessor(int max_saves, double save_interval)
    : max_saves_(max_saves), save_interval_(save_interval), save_count_(0)
{
    accumulated_cloud_ =
        pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    start_time_ = std::chrono::steady_clock::now();
}

PointCloudProcessor::~PointCloudProcessor() {}

void PointCloudProcessor::accumulatePointCloud(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
{
    *accumulated_cloud_ += *cloud;
}

void PointCloudProcessor::savePointCloud()
{
    std::time_t now = std::time(nullptr);
    std::stringstream fileName;
    fileName << "/home/xnb/rosbag_data/livox/test/pc_"
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
    accumulated_cloud_->clear();
    save_count_++;
}

bool PointCloudProcessor::shouldSave()
{
    auto elapsed = std::chrono::steady_clock::now() - start_time_;
    return std::chrono::duration<double>(elapsed).count() >=
           save_interval_ * (save_count_ + 1);
}

bool PointCloudProcessor::hasReachedMaxSaves()
{
    return save_count_ >= max_saves_;
}

// 体素栅格滤波：减少点云密度，保持关键点特征
void PointCloudProcessor::applyVoxelGridFilter(float leaf_size)
{
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud(accumulated_cloud_);
    sor.setLeafSize(leaf_size, leaf_size, leaf_size);
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(
        new pcl::PointCloud<pcl::PointXYZ>);
    sor.filter(*filtered_cloud);
    accumulated_cloud_ = filtered_cloud;  // 更新累积点云
}
// 统计去噪：移除离群点和噪声点
void PointCloudProcessor::applyStatisticalOutlierRemoval(int meanK,
                                                         double stddevMulThresh)
{
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(accumulated_cloud_);
    sor.setMeanK(meanK);                      // K 邻域的点数
    sor.setStddevMulThresh(stddevMulThresh);  // 标准差乘数阈值
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(
        new pcl::PointCloud<pcl::PointXYZ>);
    sor.filter(*filtered_cloud);
    accumulated_cloud_ = filtered_cloud;
}
// 平面分割：分割出主要平面，用于移除地面等
void PointCloudProcessor::applyPlaneSegmentation(double distanceThreshold)
{
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(distanceThreshold);
    seg.setInputCloud(accumulated_cloud_);
    seg.segment(*inliers, *coefficients);

    // 提取平面
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(accumulated_cloud_);
    extract.setIndices(inliers);
    extract.setNegative(true);  // 保留非平面部分
    pcl::PointCloud<pcl::PointXYZ>::Ptr segmented_cloud(
        new pcl::PointCloud<pcl::PointXYZ>);
    extract.filter(*segmented_cloud);
    accumulated_cloud_ = segmented_cloud;
}
// 计算法线：为点云数据计算法线，用于进一步分析。
void PointCloudProcessor::computeNormals()
{
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(
        new pcl::search::KdTree<pcl::PointXYZ>());
    ne.setSearchMethod(tree);
    ne.setInputCloud(accumulated_cloud_);
    ne.setKSearch(50);  // 设置邻域点数量
    ne.compute(*normals);
}
