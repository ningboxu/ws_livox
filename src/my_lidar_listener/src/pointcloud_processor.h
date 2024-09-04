#ifndef POINTCLOUD_PROCESSOR_H
#define POINTCLOUD_PROCESSOR_H

#include <pcl/ModelCoefficients.h>        // 模型系数（平面分割）
#include <pcl/features/normal_3d.h>       // 法线计算
#include <pcl/filters/extract_indices.h>  // 提取索引（平面分割）
#include <pcl/filters/statistical_outlier_removal.h>  // 统计去噪
#include <pcl/filters/voxel_grid.h>                   // 体素栅格滤波
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/sac_segmentation.h>  // 平面分割
#include <chrono>
#include <string>

class PointCloudProcessor
{
public:
    PointCloudProcessor(int max_saves, double save_interval);
    ~PointCloudProcessor();

    void accumulatePointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
    void savePointCloud();
    bool shouldSave();
    bool hasReachedMaxSaves();
    // 预处理功能
    void applyVoxelGridFilter(float leaf_size);  // 体素栅格滤波
    void applyStatisticalOutlierRemoval(int meanK,
                                        double stddevMulThresh);  // 统计去噪
    void applyPlaneSegmentation(double distanceThreshold);  // 平面分割
    void computeNormals();                                  // 计算法线
private:
    pcl::PointCloud<pcl::PointXYZ>::Ptr accumulated_cloud_;
    int save_count_;
    int max_saves_;
    double save_interval_;
    std::chrono::time_point<std::chrono::steady_clock> start_time_;
};

#endif  // POINTCLOUD_PROCESSOR_H
