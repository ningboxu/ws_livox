#ifndef POINTCLOUD_PROCESSOR_H
#define POINTCLOUD_PROCESSOR_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <chrono>
#include <string>

class PointCloudProcessor
{
public:
    PointCloudProcessor(int max_saves, int group_size);
    ~PointCloudProcessor();

    void accumulatePointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
    void savePointCloud(int frame_group);
    bool hasReachedMaxSaves();
    void clearAccumulatedCloud();

    void calculatePublishFrequency();  // 计算发布频率

private:
    pcl::PointCloud<pcl::PointXYZ>::Ptr accumulated_cloud_;
    int save_count_;
    int max_saves_;
    int group_size_;
    std::chrono::time_point<std::chrono::steady_clock> last_timestamp_;
    double publish_frequency_;
};

#endif  // POINTCLOUD_PROCESSOR_H
