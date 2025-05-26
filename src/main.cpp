#include "voxelizer.hpp"
#include "score_table.hpp"
#include "csm_matcher.hpp"
#include "io_utils.hpp"

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <thread>

int main() {
    // Step 1: 加载点云
    PointCloud mapCloud = loadPointCloudFromPCD("data/sample_map.pcd");
    PointCloud scanCloud = loadPointCloudFromPCD("data/input_scan.pcd");

    // Step 2: 参数设置
    float Rx = 0.5f, Ry = 0.5f, Rz = 1.0f, Zres = 0.2f;
    float minX = -20.0f, minY = -20.0f, minZ = -3.0f;

    // Step 3: 特征提取与得分查表
    auto voxelMap = voxelize(mapCloud, Rx, Ry, Rz, minX, minY, minZ);
    auto features = computeFeatures(voxelMap);
    int maxp = computeMaxCount(features);
    auto scoreTable = generateScoreTable(features, Rz, Zres, minZ, maxp);

    // Step 4: 执行CSM配准
    Pose2D best_pose = runCSMSearch(scanCloud, Rx, Ry, Rz, Zres,
                                     minX, minY, minZ,
                                     scoreTable);

    std::cout << "Best pose: x=" << best_pose.x
              << ", y=" << best_pose.y
              << ", yaw=" << best_pose.yaw << std::endl;

    // Step 5: 可视化（使用PCL）
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_map(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_scan(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_transformed(new pcl::PointCloud<pcl::PointXYZ>());

    for (const auto& pt : mapCloud)
        pcl_map->push_back(pcl::PointXYZ(pt.x, pt.y, pt.z));
    for (const auto& pt : scanCloud)
        pcl_scan->push_back(pcl::PointXYZ(pt.x, pt.y, pt.z));

    // 构造变换矩阵
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    transform.translation() << best_pose.x, best_pose.y, 0.0;
    transform.rotate(Eigen::AngleAxisf(best_pose.yaw, Eigen::Vector3f::UnitZ()));

    pcl::transformPointCloud(*pcl_scan, *pcl_transformed, transform);

    // 可视化
    pcl::visualization::PCLVisualizer viewer("PointCloud Registration");
    viewer.addPointCloud<pcl::PointXYZ>(pcl_map, "map_cloud");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 1.0, 1.0, "map_cloud");

    viewer.addPointCloud<pcl::PointXYZ>(pcl_scan, "scan_original");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 1.0, 0.0, "scan_original");

    viewer.addPointCloud<pcl::PointXYZ>(pcl_transformed, "scan_registered");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "scan_registered");

    while (!viewer.wasStopped()) {
        viewer.spinOnce(100);
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    return 0;
}
