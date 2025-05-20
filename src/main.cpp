#include "voxelizer.hpp"
#include "score_table.hpp"
#include "csm_matcher.hpp"
#include "io_utils.hpp"
#include <Open3D/Open3D.h>
#include <iostream>

int main() {
    // 加载点云
    PointCloud mapCloud = loadPointCloudFromPCD("data/sample_map.pcd");
    PointCloud scanCloud = loadPointCloudFromPCD("data/input_scan.pcd");

    // 栅格参数
    float Rx = 0.5f, Ry = 0.5f, Rz = 1.0f, Zres = 0.2f;
    float minX = -20.0f, minY = -20.0f, minZ = -3.0f;

    // Step 1：三维栅格划分与特征提取
    auto voxelMap = voxelize(mapCloud, Rx, Ry, Rz, minX, minY, minZ);
    auto features = computeFeatures(voxelMap);
    int maxp = computeMaxCount(features);

    // Step 2：构建得分查表
    auto scoreTable = generateScoreTable(features, Rz, Zres, minZ, maxp);

    // Step 3：点云配准搜索
    Pose2D best_pose = runCSMSearch(scanCloud, Rx, Ry, Rz, Zres,
                                     minX, minY, minZ,
                                     scoreTable);

    std::cout << "Best pose: x=" << best_pose.x
              << ", y=" << best_pose.y
              << ", yaw=" << best_pose.yaw << std::endl;

    // 可视化配准结果
    auto mapCloudOpen3D = std::make_shared<open3d::geometry::PointCloud>();
    for (const auto& pt : mapCloud) {
        mapCloudOpen3D->points_.emplace_back(pt.x, pt.y, pt.z);
    }

    auto scanCloudTransformed = scanCloud;
    for (auto& pt : scanCloudTransformed) {
        float x_new = pt.x * cos(best_pose.yaw) - pt.y * sin(best_pose.yaw) + best_pose.x;
        float y_new = pt.x * sin(best_pose.yaw) + pt.y * cos(best_pose.yaw) + best_pose.y;
        pt.x = x_new;
        pt.y = y_new;
    }

    auto scanCloudOpen3D = std::make_shared<open3d::geometry::PointCloud>();
    for (const auto& pt : scanCloudTransformed) {
        scanCloudOpen3D->points_.emplace_back(pt.x, pt.y, pt.z);
    }

    open3d::visualization::DrawGeometries({mapCloudOpen3D, scanCloudOpen3D},
                                          "Point Cloud Registration",
                                          800, 600);

    return 0;
}
