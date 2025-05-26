#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <random>
#include <iostream>

void addNoise(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float stddev) {
    std::default_random_engine gen;
    std::normal_distribution<float> dist(0.0f, stddev);
    for (auto& pt : cloud->points) {
        pt.x += dist(gen);
        pt.y += dist(gen);
        pt.z += dist(gen);
    }
}

int main() {
    pcl::PointCloud<pcl::PointXYZ>::Ptr original(new pcl::PointCloud<pcl::PointXYZ>);

    // 加载原始点云作为地图
    if (pcl::io::loadPCDFile<pcl::PointXYZ>("data/sample_map.pcd", *original) == -1) {
        PCL_ERROR("无法加载 sample_map.pcd\n");
        return -1;
    }

    // 保存地图
    pcl::io::savePCDFileBinary("data/map.pcd", *original);

    // 创建仿真变换（旋转 + 平移）
    float yaw_deg = 15.0f;      // 旋转 15°
    float trans_x = 0.8f;       // X 平移 0.8 米
    float trans_y = -0.4f;      // Y 平移 -0.4 米
    float noise_stddev = 0.02f; // 噪声标准差

    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    float yaw = yaw_deg * M_PI / 180.0f;
    transform.translation() << trans_x, trans_y, 0.0;
    transform.rotate(Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ()));

    // 应用变换
    pcl::PointCloud<pcl::PointXYZ>::Ptr scan(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(*original, *scan, transform);

    // 添加噪声
    addNoise(scan, noise_stddev);

    // 保存扫描点云
    pcl::io::savePCDFileBinary("data/input_scan.pcd", *scan);

    std::cout << "生成仿真点云完成！" << std::endl;
    std::cout << "地图库：data/map.pcd\n";
    std::cout << "扫描点云：data/input_scan.pcd（已添加误差）" << std::endl;

    return 0;
}
