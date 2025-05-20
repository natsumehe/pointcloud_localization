#include "io_utils.hpp"
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <iostream>

PointCloud loadPointCloudFromPCD(const std::string& filename) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(filename, *pcl_cloud) == -1) {
        std::cerr << "Error loading PCD file: " << filename << std::endl;
        return {};
    }

    PointCloud result;
    result.reserve(pcl_cloud->size());
    for (const auto& pt : *pcl_cloud) {
        result.push_back(PointXYZ{pt.x, pt.y, pt.z});
    }

    return result;
}
