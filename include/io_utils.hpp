#pragma once

#include "types.hpp"
#include <string>

/**
 * 加载 .pcd 点云文件为 PointCloud 格式
 */
PointCloud loadPointCloudFromPCD(const std::string& filename);
