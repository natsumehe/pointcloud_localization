#pragma once

#include "types.hpp"
#include <unordered_map>
#include <vector>

// 每个体素内的统计数据
struct VoxelData {
    std::vector<float> z_values;
    int count = 0;
};

// 三维体素地图类型：GridIndex -> VoxelData
using VoxelMap = std::unordered_map<GridIndex, VoxelData>;

// 三维栅格化函数
VoxelMap voxelize(const PointCloud& cloud,
                  float Rx, float Ry, float Rz,
                  float minX, float minY, float minZ);

// 每个栅格的特征（频数、均值、方差）
struct GridFeature {
    int count;
    float mean_z;
    float stddev_z;
};

// 体素地图 -> 栅格特征表
std::unordered_map<GridIndex, GridFeature> computeFeatures(const VoxelMap& voxelMap);

// 返回最大频数 maxp
int computeMaxCount(const std::unordered_map<GridIndex, GridFeature>& features);
