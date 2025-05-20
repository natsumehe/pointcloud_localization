#include "voxelizer.hpp"
#include <cmath>
#include <numeric>

VoxelMap voxelize(const PointCloud& cloud,
                  float Rx, float Ry, float Rz,
                  float minX, float minY, float minZ) {
    VoxelMap gridMap;

    for (const auto& pt : cloud) {
        int ix = static_cast<int>((pt.x - minX) / Rx);
        int iy = static_cast<int>((pt.y - minY) / Ry);
        int iz = static_cast<int>((pt.z - minZ) / Rz);
        GridIndex idx{ix, iy, iz};
        gridMap[idx].z_values.push_back(pt.z);
        gridMap[idx].count++;
    }

    return gridMap;
}

std::unordered_map<GridIndex, GridFeature> computeFeatures(const VoxelMap& voxelMap) {
    std::unordered_map<GridIndex, GridFeature> featureMap;

    for (const auto& [idx, data] : voxelMap) {
        const auto& z_vals = data.z_values;
        if (z_vals.empty()) continue;

        float sum = std::accumulate(z_vals.begin(), z_vals.end(), 0.0f);
        float mean = sum / z_vals.size();

        float var = 0.0f;
        for (float z : z_vals)
            var += (z - mean) * (z - mean);
        var /= z_vals.size();

        featureMap[idx] = GridFeature{(int)z_vals.size(), mean, std::sqrt(var)};
    }

    return featureMap;
}

int computeMaxCount(const std::unordered_map<GridIndex, GridFeature>& features) {
    int maxp = 0;
    for (const auto& [idx, feat] : features)
        maxp = std::max(maxp, feat.count);
    return maxp;
}
