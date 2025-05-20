#include "csm_matcher.hpp"
#include <cmath>
#include <algorithm>

PointCloud transformCloud(const PointCloud& cloud, float dx, float dy, float theta) {
    PointCloud result;
    result.reserve(cloud.size());

    float cos_theta = std::cos(theta);
    float sin_theta = std::sin(theta);

    for (const auto& pt : cloud) {
        PointXYZ p;
        p.x = pt.x * cos_theta - pt.y * sin_theta + dx;
        p.y = pt.x * sin_theta + pt.y * cos_theta + dy;
        p.z = pt.z;
        result.push_back(p);
    }

    return result;
}

float scorePoint(const PointXYZ& pt,
                 float Rx, float Ry, float Rz, float Zres,
                 float minX, float minY, float minZ,
                 const ScoreTable& scoreTable) {
    int ix = static_cast<int>((pt.x - minX) / Rx);
    int iy = static_cast<int>((pt.y - minY) / Ry);
    int iz = static_cast<int>((pt.z - minZ) / Rz);

    GridIndex idx{ix, iy, iz};
    float localZ = pt.z - (minZ + iz * Rz);
    int zj = static_cast<int>(localZ / Zres);

    auto it = scoreTable.find(idx);
    if (it != scoreTable.end()) {
        const auto& scores = it->second.scores;
        if (zj >= 0 && zj < scores.size()) {
            return scores[zj];
        }
    }
    return 0.0f;
}

Pose2D runCSMSearch(const PointCloud& inputCloud,
                    float Rx, float Ry, float Rz, float Zres,
                    float minX, float minY, float minZ,
                    const ScoreTable& scoreTable,
                    float dx_range,
                    float dy_range,
                    float dtheta_range,
                    float dx_step,
                    float dy_step,
                    float dtheta_step) {
    float best_score = -1e9f;
    Pose2D best_pose{};

    for (float dx = -dx_range; dx <= dx_range; dx += dx_step) {
        for (float dy = -dy_range; dy <= dy_range; dy += dy_step) {
            for (float theta = -dtheta_range; theta <= dtheta_range; theta += dtheta_step) {
                PointCloud transformed = transformCloud(inputCloud, dx, dy, theta);

                float total_score = 0.0f;
                for (const auto& pt : transformed) {
                    total_score += scorePoint(pt, Rx, Ry, Rz, Zres, minX, minY, minZ, scoreTable);
                }

                if (total_score > best_score) {
                    best_score = total_score;
                    best_pose = {dx, dy, theta};
                }
            }
        }
    }

    return best_pose;
}
