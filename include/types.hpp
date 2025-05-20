#pragma once

#include <vector>

struct PointXYZ {
    float x, y, z;
};

using PointCloud = std::vector<PointXYZ>;

struct GridIndex {
    int ix, iy, iz;

    bool operator==(const GridIndex& other) const {
        return ix == other.ix && iy == other.iy && iz == other.iz;
    }
};

namespace std {
    template <>
    struct hash<GridIndex> {
        size_t operator()(const GridIndex& k) const {
            return ((std::hash<int>()(k.ix) ^ (std::hash<int>()(k.iy) << 1)) >> 1) ^ (std::hash<int>()(k.iz) << 1);
        }
    };
}

struct Pose2D {
    float x, y, yaw;
};
