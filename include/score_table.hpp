#pragma once

#include "types.hpp"
#include "voxelizer.hpp"
#include <unordered_map>
#include <vector>

// 每个栅格细分后的z方向得分数组
struct FineScore {
    std::vector<float> scores;
};

// 得分查表类型：GridIndex -> 细分z得分数组
using ScoreTable = std::unordered_map<GridIndex, FineScore>;

/**
 * 生成 z 方向细分得分查找表
 * 
 * @param features 每个栅格的统计特征（均值、方差、频数）
 * @param Rz 原始z方向分辨率
 * @param Zres z方向细分分辨率（Zres < Rz）
 * @param minZ 点云z方向最小值（用于计算偏移）
 * @param maxp 所有栅格的最大频数（用于归一化频数得分）
 */
ScoreTable generateScoreTable(const std::unordered_map<GridIndex, GridFeature>& features,
                              float Rz, float Zres, float minZ, int maxp,
                              float alpha = 1.0f);
