#pragma once

#include "types.hpp"
#include "score_table.hpp"
#include <vector>

/**
 * 对点云进行 yaw 旋转和平移变换
 */
PointCloud transformCloud(const PointCloud& cloud, float dx, float dy, float theta);

/**
 * 计算一个点在得分查找表中的得分
 */
float scorePoint(const PointXYZ& pt,
                 float Rx, float Ry, float Rz, float Zres,
                 float minX, float minY, float minZ,
                 const ScoreTable& scoreTable);

/**
 * 对全局位姿搜索空间进行评分，返回最优匹配位姿
 */
Pose2D runCSMSearch(const PointCloud& inputCloud,
                    float Rx, float Ry, float Rz, float Zres,
                    float minX, float minY, float minZ,
                    const ScoreTable& scoreTable,
                    float dx_range = 2.0f,
                    float dy_range = 2.0f,
                    float dtheta_range = 3.14f,
                    float dx_step = 0.2f,
                    float dy_step = 0.2f,
                    float dtheta_step = 0.1f);
