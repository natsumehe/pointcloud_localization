#include "score_table.hpp"
#include <cmath>

ScoreTable generateScoreTable(const std::unordered_map<GridIndex, GridFeature>& features,
                              float Rz, float Zres, float minZ, int maxp,
                              float alpha) {
    ScoreTable table;

    int bins = static_cast<int>(Rz / Zres);

    for (const auto& [idx, feat] : features) {
        float mu = feat.mean_z;
        float sigma = feat.stddev_z;
        int n = feat.count;

        float baseZ = minZ + idx.iz * Rz;

        FineScore fs;
        fs.scores.resize(bins);

        for (int j = 0; j < bins; ++j) {
            float z_center = baseZ + j * Zres + Zres / 2.0f;
            float gauss_score = 0.0f;

            if (sigma > 1e-3f) {
                float diff = z_center - mu;
                gauss_score = alpha * std::exp(-(diff * diff) / (2.0f * sigma * sigma));
            }

            float freq_score = static_cast<float>(n) / static_cast<float>(maxp);
            fs.scores[j] = gauss_score + freq_score;
        }

        table[idx] = fs;
    }

    return table;
}
