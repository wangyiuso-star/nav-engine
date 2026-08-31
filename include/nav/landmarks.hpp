#pragma once

#include "nav/graph.hpp"

#include <vector>

namespace nav {

// ALT 启发式：预计算若干「地标」到所有节点的最短距离，
// 查询时用三角不等式给出比欧氏更紧的可采纳下界。
class Landmarks {
public:
    // landmark_nodes: 选中的地标节点编号
    Landmarks(const Graph& graph, const std::vector<int>& landmark_nodes);

    // 返回 dist(n, target) 的下界 = max_L |dist(L, target) - dist(L, n)|
    int heuristic(int n, int target) const;

    const std::vector<int>& landmarks() const { return landmark_nodes_; }

private:
    std::vector<int> landmark_nodes_;
    std::vector<std::vector<int>> dist_;  // dist_[k][n] = 地标 k 到 n 的最短距离（-1 表示不可达）
};

}  // namespace nav
