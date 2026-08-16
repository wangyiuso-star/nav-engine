#pragma once

#include <vector>

namespace nav {

// 一条有向带权边
struct Edge {
    int to;      // 指向的节点
    int weight;  // 边的权重（距离）
};

// 邻接表图：graph[u] 是「从 u 出发的所有边」
using Graph = std::vector<std::vector<Edge>>;

}  // namespace nav
