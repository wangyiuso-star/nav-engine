#pragma once

#include <vector>

#include "nav/graph.hpp"

namespace nav {

// 计算一条节点序列的总权重：依次累加 path[i] -> path[i+1] 的最小边权。
// 若某相邻两点之间不存在该方向的边，返回 -1（表示这条路径在图上是无效的）。
// 空路径或单节点路径返回 0（不消耗任何边）。
int path_cost(const Graph& graph, const std::vector<int>& path);

}  // namespace nav
