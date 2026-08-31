#pragma once

#include <functional>
#include <vector>

#include "nav/graph.hpp"
#include "nav/stats.hpp"

namespace nav {

// A* 最短路：返回从 src 到 target 的最短距离；不可达返回 -1。
//
// heuristic(n) 是「从 n 到 target 的距离估计」，必须满足可采纳性：
//   0 <= heuristic(n) <= 从 n 到 target 的真实最短距离
// 且 heuristic(target) 应为 0。满足该条件时，结果保证是最短路。
//
// 常见启发式：地图上两点间的直线距离（直线 <= 实际道路距离）。
// stats 非空时，会统计展开（settle）的节点数。
int astar(const Graph& graph, int src, int target, const std::function<int(int)>& heuristic,
          Stats* stats = nullptr);

}  // namespace nav
