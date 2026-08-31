#pragma once

#include <vector>

#include "nav/graph.hpp"
#include "nav/stats.hpp"

namespace nav {

// 返回从 src 出发到所有节点的最短距离；不可达的节点距离为 -1。
// stats 非空时，会统计展开（settle）的节点数。
std::vector<int> dijkstra(const Graph& graph, int src, Stats* stats = nullptr);

// 点到点 Dijkstra：到达 target 就提前返回，返回 dist[target]；不可达 -1。
// 与上面的区别：上面算「到所有点」，这里只算「到 target」（导航场景用它）。
int dijkstra_point_to_point(const Graph& graph, int src, int target, Stats* stats = nullptr);

}  // namespace nav
