#pragma once

#include <vector>

#include "nav/graph.hpp"

namespace nav {

// 返回从 src 出发到所有节点的最短距离；不可达的节点距离为 -1
std::vector<int> dijkstra(const Graph& graph, int src);

}  // namespace nav
