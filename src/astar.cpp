#include "nav/astar.hpp"

#include <functional>
#include <limits>
#include <queue>
#include <utility>
#include <vector>

namespace nav {

int astar(const Graph& graph, int src, int target, const std::function<int(int)>& heuristic) {
    // A* 算法
    // g[u] 表示从 src 到 u 的最短距离
    // key = f[u] = g[u] + h[u]，h[u] 是启发式函数
    const int INF = std::numeric_limits<int>::max();
    std::vector<int> g(graph.size(), INF);
    g[src] = 0;
    using P = std::pair<int, int>;
    std::priority_queue<P, std::vector<P>, std::greater<P>> pq;
    pq.push({heuristic(src), src});
    while (!pq.empty()) {
        auto [f, u] = pq.top();
        pq.pop();
        if (f != g[u] + heuristic(u)) continue;
        if (u == target) return g[target];
        for (const Edge& e : graph[u]) {
            if (g[u] + e.weight < g[e.to]) {
                g[e.to] = g[u] + e.weight;
                pq.push({g[e.to] + heuristic(e.to), e.to});
            }
        }
    }
    return -1;
}
}  // namespace nav
