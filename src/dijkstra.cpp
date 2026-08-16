#include "nav/dijkstra.hpp"

#include <functional>
#include <limits>
#include <queue>
#include <utility>

namespace nav {

std::vector<int> dijkstra(const Graph& graph, int src) {
    const int INF = std::numeric_limits<int>::max();
    std::vector<int> dist(graph.size(), INF);
    dist[src] = 0;

    // 小根堆：按距离从小到大取，pair 是 (距离, 节点)
    using P = std::pair<int, int>;
    std::priority_queue<P, std::vector<P>, std::greater<P>> pq;
    pq.push({0, src});

    while (!pq.empty()) {
        auto [d, u] = pq.top();  // 当前距离最小的节点
        pq.pop();

        if (d != dist[u]) continue;  // 是过期的旧记录，跳过

        for (const Edge& e : graph[u]) {
            if (dist[u] + e.weight < dist[e.to]) {
                dist[e.to] = dist[u] + e.weight;
                pq.push({dist[e.to], e.to});
            }
        }
    }

    // 不可达的节点标记为 -1
    for (int& d : dist) {
        if (d == INF) d = -1;
    }
    return dist;
}

}  // namespace nav
