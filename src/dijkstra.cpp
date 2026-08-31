#include "nav/dijkstra.hpp"

#include <functional>
#include <limits>
#include <queue>
#include <utility>

namespace nav {

std::vector<int> dijkstra(const Graph& graph, int src, Stats* stats) {
    const int INF = std::numeric_limits<int>::max();
    std::vector<int> dist(graph.size(), INF);
    dist[src] = 0;

    using P = std::pair<int, int>;  // (距离, 节点)
    std::priority_queue<P, std::vector<P>, std::greater<P>> pq;
    pq.push({0, src});

    while (!pq.empty()) {
        auto [d, u] = pq.top();
        pq.pop();

        if (d != dist[u]) continue;  // 过期记录，跳过
        if (stats) ++stats->nodes_expanded;

        for (const Edge& e : graph[u]) {
            if (dist[u] + e.weight < dist[e.to]) {
                dist[e.to] = dist[u] + e.weight;
                pq.push({dist[e.to], e.to});
            }
        }
    }

    for (int& d : dist) {
        if (d == INF) d = -1;
    }
    return dist;
}

int dijkstra_point_to_point(const Graph& graph, int src, int target, Stats* stats) {
    const int INF = std::numeric_limits<int>::max();
    std::vector<int> dist(graph.size(), INF);
    dist[src] = 0;

    using P = std::pair<int, int>;
    std::priority_queue<P, std::vector<P>, std::greater<P>> pq;
    pq.push({0, src});

    while (!pq.empty()) {
        auto [d, u] = pq.top();
        pq.pop();

        if (d != dist[u]) continue;  // 过期记录，跳过
        if (stats) ++stats->nodes_expanded;
        if (u == target) return d;  // 关键：到 target 就停

        for (const Edge& e : graph[u]) {
            if (dist[u] + e.weight < dist[e.to]) {
                dist[e.to] = dist[u] + e.weight;
                pq.push({dist[e.to], e.to});
            }
        }
    }
    return -1;  // 不可达
}

}  // namespace nav
