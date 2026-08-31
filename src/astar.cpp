#include "nav/astar.hpp"

#include <functional>
#include <limits>
#include <queue>
#include <vector>

namespace nav {

namespace {

// 堆元素：f = g + h，g 是真实已走距离，node 是节点编号
struct AStarState {
    int f;
    int g;
    int node;
};

// 比较器（tie-breaking）：
//   - f 小者优先；
//   - f 相同时，g 大者优先（离目标更近，避免在同分节点里乱弹）
struct CompareAStar {
    bool operator()(const AStarState& a, const AStarState& b) const {
        if (a.f != b.f) return a.f > b.f;
        return a.g < b.g;
    }
};

}  // namespace

int astar(const Graph& graph, int src, int target, const std::function<int(int)>& heuristic,
          Stats* stats) {
    // A* 算法
    // g[u] 表示从 src 到 u 的最短距离
    // key = f[u] = g[u] + h[u]，h[u] 是启发式函数
    const int INF = std::numeric_limits<int>::max();
    std::vector<int> g(graph.size(), INF);
    g[src] = 0;

    std::priority_queue<AStarState, std::vector<AStarState>, CompareAStar> pq;
    pq.push({heuristic(src), 0, src});

    while (!pq.empty()) {
        AStarState s = pq.top();
        pq.pop();
        int u = s.node;

        if (s.g != g[u]) continue;  // 过期记录，跳过
        if (stats) ++stats->nodes_expanded;
        if (u == target) return g[u];

        for (const Edge& e : graph[u]) {
            int ng = g[u] + e.weight;
            if (ng < g[e.to]) {
                g[e.to] = ng;
                pq.push({ng + heuristic(e.to), ng, e.to});
            }
        }
    }
    return -1;
}

}  // namespace nav
