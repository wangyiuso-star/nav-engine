#include "nav/path.hpp"

#include <cstddef>
#include <limits>

namespace nav {

int path_cost(const Graph& graph, const std::vector<int>& path) {
    if (path.size() <= 1) return 0;  // 空路径 / 单节点：不消耗任何边

    const int n = static_cast<int>(graph.size());
    const int INF = std::numeric_limits<int>::max();

    int total = 0;
    for (std::size_t i = 0; i + 1 < path.size(); ++i) {
        const int u = path[i];
        const int v = path[i + 1];
        if (u < 0 || u >= n || v < 0 || v >= n) return -1;  // 非法节点编号

        int best = INF;  // 平行边取最小权重
        for (const Edge& e : graph[u]) {
            if (e.to == v && e.weight < best) best = e.weight;
        }
        if (best == INF) return -1;  // 这条有向边不存在
        total += best;
    }
    return total;
}

}  // namespace nav
