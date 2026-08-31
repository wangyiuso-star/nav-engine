#include "nav/landmarks.hpp"

#include "nav/dijkstra.hpp"

#include <cstdlib>

namespace nav {

Landmarks::Landmarks(const Graph& graph, const std::vector<int>& landmark_nodes)
    : landmark_nodes_(landmark_nodes) {
    dist_.reserve(landmark_nodes.size());
    for (int L : landmark_nodes) {
        // 对每个地标跑一次全图 Dijkstra，得到 dist(L, 所有节点)
        dist_.push_back(dijkstra(graph, L));
    }
}

int Landmarks::heuristic(int n, int target) const {
    int best = 0;
    for (std::size_t k = 0; k < landmark_nodes_.size(); ++k) {
        int d_t = dist_[k][target];
        int d_n = dist_[k][n];
        if (d_t < 0 || d_n < 0) continue;  // 不可达，跳过该地标
        int bound = std::abs(d_t - d_n);   // 三角不等式下界
        if (bound > best) best = bound;
    }
    return best;
}

}  // namespace nav
