#include "nav/ch.hpp"

#include <cstddef>
#include <functional>
#include <limits>
#include <queue>
#include <unordered_map>
#include <utility>

namespace nav {

namespace {

// witness search：在「去掉 v、且跳过已收缩节点」的当前图上，
// 从 u 出发能否在 limit 距离内到达 w。到达 = 有见证路径 = 无需捷径。
bool has_witness(const std::vector<std::vector<Edge>>& adj, const std::vector<char>& contracted,
                 int u, int w, int v, int limit) {
    std::unordered_map<int, int> dist;
    dist[u] = 0;

    using P = std::pair<int, int>;
    std::priority_queue<P, std::vector<P>, std::greater<P>> pq;
    pq.push({0, u});

    int settled = 0;
    while (!pq.empty()) {
        auto [d, x] = pq.top();
        pq.pop();
        auto it = dist.find(x);
        if (it == dist.end() || it->second != d) continue;  // 过期
        if (d > limit) return false;
        if (x == w) return true;
        if (++settled > 128) return false;  // 限制规模；安全（只会多加捷径）

        for (const Edge& e : adj[x]) {
            if (e.to == v || contracted[e.to]) continue;
            int nd = d + e.weight;
            auto nit = dist.find(e.to);
            if (nit == dist.end() || nd < nit->second) {
                dist[e.to] = nd;
                pq.push({nd, e.to});
            }
        }
    }
    return false;
}

}  // namespace

CH::CH(const Graph& graph) : n_(static_cast<int>(graph.size())) {
    // 可变邻接表：原始边 + 后续加入的捷径
    std::vector<std::vector<Edge>> adj(graph.begin(), graph.end());

    std::vector<char> contracted(n_, 0);
    rank_.assign(n_, -1);

    // 节点当前「度」（只数未收缩邻居）
    auto degree = [&](int x) {
        int d = 0;
        for (const Edge& e : adj[x]) {
            if (!contracted[e.to]) ++d;
        }
        return d;
    };

    // 排序启发式：优先收缩度小的节点（懒删除）
    using P = std::pair<int, int>;  // (degree, node)
    std::priority_queue<P, std::vector<P>, std::greater<P>> pq;
    for (int v = 0; v < n_; ++v) pq.push({degree(v), v});

    int next_rank = 0;
    while (next_rank < n_) {
        int v = -1;
        while (!pq.empty()) {
            auto [d, x] = pq.top();
            if (!contracted[x] && d == degree(x)) {
                v = x;
                pq.pop();
                break;
            }
            pq.pop();
        }
        if (v == -1) break;  // 理论不会发生

        // 未收缩邻居：(权重, 节点)
        std::vector<std::pair<int, int>> neighbors;
        for (const Edge& e : adj[v]) {
            if (!contracted[e.to]) neighbors.push_back({e.weight, e.to});
        }

        // 邻居对 witness search，无见证则加捷径
        for (std::size_t i = 0; i < neighbors.size(); ++i) {
            for (std::size_t j = i + 1; j < neighbors.size(); ++j) {
                int u = neighbors[i].second;
                int w = neighbors[j].second;
                int len = neighbors[i].first + neighbors[j].first;
                if (!has_witness(adj, contracted, u, w, v, len)) {
                    adj[u].push_back({w, len});
                    adj[w].push_back({u, len});
                }
            }
        }

        contracted[v] = 1;
        rank_[v] = next_rank++;

        // 邻居失去 v / 获得捷径，度数变化，重新入队
        for (const auto& nb : neighbors) {
            int u = nb.second;
            if (!contracted[u]) pq.push({degree(u), u});
        }
    }

    // 构建 upward 图：每条边只保留「低 rank -> 高 rank」方向
    up_graph_.assign(n_, {});
    for (int u = 0; u < n_; ++u) {
        for (const Edge& e : adj[u]) {
            if (rank_[e.to] > rank_[u]) {
                up_graph_[u].push_back(e);
            }
        }
    }
}

int CH::query(int s, int t, Stats* stats) const {
    if (s == t) return 0;
    const int INF = std::numeric_limits<int>::max();

    std::vector<int> df(n_, INF), db(n_, INF);
    df[s] = 0;
    db[t] = 0;

    using P = std::pair<int, int>;
    std::priority_queue<P, std::vector<P>, std::greater<P>> fq, bq;
    fq.push({0, s});
    bq.push({0, t});

    std::vector<char> settled_f(n_, 0), settled_b(n_, 0);

    // 前向：只沿向上边
    while (!fq.empty()) {
        auto [d, u] = fq.top();
        fq.pop();
        if (d != df[u]) continue;
        settled_f[u] = 1;
        if (stats) ++stats->nodes_expanded;
        for (const Edge& e : up_graph_[u]) {
            if (df[u] + e.weight < df[e.to]) {
                df[e.to] = df[u] + e.weight;
                fq.push({df[e.to], e.to});
            }
        }
    }

    // 后向：同样只沿向上边（无向图，反向前进即反向上升）
    while (!bq.empty()) {
        auto [d, u] = bq.top();
        bq.pop();
        if (d != db[u]) continue;
        settled_b[u] = 1;
        if (stats) ++stats->nodes_expanded;
        for (const Edge& e : up_graph_[u]) {
            if (db[u] + e.weight < db[e.to]) {
                db[e.to] = db[u] + e.weight;
                bq.push({db[e.to], e.to});
            }
        }
    }

    // 在峰点相遇
    int best = INF;
    for (int u = 0; u < n_; ++u) {
        if (settled_f[u] && settled_b[u]) {
            int cand = df[u] + db[u];
            if (cand < best) best = cand;
        }
    }
    return best == INF ? -1 : best;
}

}  // namespace nav
