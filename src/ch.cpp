#include "nav/ch.hpp"

#include <algorithm>
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

    // 查询缓冲区：一次性分配并建立不变式（query 中复用，避免每次 O(n) 分配/清零）
    const int INF = std::numeric_limits<int>::max();
    df_.assign(n_, INF);
    db_.assign(n_, INF);
    settled_f_.assign(n_, 0);
    settled_b_.assign(n_, 0);
    df_touched_.reserve(1024);
    db_touched_.reserve(1024);
    settled_f_touched_.reserve(1024);
    settled_b_touched_.reserve(1024);
    fq_.reserve(1024);
    bq_.reserve(1024);
}

int CH::query(int s, int t, Stats* stats) const {
    if (s == t) return 0;
    const int INF = std::numeric_limits<int>::max();
    const std::greater<Item> kGreater;  // 与 std::priority_queue<..., greater> 语义一致

    // 复用堆：clear 保留容量，后续查询不再分配
    fq_.clear();
    bq_.clear();

    // ---- 前向：只沿向上边 ----
    df_[s] = 0;
    df_touched_.push_back(s);
    fq_.push_back({0, s});
    std::push_heap(fq_.begin(), fq_.end(), kGreater);

    while (!fq_.empty()) {
        const Item top = fq_.front();
        std::pop_heap(fq_.begin(), fq_.end(), kGreater);
        fq_.pop_back();

        const int d = top.first;
        const int u = top.second;
        if (d != df_[u]) continue;  // 过期记录
        settled_f_[u] = 1;
        settled_f_touched_.push_back(u);
        if (stats) ++stats->nodes_expanded;

        for (const Edge& e : up_graph_[u]) {
            const int nd = df_[u] + e.weight;
            if (nd < df_[e.to]) {
                if (df_[e.to] == INF) df_touched_.push_back(e.to);  // 首次写入才登记
                df_[e.to] = nd;
                fq_.push_back({nd, e.to});
                std::push_heap(fq_.begin(), fq_.end(), kGreater);
            }
        }
    }

    // ---- 后向：同样只沿向上边（无向图，反向前进即反向上升）----
    db_[t] = 0;
    db_touched_.push_back(t);
    bq_.push_back({0, t});
    std::push_heap(bq_.begin(), bq_.end(), kGreater);

    while (!bq_.empty()) {
        const Item top = bq_.front();
        std::pop_heap(bq_.begin(), bq_.end(), kGreater);
        bq_.pop_back();

        const int d = top.first;
        const int u = top.second;
        if (d != db_[u]) continue;  // 过期记录
        settled_b_[u] = 1;
        settled_b_touched_.push_back(u);
        if (stats) ++stats->nodes_expanded;

        for (const Edge& e : up_graph_[u]) {
            const int nd = db_[u] + e.weight;
            if (nd < db_[e.to]) {
                if (db_[e.to] == INF) db_touched_.push_back(e.to);
                db_[e.to] = nd;
                bq_.push_back({nd, e.to});
                std::push_heap(bq_.begin(), bq_.end(), kGreater);
            }
        }
    }

    // ---- 在峰点相遇：只遍历前向 settle 过的节点，O(已 settle 数) ----
    int best = INF;
    for (const int u : settled_f_touched_) {
        if (settled_b_[u]) {
            const int cand = df_[u] + db_[u];
            if (cand < best) best = cand;
        }
    }

    // ---- 精确复位：只还原本次碰过的节点，恢复不变式 ----
    for (const int u : df_touched_) df_[u] = INF;
    for (const int u : db_touched_) db_[u] = INF;
    for (const int u : settled_f_touched_) settled_f_[u] = 0;
    for (const int u : settled_b_touched_) settled_b_[u] = 0;
    df_touched_.clear();
    db_touched_.clear();
    settled_f_touched_.clear();
    settled_b_touched_.clear();

    return best == INF ? -1 : best;
}

}  // namespace nav
