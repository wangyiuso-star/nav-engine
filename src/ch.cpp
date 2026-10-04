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
// 只读 e.to / e.weight，不关心捷径信息。
bool has_witness(const std::vector<std::vector<CH::ChEdge>>& adj,
                 const std::vector<char>& contracted, int u, int w, int v, int limit) {
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

        for (const CH::ChEdge& e : adj[x]) {
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
    // 完整邻接表（成员 adj_）：先把原始边全部拷入，按「原始边」约定取 via = -1
    adj_.assign(n_, {});
    for (int u = 0; u < n_; ++u) {
        adj_[u].reserve(graph[u].size());
        for (const Edge& e : graph[u]) {
            adj_[u].push_back(ChEdge{e.to, e.weight, -1, 0, 0});
        }
    }

    std::vector<char> contracted(n_, 0);
    rank_.assign(n_, -1);

    // 节点当前「度」（只数未收缩邻居）
    auto degree = [&](int x) {
        int d = 0;
        for (const ChEdge& e : adj_[x]) {
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

        // 未收缩邻居：(v 到该邻居的权重, 节点)
        std::vector<std::pair<int, int>> neighbors;
        for (const ChEdge& e : adj_[v]) {
            if (!contracted[e.to]) neighbors.push_back({e.weight, e.to});
        }

        // 邻居对 witness search，无见证则加捷径
        for (std::size_t i = 0; i < neighbors.size(); ++i) {
            for (std::size_t j = i + 1; j < neighbors.size(); ++j) {
                int u = neighbors[i].second;
                int w = neighbors[j].second;
                int wu = neighbors[i].first;  // u—v 的权重
                int ww = neighbors[j].first;  // w—v 的权重
                int len = wu + ww;
                if (!has_witness(adj_, contracted, u, w, v, len)) {
                    // u—w 捷径代表真实路径 u → v → w：先 u→v（wu），再 v→w（ww）
                    adj_[u].push_back(ChEdge{w, len, v, wu, ww});
                    // w—u 捷径代表真实路径 w → v → u：先 w→v（ww），再 v→u（wu）
                    adj_[w].push_back(ChEdge{u, len, v, ww, wu});
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

    // 构建 upward 图：每条边只保留「低 rank -> 高 rank」方向（via / w1 / w2 原样保留）
    up_graph_.assign(n_, {});
    for (int u = 0; u < n_; ++u) {
        for (const ChEdge& e : adj_[u]) {
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

    // 路径重建缓冲（仅当 query 的 path != nullptr 时写入）
    par_f_.assign(n_, ParInfo{});
    par_b_.assign(n_, ParInfo{});
    par_f_touched_.reserve(1024);
    par_b_touched_.reserve(1024);
}

int CH::query(int s, int t, Stats* stats, std::vector<int>* path) const {
    const bool want_path = (path != nullptr);
    if (want_path) path->clear();

    if (s == t) {
        if (want_path) path->push_back(s);
        return 0;
    }
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

        for (const ChEdge& e : up_graph_[u]) {
            const int nd = df_[u] + e.weight;
            if (nd < df_[e.to]) {
                if (df_[e.to] == INF) df_touched_.push_back(e.to);  // 首次写入才登记
                df_[e.to] = nd;
                if (want_path) {
                    if (par_f_[e.to].parent == -1) par_f_touched_.push_back(e.to);
                    par_f_[e.to] = ParInfo{u, e.via, e.weight};
                }
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

        for (const ChEdge& e : up_graph_[u]) {
            const int nd = db_[u] + e.weight;
            if (nd < db_[e.to]) {
                if (db_[e.to] == INF) db_touched_.push_back(e.to);
                db_[e.to] = nd;
                if (want_path) {
                    if (par_b_[e.to].parent == -1) par_b_touched_.push_back(e.to);
                    par_b_[e.to] = ParInfo{u, e.via, e.weight};
                }
                bq_.push_back({nd, e.to});
                std::push_heap(bq_.begin(), bq_.end(), kGreater);
            }
        }
    }

    // ---- 在峰点相遇：只遍历前向 settle 过的节点，O(已 settle 数) ----
    int best = INF;
    int meet = -1;
    for (const int u : settled_f_touched_) {
        if (settled_b_[u]) {
            const int cand = df_[u] + db_[u];
            if (cand < best) {
                best = cand;
                meet = u;
            }
        }
    }

    // ---- 路径重建：拼装双向链，并递归展开捷径 ----
    if (want_path && best != INF && meet != -1) {
        // 前向链：meet → ... → s（沿 par_f_），反转后即 s → ... → meet
        std::vector<int> fwd;
        for (int x = meet; x != -1; x = par_f_[x].parent) fwd.push_back(x);
        std::reverse(fwd.begin(), fwd.end());

        path->push_back(s);
        for (std::size_t i = 0; i + 1 < fwd.size(); ++i) {
            expand(fwd[i], fwd[i + 1], par_f_[fwd[i + 1]].weight, *path);
        }

        // 后向链：meet → par_b_[meet] → ... → t（无向图下这个顺序就是真实行进方向）
        int x = meet;
        while (x != t) {
            const int nx = par_b_[x].parent;
            if (nx == -1) break;  // 理论不会发生
            expand(x, nx, par_b_[x].weight, *path);
            x = nx;
        }
    }

    // ---- 精确复位：只还原本次碰过的节点，恢复不变式 ----
    for (const int u : df_touched_) df_[u] = INF;
    for (const int u : db_touched_) db_[u] = INF;
    for (const int u : settled_f_touched_) settled_f_[u] = 0;
    for (const int u : settled_b_touched_) settled_b_[u] = 0;
    if (want_path) {
        for (const int u : par_f_touched_) par_f_[u].parent = -1;
        for (const int u : par_b_touched_) par_b_[u].parent = -1;
        par_f_touched_.clear();
        par_b_touched_.clear();
    }
    df_touched_.clear();
    db_touched_.clear();
    settled_f_touched_.clear();
    settled_b_touched_.clear();

    return best == INF ? -1 : best;
}

const CH::ChEdge* CH::find_edge(int a, int b, int weight) const {
    for (const ChEdge& e : adj_[a]) {
        if (e.to == b && e.weight == weight) return &e;
    }
    return nullptr;
}

void CH::expand(int a, int b, int weight, std::vector<int>& out) const {
    const ChEdge* e = find_edge(a, b, weight);
    if (e == nullptr || e->via == -1) {
        out.push_back(b);  // 原始边（或查不到，退化为直接输出）
        return;
    }
    // 捷径 a—b 代表真实路径 a → via → b，递归展开两段
    expand(a, e->via, e->w1, out);
    expand(e->via, b, e->w2, out);
}

}  // namespace nav
