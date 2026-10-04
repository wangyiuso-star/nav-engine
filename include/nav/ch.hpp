#pragma once

#include <vector>

#include "nav/graph.hpp"
#include "nav/stats.hpp"

namespace nav {

// Contraction Hierarchies（无向图版）
//
// 预处理：按节点重要性逐个「收缩」，必要时在邻居间加捷径边；
// 查询：双向、只沿「rank 上升」方向走，在峰点相遇。
//
// 线程安全：query() 复用成员级缓冲区（消除每次查询的 O(n) 分配与清零），
// 因此 const 的 query() 会写入 mutable 成员 —— **它并非线程安全**。
// 生产实现应改用 thread_local 缓冲区，或让每个查询线程持有独立的查询对象。
//
// 内存取舍：为支持「捷径展开」（把压缩过的捷径还原成真实节点序列），
// 预处理后**同时保留**完整邻接表 adj_（原始边 + 所有捷径）与仅供查询使用的
// 向上图 up_graph_（adj_ 中「低 rank -> 高 rank」方向的子集）。
// 由于每条边按 ChEdge 计（4 个 int = 16 字节），边存储量约增加一倍，换取路径还原能力。
class CH {
   public:
    // CH 内部边：比 nav::Edge 多带「捷径信息」。
    //   via == -1：原始边（w1 / w2 无意义，恒为 0）
    //   via != -1：捷径 u—to，代表真实路径 u → via → to，
    //              即先走边 u→via（权重 w1），再走边 via→to（权重 w2），
    //              且必有 weight == w1 + w2。
    struct ChEdge {
        int to = -1;     // 目标节点
        int weight = 0;  // 边权（捷径时等于 w1 + w2）
        int via = -1;    // 捷径的中间节点；-1 表示原始边
        int w1 = 0;      // u -> via 的权重（via != -1 时有效）
        int w2 = 0;      // via -> to 的权重（via != -1 时有效）
    };

    // 预处理整张图（可能较慢，取决于图规模）
    explicit CH(const Graph& graph);

    // 查询 s -> t 的最短距离；不可达返回 -1。
    // stats 非空时统计双向共 settle 的节点数。
    int query(int s, int t, Stats* stats = nullptr) const;

    int node_count() const { return n_; }

   private:
    using Item = std::pair<int, int>;  // 小根堆元素：(key, node)

    int n_;
    std::vector<int> rank_;  // rank_[v] 越大越「重要」（越晚收缩）

    // 完整邻接表：原始边 + 收缩时加入的所有捷径（保留以支持捷径展开）
    std::vector<std::vector<ChEdge>> adj_;
    // 向上图：只含「低 rank -> 高 rank」的边（查询用）
    std::vector<std::vector<ChEdge>> up_graph_;

    // ---- 查询复用缓冲区：构造时分配一次，query 反复使用 ----
    // 不变式（每次 query 返回前必须恢复）：
    //   df_ / db_ 全为 INF；settled_f_ / settled_b_ 全为 0；
    //   四个 *_touched_ 列表为空。
    mutable std::vector<int> df_;          // 前向距离
    mutable std::vector<int> db_;          // 后向距离
    mutable std::vector<char> settled_f_;  // 前向已 settle 标记
    mutable std::vector<char> settled_b_;  // 后向已 settle 标记

    // 本次查询「被写过」的节点，用于精确复位（替代 O(n) 全量清零）
    mutable std::vector<int> df_touched_;         // df_ 被写过的节点（各一次）
    mutable std::vector<int> db_touched_;         // db_ 被写过的节点（各一次）
    mutable std::vector<int> settled_f_touched_;  // 前向 settle 顺序，兼作相遇扫描候选集
    mutable std::vector<int> settled_b_touched_;  // 后向 settle 顺序

    // 复用小根堆（clear 保留容量，避免每次查询重新分配）
    mutable std::vector<Item> fq_;
    mutable std::vector<Item> bq_;
};

}  // namespace nav
