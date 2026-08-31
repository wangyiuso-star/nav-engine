#pragma once

#include <vector>

#include "nav/graph.hpp"
#include "nav/stats.hpp"

namespace nav {

// Contraction Hierarchies（无向图版）
//
// 预处理：按节点重要性逐个「收缩」，必要时在邻居间加捷径边；
// 查询：双向、只沿「rank 上升」方向走，在峰点相遇。
class CH {
   public:
    // 预处理整张图（可能较慢，取决于图规模）
    explicit CH(const Graph& graph);

    // 查询 s -> t 的最短距离；不可达返回 -1。
    // stats 非空时统计双向共 settle 的节点数。
    int query(int s, int t, Stats* stats = nullptr) const;

    int node_count() const { return n_; }

   private:
    int n_;
    std::vector<int> rank_;                    // rank_[v] 越大越「重要」（越晚收缩）
    std::vector<std::vector<Edge>> up_graph_;  // 只含「低 rank -> 高 rank」的边
};

}  // namespace nav
