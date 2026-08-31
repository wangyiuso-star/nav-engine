#pragma once

namespace nav {

// 统计一次最短路查询展开了多少个节点（用于对比算法效率）
struct Stats {
    long long nodes_expanded = 0;
};

}  // namespace nav
