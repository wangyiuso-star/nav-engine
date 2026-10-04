#include <cmath>
#include <iostream>
#include <utility>
#include <vector>

#include "check.hpp"
#include "nav/astar.hpp"

int main() {
    // 5 个带坐标的节点，终点是 4
    // 坐标用于计算「直线距离」作为启发式
    const std::vector<std::pair<int, int>> coords = {{0, 0}, {1, 0}, {2, 0}, {3, 0}, {3, 1}};

    nav::Graph g(5);
    g[0] = {{1, 1}, {2, 3}};
    g[1] = {{2, 1}, {3, 3}};
    g[2] = {{3, 1}};
    g[3] = {{4, 1}};
    g[4] = {};

    // 启发式工厂：给定终点 t，返回「从 n 到 t 的欧氏直线距离，向下取整」
    // 直线距离 <= 真实道路距离，所以可采纳。
    auto make_heuristic = [&](int t) {
        return [&, t](int n) -> int {
            double dx = coords[n].first - coords[t].first;
            double dy = coords[n].second - coords[t].second;
            return static_cast<int>(std::floor(std::sqrt(dx * dx + dy * dy)));
        };
    };

    // 0 -> 4 最短：0->1->2->3->4 = 1+1+1+1 = 4
    CHECK(nav::astar(g, 0, 4, make_heuristic(4)) == 4);
    CHECK(nav::astar(g, 1, 4, make_heuristic(4)) == 3);  // 1->2->3->4
    CHECK(nav::astar(g, 2, 4, make_heuristic(4)) == 2);  // 2->3->4
    CHECK(nav::astar(g, 3, 4, make_heuristic(4)) == 1);  // 3->4
    CHECK(nav::astar(g, 4, 4, make_heuristic(4)) == 0);  // 已在终点

    // 不可达：4 没有出边，4 -> 0 到不了
    CHECK(nav::astar(g, 4, 0, make_heuristic(0)) == -1);

    std::cout << "all tests passed\n";
    return 0;
}
