#include <cmath>
#include <iostream>
#include <utility>
#include <vector>

#include "check.hpp"
#include "nav/astar.hpp"
#include "nav/path.hpp"

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

    // ---- 路径重建 ----
    std::vector<int> path;

    // 0 -> 4：路径 0->1->2->3->4，总权 4
    CHECK(nav::astar(g, 0, 4, make_heuristic(4), nullptr, &path) == 4);
    CHECK(!path.empty());
    CHECK(path.front() == 0);
    CHECK(path.back() == 4);
    CHECK(nav::path_cost(g, path) == 4);

    // 2 -> 4：路径 2->3->4，总权 2
    CHECK(nav::astar(g, 2, 4, make_heuristic(4), nullptr, &path) == 2);
    CHECK(path.front() == 2);
    CHECK(path.back() == 4);
    CHECK(nav::path_cost(g, path) == 2);

    // 3 -> 4：路径 3->4，总权 1
    CHECK(nav::astar(g, 3, 4, make_heuristic(4), nullptr, &path) == 1);
    CHECK(path.front() == 3);
    CHECK(path.back() == 4);
    CHECK(nav::path_cost(g, path) == 1);

    // src == target：单节点路径 [4]，总权 0
    CHECK(nav::astar(g, 4, 4, make_heuristic(4), nullptr, &path) == 0);
    CHECK(path.size() == 1);
    CHECK(path.front() == 4);
    CHECK(path.back() == 4);
    CHECK(nav::path_cost(g, path) == 0);

    // 不可达：4 -> 0；path 必须被清空
    path.assign(5, 42);  // 先塞垃圾，验证确实被 clear()
    CHECK(nav::astar(g, 4, 0, make_heuristic(0), nullptr, &path) == -1);
    CHECK(path.empty());

    std::cout << "all tests passed\n";
    return 0;
}
