#include <iostream>
#include <vector>

#include "check.hpp"
#include "nav/dijkstra.hpp"
#include "nav/path.hpp"

int main() {
    // 已知答案的小图，用来验证 Dijkstra 正确
    nav::Graph g(4);
    g[0] = {{1, 1}, {2, 4}};
    g[1] = {{2, 2}, {3, 6}};
    g[2] = {{3, 3}};
    g[3] = {};

    auto dist = nav::dijkstra(g, 0);
    CHECK(dist[0] == 0);
    CHECK(dist[1] == 1);
    CHECK(dist[2] == 3);
    CHECK(dist[3] == 6);

    // ---- 点到点 + 路径重建 ----
    std::vector<int> path;

    // 0 -> 3：路径 0->1->2->3，总权 1+2+3 = 6
    CHECK(nav::dijkstra_point_to_point(g, 0, 3, nullptr, &path) == 6);
    CHECK(!path.empty());
    CHECK(path.front() == 0);
    CHECK(path.back() == 3);
    CHECK(nav::path_cost(g, path) == 6);

    // 0 -> 1：路径 0->1，总权 1
    CHECK(nav::dijkstra_point_to_point(g, 0, 1, nullptr, &path) == 1);
    CHECK(path.front() == 0);
    CHECK(path.back() == 1);
    CHECK(nav::path_cost(g, path) == 1);

    // src == target：单节点路径 [2]，总权 0
    CHECK(nav::dijkstra_point_to_point(g, 2, 2, nullptr, &path) == 0);
    CHECK(path.size() == 1);
    CHECK(path.front() == 2);
    CHECK(path.back() == 2);
    CHECK(nav::path_cost(g, path) == 0);

    // 不可达：3 没有出边，3 -> 0 到不了；path 必须被清空
    path.assign(5, 42);  // 先塞垃圾，验证确实被 clear()
    CHECK(nav::dijkstra_point_to_point(g, 3, 0, nullptr, &path) == -1);
    CHECK(path.empty());

    // path_cost 的边界：空路径 / 单节点返回 0
    std::vector<int> empty_path;
    CHECK(nav::path_cost(g, empty_path) == 0);
    std::vector<int> single = {2};
    CHECK(nav::path_cost(g, single) == 0);

    // path_cost 的边界：相邻两点之间没有边 -> -1
    std::vector<int> bad = {3, 0};  // 只有 0->3 方向不存在，3 没有出边
    CHECK(nav::path_cost(g, bad) == -1);

    std::cout << "all tests passed\n";
    return 0;
}
