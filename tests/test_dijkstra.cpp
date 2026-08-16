#include <cassert>
#include <iostream>

#include "nav/dijkstra.hpp"

int main() {
    // 已知答案的小图，用来验证 Dijkstra 正确
    nav::Graph g(4);
    g[0] = {{1, 1}, {2, 4}};
    g[1] = {{2, 2}, {3, 6}};
    g[2] = {{3, 3}};
    g[3] = {};

    auto dist = nav::dijkstra(g, 0);
    assert(dist[0] == 0);
    assert(dist[1] == 1);
    assert(dist[2] == 3);
    assert(dist[3] == 6);

    std::cout << "all tests passed\n";
    return 0;
}
