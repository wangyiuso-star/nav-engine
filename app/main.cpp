#include "nav/dijkstra.hpp"

#include <cstddef>
#include <iostream>

int main() {
    // 一个 5 个节点的小图，验证 Dijkstra
    nav::Graph g(5);
    g[0] = {{1, 4}, {2, 1}};
    g[2] = {{1, 2}, {3, 5}};
    g[1] = {{3, 1}};
    g[3] = {{4, 3}};
    g[4] = {};

    auto dist = nav::dijkstra(g, 0);
    for (std::size_t i = 0; i < dist.size(); ++i) {
        std::cout << "dist[0 -> " << i << "] = " << dist[i] << '\n';
    }
    return 0;
}
