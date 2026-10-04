#include <iostream>

#include "check.hpp"
#include "nav/ch.hpp"
#include "nav/dijkstra.hpp"

int main() {
    // 无向小图（每条边双向都有，权相同）
    nav::Graph g(5);
    g[0] = {{1, 2}, {2, 5}};
    g[1] = {{0, 2}, {2, 1}, {3, 4}};
    g[2] = {{0, 5}, {1, 1}, {3, 1}};
    g[3] = {{1, 4}, {2, 1}, {4, 3}};
    g[4] = {{3, 3}};

    nav::CH ch(g);

    // 对所有 (s, t) 对比 CH 和 Dijkstra
    for (int s = 0; s < 5; ++s) {
        for (int t = 0; t < 5; ++t) {
            int expected = nav::dijkstra_point_to_point(g, s, t);
            int got = ch.query(s, t);
            CHECK(got == expected);
        }
    }

    std::cout << "all tests passed\n";
    return 0;
}
