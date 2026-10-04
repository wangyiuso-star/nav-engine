#include <iostream>
#include <vector>

#include "check.hpp"
#include "nav/ch.hpp"
#include "nav/dijkstra.hpp"
#include "nav/path.hpp"

int main() {
    // ---- 用例 1：5 节点无向图，对所有 (s,t) 对比 CH / Dijkstra 并校验路径 ----
    {
        nav::Graph g(5);
        g[0] = {{1, 2}, {2, 5}};
        g[1] = {{0, 2}, {2, 1}, {3, 4}};
        g[2] = {{0, 5}, {1, 1}, {3, 1}};
        g[3] = {{1, 4}, {2, 1}, {4, 3}};
        g[4] = {{3, 3}};

        nav::CH ch(g);

        for (int s = 0; s < 5; ++s) {
            for (int t = 0; t < 5; ++t) {
                const int expected = nav::dijkstra_point_to_point(g, s, t);

                std::vector<int> p;
                const int got = ch.query(s, t, nullptr, &p);
                CHECK(got == expected);

                if (expected < 0) {
                    CHECK(p.empty());  // 不可达：路径应为空
                } else if (s == t) {
                    CHECK(p.size() == 1 && p[0] == s);
                } else {
                    CHECK(!p.empty());
                    CHECK(p.front() == s);
                    CHECK(p.back() == t);
                    // 关键：路径必须已完全展开为原始边，且总权重 == 距离
                    CHECK(nav::path_cost(g, p) == expected);
                }
            }
        }
    }

    // ---- 用例 2：构造**必然产生捷径**的图，专门检验捷径展开 ----
    // 6 环：0-1-3-4-5-2-0，权重全 1。
    // 收缩节点 0 时，1 到 2 绕开 0 的唯一替代路径长 4 > 捷径长 2（1+1），
    // 故必然产生捷径 1—2（via 0，权重 2）。查询 1→2 的路径若未展开，
    // 会得到 [1,2]，但原图中不存在边 1-2 → path_cost 返回 -1，测试立刻失败。
    {
        nav::Graph h(6);
        h[0] = {{1, 1}, {2, 1}};
        h[1] = {{0, 1}, {3, 1}};
        h[2] = {{0, 1}, {5, 1}};
        h[3] = {{1, 1}, {4, 1}};
        h[4] = {{3, 1}, {5, 1}};
        h[5] = {{4, 1}, {2, 1}};

        nav::CH ch2(h);

        std::vector<int> p;
        const int d = ch2.query(1, 2, nullptr, &p);
        CHECK(d == 2);  // 1-0-2
        CHECK(p.size() == 3);
        CHECK(p.front() == 1);
        CHECK(p.back() == 2);
        CHECK(p[1] == 0);  // 捷径 1—2 必须被展开出中间节点 0
        CHECK(nav::path_cost(h, p) == 2);

        // 反向同样要正确
        std::vector<int> q;
        const int d2 = ch2.query(2, 1, nullptr, &q);
        CHECK(d2 == 2);
        CHECK(q.front() == 2 && q.back() == 1 && q[1] == 0);
        CHECK(nav::path_cost(h, q) == 2);

        // 同一实例反复查询：确认缓冲复位正确（不变式不被破坏）
        for (int i = 0; i < 100; ++i) {
            std::vector<int> r;
            CHECK(ch2.query(1, 2, nullptr, &r) == 2);
            CHECK(nav::path_cost(h, r) == 2);
        }
    }

    std::cout << "all tests passed\n";
    return 0;
}
