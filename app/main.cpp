#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

#include "nav/astar.hpp"
#include "nav/ch.hpp"
#include "nav/dijkstra.hpp"
#include "nav/io.hpp"
#include "nav/landmarks.hpp"

namespace {

// 贪心「最远点」采样：选 k 个坐标上尽量分散的地标
std::vector<int> pick_farthest(const nav::Map& map, int k) {
    const int n = static_cast<int>(map.graph.size());
    std::vector<int> landmarks;
    landmarks.push_back(0);

    while (static_cast<int>(landmarks.size()) < k) {
        int best_node = -1;
        double best_score = -1.0;
        for (int i = 0; i < n; ++i) {
            double min_d = 1e18;
            for (int L : landmarks) {
                double dx = map.coords[i].first - map.coords[L].first;
                double dy = map.coords[i].second - map.coords[L].second;
                double d = dx * dx + dy * dy;
                if (d < min_d) min_d = d;
            }
            if (min_d > best_score) {
                best_score = min_d;
                best_node = i;
            }
        }
        landmarks.push_back(best_node);
    }
    return landmarks;
}

}  // namespace

int main(int argc, char* argv[]) {
    std::string path = "grid.txt";
    if (argc > 1) path = argv[1];

    std::cout << "加载地图: " << path << " ...\n";
    nav::Map map = nav::load_map(path);
    const int n = static_cast<int>(map.graph.size());
    std::cout << "节点数 = " << n << "\n";

    // 选地理上尽量远的一对点：最左下 vs 最右上（x+y 最小 vs 最大）
    int src = 0;
    int target = 0;
    double min_xy = 1e18, max_xy = -1e18;
    for (int i = 0; i < n; ++i) {
        double s = map.coords[i].first + map.coords[i].second;
        if (s < min_xy) {
            min_xy = s;
            src = i;
        }
        if (s > max_xy) {
            max_xy = s;
            target = i;
        }
    }

    using Clock = std::chrono::high_resolution_clock;

    // ---- Dijkstra：点到点 ----
    nav::Stats dijkstra_stats;
    auto t0 = Clock::now();
    int dijkstra_dist = nav::dijkstra_point_to_point(map.graph, src, target, &dijkstra_stats);
    auto t1 = Clock::now();
    double dijkstra_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
    std::cout << "Dijkstra: " << dijkstra_ms << " ms, 展开节点 " << dijkstra_stats.nodes_expanded
              << ", dist = " << dijkstra_dist << "\n";

    // ---- A* + 欧氏启发式（基线）----
    auto euclidean = [&](int node) -> int {
        double dx = map.coords[node].first - map.coords[target].first;
        double dy = map.coords[node].second - map.coords[target].second;
        return static_cast<int>(std::floor(std::sqrt(dx * dx + dy * dy)));
    };

    nav::Stats astar_stats;
    t0 = Clock::now();
    int astar_dist = nav::astar(map.graph, src, target, euclidean, &astar_stats);
    t1 = Clock::now();
    double astar_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
    std::cout << "A*+欧氏:  " << astar_ms << " ms, 展开节点 " << astar_stats.nodes_expanded
              << ", dist = " << astar_dist << "\n";

    // ---- A* + ALT（farthest, K=8）----
    std::vector<int> landmarks = pick_farthest(map, 8);
    nav::Landmarks lm(map.graph, landmarks);
    auto alt = [&](int node) -> int { return lm.heuristic(node, target); };

    nav::Stats alt_stats;
    t0 = Clock::now();
    int alt_dist = nav::astar(map.graph, src, target, alt, &alt_stats);
    t1 = Clock::now();
    double alt_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
    std::cout << "A*+ALT:   " << alt_ms << " ms, 展开节点 " << alt_stats.nodes_expanded
              << ", dist = " << alt_dist << "\n";

    // ---- CH：预处理 + 双向查询 ----
    auto tp0 = Clock::now();
    nav::CH ch(map.graph);
    auto tp1 = Clock::now();
    double ch_prep_ms = std::chrono::duration<double, std::milli>(tp1 - tp0).count();

    nav::Stats ch_stats;
    auto tq0 = Clock::now();
    int ch_dist = ch.query(src, target, &ch_stats);
    auto tq1 = Clock::now();
    double ch_ms = std::chrono::duration<double, std::milli>(tq1 - tq0).count();
    std::cout << "CH: 预处理 " << ch_prep_ms << " ms, 查询 " << ch_ms << " ms, 展开节点 "
              << ch_stats.nodes_expanded << ", dist = " << ch_dist << "\n";

    // ---- CH 重复查询：测平均耗时（预处理不计入，循环里只调 query）----
    const int reps = 2000;
    long long checksum = 0;
    auto tr0 = Clock::now();
    for (int i = 0; i < reps; ++i) {
        checksum += ch.query(src, target);
    }
    auto tr1 = Clock::now();
    double avg_us = std::chrono::duration<double, std::micro>(tr1 - tr0).count() / reps;
    std::cout << "CH 查询平均 " << std::fixed << std::setprecision(2) << avg_us << " us (N=" << reps
              << "), checksum=" << checksum << "\n";

    // ---- 正确性核对 ----
    if (astar_dist != dijkstra_dist || alt_dist != dijkstra_dist || ch_dist != dijkstra_dist) {
        std::cout << "错误：结果不一致！\n";
        return 1;
    }
    std::cout << "结果一致 ✓\n";
    return 0;
}
