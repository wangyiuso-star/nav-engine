#include <chrono>
#include <cmath>
#include <exception>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

#include "nav/astar.hpp"
#include "nav/ch.hpp"
#include "nav/dijkstra.hpp"
#include "nav/io.hpp"
#include "nav/landmarks.hpp"
#include "nav/path.hpp"

namespace {

constexpr int kLabelWidth = 30;
constexpr int kTimeWidth = 6;
constexpr int kCountWidth = 8;

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

void print_usage() { std::cout << "usage: nav_cli <map file> [--dump-path <file>]\n"; }

// 统一的算法结果行：标签左对齐，耗时/展开节点定宽右对齐
void print_algorithm_line(const std::string& label, double ms, int nodes, int dist) {
    std::cout << std::left << std::setw(kLabelWidth) << label << ": " << std::right << std::fixed
              << std::setprecision(2) << std::setw(kTimeWidth) << ms << " ms"
              << std::setw(kCountWidth) << nodes << " nodes expanded   dist = " << dist << "\n";
}

// 路径重建校验行：  path <N> nodes, re-summed <C> (dist <D>) [ok]/[FAIL]
void print_path_check(const std::vector<int>& path, int cost, int dist, bool ok) {
    std::cout << "  path " << path.size() << " nodes, re-summed " << cost << " (dist " << dist
              << ") " << (ok ? "[ok]" : "[FAIL]") << "\n";
}

}  // namespace

int main(int argc, char* argv[]) {
    // argv[1] 以 '-' 开头视为参数（例如 --help），打印用法后退出。
    if (argc > 1 && argv[1][0] == '-') {
        print_usage();
        return 1;
    }

    std::string path = "grid.txt";
    if (argc > 1) path = argv[1];

    std::string dump_path;
    for (int i = 2; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--dump-path") {
            if (i + 1 >= argc) {
                std::cout << "error: --dump-path needs an output file\n";
                print_usage();
                return 1;
            }
            dump_path = argv[++i];
        } else {
            std::cout << "error: unknown argument: " << arg << "\n";
            print_usage();
            return 1;
        }
    }

    nav::Map map;
    try {
        map = nav::load_map(path);
    } catch (const std::exception& err) {
        std::cout << err.what() << "\n";
        return 1;
    }
    const int n = static_cast<int>(map.graph.size());
    int edge_count = 0;
    for (const auto& adj : map.graph) edge_count += static_cast<int>(adj.size());
    std::cout << "loading map: " << path << " ... " << n << " nodes, " << edge_count << " edges\n";

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
    print_algorithm_line("Dijkstra (point-to-point)", dijkstra_ms, dijkstra_stats.nodes_expanded,
                         dijkstra_dist);

    // ---- 路径重建（Dijkstra）：另跑一次（不计时，保持上面的性能数字可比）----
    {
        std::vector<int> dij_path;
        const int dij_dist_check =
            nav::dijkstra_point_to_point(map.graph, src, target, nullptr, &dij_path);
        const int dij_cost = nav::path_cost(map.graph, dij_path);
        const bool ok = (dij_dist_check == dijkstra_dist) && (dij_cost == dijkstra_dist);
        print_path_check(dij_path, dij_cost, dijkstra_dist, ok);
        if (!ok) {
            std::cout << "error: path verification failed (Dijkstra)\n";
            return 1;
        }
    }

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
    print_algorithm_line("A* + Euclidean", astar_ms, astar_stats.nodes_expanded, astar_dist);

    // ---- 路径重建（A*+欧氏）：另跑一次（不计时）----
    {
        std::vector<int> a_path;
        const int a_dist_check = nav::astar(map.graph, src, target, euclidean, nullptr, &a_path);
        const int a_cost = nav::path_cost(map.graph, a_path);
        const bool ok = (a_dist_check == astar_dist) && (a_cost == astar_dist);
        print_path_check(a_path, a_cost, astar_dist, ok);
        if (!ok) {
            std::cout << "error: path verification failed (A*)\n";
            return 1;
        }
    }

    // ---- A* + ALT（farthest, K=8）----
    std::vector<int> landmarks = pick_farthest(map, 8);
    nav::Landmarks lm(map.graph, landmarks);
    auto alt = [&](int node) -> int { return lm.heuristic(node, target); };

    nav::Stats alt_stats;
    t0 = Clock::now();
    int alt_dist = nav::astar(map.graph, src, target, alt, &alt_stats);
    t1 = Clock::now();
    double alt_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
    print_algorithm_line("A* + ALT (farthest, K=8)", alt_ms, alt_stats.nodes_expanded, alt_dist);

    // ---- CH：预处理 + 双向查询 ----
    auto tp0 = Clock::now();
    nav::CH ch(map.graph);
    auto tp1 = Clock::now();
    double ch_prep_ms = std::chrono::duration<double, std::milli>(tp1 - tp0).count();
    std::cout << std::left << std::setw(kLabelWidth) << "CH preprocessing" << ": " << std::right
              << std::fixed << std::setprecision(2) << std::setw(kTimeWidth) << ch_prep_ms
              << " ms\n";

    nav::Stats ch_stats;
    auto tq0 = Clock::now();
    int ch_dist = ch.query(src, target, &ch_stats);
    auto tq1 = Clock::now();
    double ch_ms = std::chrono::duration<double, std::milli>(tq1 - tq0).count();
    print_algorithm_line("CH first query", ch_ms, ch_stats.nodes_expanded, ch_dist);

    // ---- CH 重复查询：测平均耗时（预处理不计入，循环里只调 query）----
    const int reps = 2000;
    long long checksum = 0;
    auto tr0 = Clock::now();
    for (int i = 0; i < reps; ++i) {
        checksum += ch.query(src, target);
    }
    auto tr1 = Clock::now();
    double avg_us = std::chrono::duration<double, std::micro>(tr1 - tr0).count() / reps;
    std::cout << std::left << std::setw(kLabelWidth) << "CH 2000 queries" << ": " << std::right
              << std::fixed << std::setprecision(2) << std::setw(kTimeWidth) << avg_us
              << " us/query   checksum = " << checksum << "\n";

    // ---- CH 路径重建：另跑一次（不计时）；校验捷径是否被完整展开 ----
    {
        std::vector<int> ch_path;
        const int ch_dist_check = ch.query(src, target, nullptr, &ch_path);
        const int ch_cost = nav::path_cost(map.graph, ch_path);
        const bool ok = (ch_dist_check == ch_dist) && (ch_cost == ch_dist) && !ch_path.empty();
        print_path_check(ch_path, ch_cost, ch_dist, ok);
        if (!ok) {
            std::cout << "error: path verification failed (CH shortcut expansion)\n";
            return 1;
        }
    }

    // ---- 正确性核对 ----
    if (astar_dist != dijkstra_dist || alt_dist != dijkstra_dist || ch_dist != dijkstra_dist) {
        std::cout << "error: distances disagree\n";
        return 1;
    }
    std::cout << "all algorithms agree: dist = " << dijkstra_dist << "\n";

    // ---- 可选：导出 CH 路径（node_id x y，坐标 6 位小数）----
    if (!dump_path.empty()) {
        std::vector<int> out_path;
        const int out_dist = ch.query(src, target, nullptr, &out_path);
        if (out_dist < 0 || out_path.empty()) {
            std::cout << "error: path is unreachable\n";
            return 1;
        }
        std::ofstream out(dump_path);
        if (!out) {
            std::cout << "error: cannot write path file: " << dump_path << "\n";
            return 1;
        }
        out << std::fixed << std::setprecision(6);
        for (int node : out_path) {
            out << node << ' ' << map.coords[node].first << ' ' << map.coords[node].second << '\n';
        }
        std::cout << "path dumped to " << dump_path << " (" << out_path.size() << " nodes)\n";
    }
    return 0;
}
