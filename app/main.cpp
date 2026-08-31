#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <iomanip>
#include <iostream>
#include <random>
#include <string>
#include <utility>
#include <vector>

#include "nav/astar.hpp"
#include "nav/dijkstra.hpp"
#include "nav/io.hpp"
#include "nav/landmarks.hpp"

namespace {

// --- 三种 landmark 选择策略 ---

// 1) 随机（固定种子，可复现）
std::vector<int> pick_random(const nav::Map& map, int k) {
    std::mt19937 rng(42);
    std::uniform_int_distribution<int> dist(0, static_cast<int>(map.graph.size()) - 1);
    std::vector<int> out;
    while (static_cast<int>(out.size()) < k) {
        int x = dist(rng);
        if (std::find(out.begin(), out.end(), x) == out.end()) out.push_back(x);
    }
    return out;
}

// 2) 贪心最远点采样：每次挑离已选地标（坐标距离）最远的节点
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

// 3) 边界均匀采样：在 bbox 周长上均匀取 k 个点，各找最近节点
std::vector<int> pick_boundary(const nav::Map& map, int k) {
    double min_x = 1e18, max_x = -1e18, min_y = 1e18, max_y = -1e18;
    for (const auto& p : map.coords) {
        min_x = std::min(min_x, p.first);
        max_x = std::max(max_x, p.first);
        min_y = std::min(min_y, p.second);
        max_y = std::max(max_y, p.second);
    }
    const double w = max_x - min_x;
    const double h = max_y - min_y;
    const double per = 2 * (w + h);

    std::vector<int> out;
    for (int i = 0; i < k; ++i) {
        double d = (static_cast<double>(i) / k) * per;
        double px, py;
        if (d < w) {
            px = min_x + d;
            py = min_y;
        } else if (d < w + h) {
            px = max_x;
            py = min_y + (d - w);
        } else if (d < 2 * w + h) {
            px = max_x - (d - w - h);
            py = max_y;
        } else {
            px = min_x;
            py = max_y - (d - 2 * w - h);
        }

        int best = 0;
        double bd = 1e18;
        for (int j = 0; j < static_cast<int>(map.coords.size()); ++j) {
            double dx = map.coords[j].first - px;
            double dy = map.coords[j].second - py;
            double dd = dx * dx + dy * dy;
            if (dd < bd) {
                bd = dd;
                best = j;
            }
        }
        out.push_back(best);
    }
    return out;
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

    // ---- ALT：三种策略 × 四种 K，看「展开节点」越少越好 ----
    auto run_alt = [&](const std::vector<int>& lms) {
        nav::Landmarks lm(map.graph, lms);
        auto h = [&](int node) -> int { return lm.heuristic(node, target); };
        nav::Stats s;
        int d = nav::astar(map.graph, src, target, h, &s);
        return std::make_pair(s.nodes_expanded, d);
    };

    const std::vector<int> ks = {4, 8, 16, 32};
    const std::vector<std::pair<std::string, std::function<std::vector<int>(int)>>> strategies = {
        {"random", [&](int k) { return pick_random(map, k); }},
        {"farthest", [&](int k) { return pick_farthest(map, k); }},
        {"boundary", [&](int k) { return pick_boundary(map, k); }},
    };

    std::cout << "\n=== ALT：landmark 策略 × 数量 K（展开节点，越少越好）===\n";
    std::cout << std::left << std::setw(10) << "strategy";
    for (int k : ks) std::cout << std::right << std::setw(9) << ("K=" + std::to_string(k));
    std::cout << "\n";

    bool all_ok = true;
    for (const auto& [name, fn] : strategies) {
        std::cout << std::left << std::setw(10) << name;
        for (int k : ks) {
            auto [expanded, d] = run_alt(fn(k));
            if (d != dijkstra_dist) all_ok = false;
            std::cout << std::right << std::setw(9) << expanded;
        }
        std::cout << "\n";
    }

    // ---- 正确性核对 ----
    if (astar_dist != dijkstra_dist || !all_ok) {
        std::cout << "错误：结果不一致！\n";
        return 1;
    }
    std::cout << "结果一致 ✓\n";
    return 0;
}
