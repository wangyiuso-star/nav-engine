#pragma once

#include <string>
#include <utility>
#include <vector>

#include "nav/graph.hpp"

namespace nav {

// 一张带坐标的地图：图 + 每个节点的 (x, y) 坐标
struct Map {
    Graph graph;
    std::vector<std::pair<double, double>> coords;
};

// 从文本文件读取地图。
// 文件格式：
//   n m          （第一行：节点数 n、边数 m）
//   x y          （接着 n 行，每行一个节点的坐标）
//   ...
//   u v w        （接着 m 行，每行一条有向边 u->v，权重 w）
Map load_map(const std::string& path);

}  // namespace nav
