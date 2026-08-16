# nav-engine

一个从零开始、用于学习「现代 C++ 工程落地」的最小导航引擎。

## 当前进度
- [x] 图结构（邻接表）
- [x] Dijkstra 最短路径
- [ ] A* 算法
- [ ] ALT（Landmark 启发式）
- [ ] Contraction Hierarchies

## 怎么跑

```bash
cmake -S . -B build
cmake --build build
./build/nav_cli
ctest --test-dir build --output-on-failure
```

## 目录结构

```
nav-engine/
├── CMakeLists.txt        # 构建图纸
├── include/nav/          # 接口（头文件）
│   ├── graph.hpp
│   └── dijkstra.hpp
├── src/                  # 实现（源文件）
│   └── dijkstra.cpp
├── app/                  # 命令行入口
│   └── main.cpp
├── tests/                # 测试
│   └── test_dijkstra.cpp
└── README.md
```
