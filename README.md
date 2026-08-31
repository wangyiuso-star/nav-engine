# nav-engine

A from-scratch **C++17 route planning engine**: Dijkstra, A*, and ALT (A* + Landmarks + Triangle inequality) over real OpenStreetMap road networks.

Built to practice **modern C++ engineering** (CMake, unit tests, sanitizers, benchmarking) and **route-planning algorithms** — the kind used by map/navigation and ride-hailing services.

## Algorithms

| Algorithm | Status | Description |
|-----------|--------|-------------|
| Dijkstra (SSSP + point-to-point) | ✅ | Baseline single-source and early-stopping variants |
| A* | ✅ | Admissible heuristic + tie-breaking (larger `g` first) |
| ALT | ✅ | A* + landmarks + triangle-inequality lower bounds |
| Contraction Hierarchies | 🚧 planned | Next step |

## Build

```bash
cmake -S . -B build
cmake --build build
```

Requires a C++17 compiler and CMake ≥ 3.16.

## Run the benchmarks

Synthetic obstacle grid:

```bash
python3 tools/gen_grid.py 300 300 0.2 grid.txt
./build/nav_cli grid.txt
```

Real road network (fetches from OpenStreetMap via the Overpass API):

```bash
python3 tools/fetch_osm.py 22.26 114.14 22.33 114.22 hk_metro.txt
./build/nav_cli hk_metro.txt
```

`nav_cli` picks the two most distant nodes and compares Dijkstra, A* (Euclidean heuristic), and A*+ALT across several landmark selection strategies and counts.

## Benchmark results

Graph: Hong Kong metro area (Central + Tsim Sha Tsui), **42,173 nodes / 88,690 edges**.

| Algorithm | Nodes expanded | Time | Shortest distance |
|-----------|---------------|------|-------------------|
| Dijkstra | 41,758 | 15.7 ms | 17,939 |
| A* (Euclidean) | 30,121 | 12.9 ms | 17,939 |
| **A* + ALT (farthest, K=8)** | **1,303** | **0.56 ms** | 17,939 |

Landmark selection strategy × count K (nodes expanded, fewer is better):

| strategy | K=4 | K=8 | K=16 | K=32 |
|----------|-----|-----|------|------|
| random   | 16,196 | 12,333 | 8,365 | 4,825 |
| farthest | **1,303** | **1,303** | **1,303** | **1,303** |
| boundary | 1,652 | 1,633 | 1,628 | 1,303 |

**~32× fewer nodes than Dijkstra** with ALT. The "farthest" landmark selection saturates at K=4 (the four corners), which is optimal for a cross-city query.

## Tests

```bash
ctest --test-dir build --output-on-failure
```

## Project layout

```
nav-engine/
├── CMakeLists.txt          # build recipe
├── include/nav/            # interfaces (headers)
│   ├── graph.hpp
│   ├── dijkstra.hpp
│   ├── astar.hpp
│   ├── landmarks.hpp
│   ├── io.hpp
│   └── stats.hpp
├── src/                    # implementations
│   ├── dijkstra.cpp
│   ├── astar.cpp
│   ├── landmarks.cpp
│   └── io.cpp
├── app/main.cpp            # benchmark CLI
├── tests/                  # unit tests
│   ├── test_dijkstra.cpp
│   └── test_astar.cpp
├── tools/                  # data generation / fetching scripts
│   ├── gen_grid.py
│   └── fetch_osm.py
├── .clang-format
├── .gitignore
├── LICENSE
└── README.md
```

## License

MIT
