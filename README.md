# nav-engine

A from-scratch **C++17 route planning engine**: Dijkstra, A*, ALT (A\* + Landmarks + Triangle inequality) and **Contraction Hierarchies**, running on real OpenStreetMap road networks — and able to output the actual node-by-node route, not just the distance.

Built to practice **modern C++ engineering** (CMake, unit tests, NDEBUG-safe assertions, clang-format, profiling with callgrind) and **route-planning algorithms** — the kind used by map/navigation and ride-hailing services.

## Algorithms

| Algorithm | Status | Description |
|-----------|--------|-------------|
| Dijkstra (SSSP + point-to-point) | ✅ | Baseline single-source and early-stopping variants |
| A* | ✅ | Admissible heuristic + tie-breaking (larger `g` first) |
| ALT | ✅ | A\* + landmarks + triangle-inequality lower bounds |
| Contraction Hierarchies | ✅ | Node contraction, witness search, shortcut unpacking |
| Path reconstruction | ✅ | All four algorithms return the full route (CH unpacks shortcuts recursively) |

## Build

```bash
cmake -S . -B build
cmake --build build
```

Requires a C++17 compiler and CMake ≥ 3.16. The default build type is `Release`
(`-O3 -DNDEBUG`); if you change the build type, re-run CMake with a cleaned cache
(`rm -rf build`) — CMake caches `CMAKE_BUILD_TYPE`.

> Note: because `Release` defines `NDEBUG`, the tests do **not** use `<cassert>` —
> they use an NDEBUG-safe `CHECK` macro (`tests/check.hpp`) so assertions remain active
> in optimized builds.

## Usage

Synthetic obstacle grid:

```bash
python3 tools/gen_grid.py 300 300 0.2 grid.txt
./build/nav_cli grid.txt
```

Real road network (fetched from OpenStreetMap via the Overpass API):

```bash
python3 tools/fetch_osm.py 22.26 114.14 22.33 114.22 hk_metro.txt
./build/nav_cli hk_metro.txt
```

`nav_cli` picks the two most distant nodes in the graph, runs every algorithm, verifies
that all of them agree on the shortest distance, and reconstructs + verifies the route.

## Benchmark results

Graph: Hong Kong metro area (Central + Tsim Sha Tsui), **42,173 nodes / 88,690 edges**.
Node counts are deterministic; timings are from `-O3` builds and vary a few percent run to run.

| Algorithm | Nodes expanded | Query time | Shortest distance |
|-----------|---------------|------------|-------------------|
| Dijkstra (point-to-point) | 41,758 | 2.8 ms | 17,939 |
| A\* (Euclidean heuristic) | 30,121 | 2.6 ms | 17,939 |
| A\* + ALT (farthest, K=8) | 1,303 | 0.13 ms | 17,939 |
| **Contraction Hierarchies** | **428** | **8.8 µs** | 17,939 |

CH preprocessing cost: ~145 ms (one-time, offline).

**CH expands ~98× fewer nodes than Dijkstra.** Interestingly, CH touches only 428 nodes
while the shortest path itself is **1,303 nodes long** — the 875-node gap is exactly what
shortcuts compress away. All three path-reconstructing algorithms independently produce the
same 1,303-node route, and a verifier re-sums the weights on the **original** graph to
confirm it equals the reported distance (which also proves shortcuts were fully unpacked).

### Landmark selection strategy × count K (ALT)

Nodes expanded, fewer is better:

| strategy | K=4 | K=8 | K=16 | K=32 |
|----------|-----|-----|------|------|
| random   | 16,196 | 12,333 | 8,365 | 4,825 |
| farthest | **1,303** | **1,303** | **1,303** | **1,303** |
| boundary | 1,652 | 1,633 | 1,628 | 1,303 |

The greedy "farthest" landmark selection saturates at K=4 (it picks the four corners), which
is optimal for a cross-city query. Random selection is unstable and needs ~8× more landmarks
to approach it — a concrete demonstration that **landmark placement, not just landmark count,
determines ALT's performance**.

## Performance engineering

The CH query was optimised on top of `callgrind` data rather than intuition:

1. **Build configuration first.** The project originally had no `CMAKE_BUILD_TYPE`, so every
   measurement was from an unoptimised build. Setting `Release` improved every algorithm by
   3–6× — before touching a single line of algorithm code.
2. **Profile.** `callgrind` showed `CH::query` at 26.3% of all instructions, with `memset`
   (6.9%) plus the allocator (`malloc`/`free`/`operator new`, ~15%) dominating. The bottleneck
   was not the search — it was **allocating and zeroing ~422 KB of per-query scratch arrays**
   plus an O(n) scan over all nodes to find the meeting point.
3. **Optimise.** Reusable member buffers + a "touched list" that resets only the ~428 nodes
   actually visited (instead of clearing all 42,173), an O(settled) meeting scan, and a manual
   heap over a reused vector (removing the last per-query allocation).
4. **Re-measure.** CH query: **26.8 µs → 8.8 µs (3.1×)**; the first (cold) query improved
   0.70 ms → 0.044 ms (16×), because the per-query 422 KB allocation is gone.

Known remaining costs (measured, not yet addressed): iostream-based file loading (~5.6% of
instructions — `operator>>` formatting is slow), `std::unordered_map` in the witness search,
and `std::function` type-erasure in A\*'s hot loop (A\* does *fewer* instructions than Dijkstra
yet can take *more* time — an instruction-count profile cannot see indirect-call costs).

## Data format

The engine reads a simple text format (produced by `tools/gen_grid.py` and `tools/fetch_osm.py`):

```
n m            # node count, edge count
x y            # n lines of node coordinates
...
u v w          # m lines of directed edges
...
```

## Tests

```bash
ctest --test-dir build --output-on-failure
```

- `test_dijkstra` — distances, path reconstruction, `path_cost` edge cases
- `test_astar` — heuristic correctness, path reconstruction, unreachable handling
- `test_ch` — CH vs Dijkstra on all node pairs, plus a **6-cycle graph that is guaranteed to
  produce a shortcut**, so the recursive shortcut-unpacking path is always exercised

## Project layout

```
nav-engine/
├── CMakeLists.txt          # build recipe (default: Release)
├── include/nav/            # interfaces (headers)
│   ├── graph.hpp
│   ├── dijkstra.hpp
│   ├── astar.hpp
│   ├── landmarks.hpp
│   ├── ch.hpp
│   ├── path.hpp
│   ├── io.hpp
│   └── stats.hpp
├── src/                    # implementations
│   ├── dijkstra.cpp
│   ├── astar.cpp
│   ├── landmarks.cpp
│   ├── ch.cpp
│   ├── path.cpp
│   └── io.cpp
├── app/main.cpp            # benchmark + verification CLI
├── tests/                  # unit tests
│   ├── check.hpp
│   ├── test_dijkstra.cpp
│   ├── test_astar.cpp
│   └── test_ch.cpp
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
