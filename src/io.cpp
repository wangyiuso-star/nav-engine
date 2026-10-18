#include "nav/io.hpp"

#include <fstream>
#include <stdexcept>

namespace nav {

Map load_map(const std::string& path) {
    std::ifstream in(path);
    if (!in) {
        throw std::runtime_error("error: cannot open map file: " + path);
    }

    int n = 0, m = 0;
    in >> n >> m;

    Map map;
    map.graph.resize(n);
    map.coords.resize(n);

    for (int i = 0; i < n; ++i) {
        double x = 0, y = 0;
        in >> x >> y;
        map.coords[i] = {x, y};
    }

    for (int j = 0; j < m; ++j) {
        int u = 0, v = 0, w = 0;
        in >> u >> v >> w;
        map.graph[u].push_back(Edge{v, w});
    }
    return map;
}

}  // namespace nav
