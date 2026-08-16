cmake_minimum_required(VERSION 3.16)
project(nav_engine CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_compile_options(-Wall -Wextra -Wpedantic)

# 核心算法库：图 + 最短路
add_library(nav
    src/dijkstra.cpp
)
target_include_directories(nav PUBLIC include)

# 命令行主程序
add_executable(nav_cli app/main.cpp)
target_link_libraries(nav_cli PRIVATE nav)

# 测试
enable_testing()
add_executable(test_dijkstra tests/test_dijkstra.cpp)
target_link_libraries(test_dijkstra PRIVATE nav)
add_test(NAME dijkstra COMMAND test_dijkstra)
