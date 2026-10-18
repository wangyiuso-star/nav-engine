#pragma once

#include <cstdio>
#include <cstdlib>

// 不依赖 NDEBUG 的断言：Release 下依然真实校验。
// 条件为假时向 stderr 打印条件、文件名、行号，然后以非 0 退出。
#define CHECK(cond)                                                                         \
    do {                                                                                    \
        if (!(cond)) {                                                                      \
            std::fprintf(stderr, "CHECK failed: %s at %s:%d\n", #cond, __FILE__, __LINE__); \
            std::exit(1);                                                                   \
        }                                                                                   \
    } while (0)
