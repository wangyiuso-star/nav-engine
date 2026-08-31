#!/usr/bin/env python3
"""生成带障碍的网格路网（模拟城市街区），供 nav-engine 测试。

用法: python3 gen_grid.py <宽> <高> <障碍率0-1> <输出文件>
示例: python3 gen_grid.py 300 300 0.2 grid.txt
"""
import random
import sys


def main():
    w = int(sys.argv[1])
    h = int(sys.argv[2])
    obstacle_rate = float(sys.argv[3])
    out = sys.argv[4]
    random.seed(42)  # 固定随机种子，结果可复现

    n = w * h
    blocked = [0] * n
    for i in range(n):
        if random.random() < obstacle_rate:
            blocked[i] = 1
    blocked[0] = 0        # 起点可通行
    blocked[n - 1] = 0    # 终点可通行

    def idx(r, c):
        return r * w + c

    edges = []
    for r in range(h):
        for c in range(w):
            u = idx(r, c)
            if blocked[u]:
                continue  # 墙没有出边
            for dr, dc in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                rr, cc = r + dr, c + dc
                if 0 <= rr < h and 0 <= cc < w:
                    v = idx(rr, cc)
                    if not blocked[v]:
                        edges.append((u, v, 1))

    with open(out, "w") as f:
        f.write(f"{n} {len(edges)}\n")
        for r in range(h):
            for c in range(w):
                f.write(f"{c} {r}\n")  # 坐标 (x, y) = (列, 行)
        for u, v, w_ in edges:
            f.write(f"{u} {v} {w_}\n")

    print(f"生成 {out}: {n} 节点, {len(edges)} 边, 障碍 {sum(blocked)} 个")


if __name__ == "__main__":
    main()
