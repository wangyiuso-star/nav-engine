#!/usr/bin/env python3
"""从 OpenStreetMap 抓取一个区域的路网，转成 nav-engine 的格式。

用法: python3 fetch_osm.py <南纬> <西经> <北纬> <东经> <输出文件>
示例: python3 fetch_osm.py 22.37 114.17 22.43 114.24 hk_shatin.txt

说明:
- 只抓机动车道路（motorway/trunk/primary/secondary/tertiary/residential...）
- 边权 = 投影后的直线距离（米），启发式用欧氏距离即天然可采纳
- 为简化，忽略单行道（所有路都当双向）
"""
import json
import math
import sys
import time
import urllib.error
import urllib.parse
import urllib.request

HIGHWAY_RE = (
    "^(motorway|trunk|primary|secondary|tertiary|residential|unclassified"
    "|living_street|motorway_link|trunk_link|primary_link|secondary_link|tertiary_link)$"
)

# 多个 Overpass 镜像，第一个失败就换下一个
ENDPOINTS = [
    "https://overpass-api.de/api/interpreter",
    "https://overpass.kumi.systems/api/interpreter",
]

HEADERS = {"User-Agent": "nav-engine-learning/0.1 (education project)"}


def fetch(south, west, north, east):
    query = (
        f'[out:json][timeout:90];'
        f'way["highway"~"{HIGHWAY_RE}"]({south},{west},{north},{east});'
        f'(._;>;);out body;'
    )
    params = urllib.parse.urlencode({"data": query})

    last_err = ""
    for ep in ENDPOINTS:
        url = f"{ep}?{params}"
        print(f"请求: {ep}")
        try:
            req = urllib.request.Request(url, headers=HEADERS)
            with urllib.request.urlopen(req, timeout=120) as resp:
                return json.loads(resp.read().decode("utf-8"))
        except urllib.error.HTTPError as e:
            body = e.read().decode("utf-8", "ignore")[:300]
            last_err = f"HTTP {e.code}: {body}"
            print(f"  失败: HTTP {e.code}")
        except Exception as e:  # 网络错误等
            last_err = str(e)
            print(f"  失败: {e}")
        time.sleep(1)

    print()
    print("所有镜像端点都失败了。")
    print("  若看到 HTTP 429（限流）或 504（超时）→ 等 1-2 分钟重跑即可；")
    print("  若看到 HTTP 400（查询语法错）→ 把上面打印的错误发给我。")
    sys.exit(1)


def project(lat, lon, lat0, lon0):
    # 等距圆柱投影：把经纬度转成以 (lat0, lon0) 为原点的米坐标
    x = (lon - lon0) * 111320.0 * math.cos(math.radians(lat0))
    y = (lat - lat0) * 111320.0
    return x, y


def main():
    south = float(sys.argv[1])
    west = float(sys.argv[2])
    north = float(sys.argv[3])
    east = float(sys.argv[4])
    out = sys.argv[5]

    data = fetch(south, west, north, east)

    nodes = {}
    ways = []
    for el in data.get("elements", []):
        if el["type"] == "node":
            nodes[el["id"]] = (el["lat"], el["lon"])
        elif el["type"] == "way":
            ways.append(el.get("nodes", []))

    if not nodes or not ways:
        print("错误：没抓到数据。bbox 可能太小/太偏，或网络问题，稍后重试。")
        sys.exit(1)

    lat0 = (south + north) / 2
    lon0 = (west + east) / 2

    ids = sorted(nodes.keys())
    idx = {nid: i for i, nid in enumerate(ids)}

    edges = []
    for w in ways:
        for a, b in zip(w, w[1:]):
            if a in nodes and b in nodes:
                x1, y1 = project(*nodes[a], lat0, lon0)
                x2, y2 = project(*nodes[b], lat0, lon0)
                d = math.hypot(x2 - x1, y2 - y1)
                if d > 0:
                    w_int = int(math.ceil(d))  # 向上取整，保证启发式可采纳
                    edges.append((idx[a], idx[b], w_int))
                    edges.append((idx[b], idx[a], w_int))  # 双向

    with open(out, "w") as f:
        f.write(f"{len(ids)} {len(edges)}\n")
        for nid in ids:
            x, y = project(*nodes[nid], lat0, lon0)
            f.write(f"{x:.1f} {y:.1f}\n")
        for u, v, d in edges:
            f.write(f"{u} {v} {d}\n")

    print(f"生成 {out}: {len(ids)} 节点, {len(edges)} 边")


if __name__ == "__main__":
    main()
