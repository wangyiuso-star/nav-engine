#!/usr/bin/env python3
"""渲染 nav-engine 的 README 演示图：docs/demo.gif（动图）+ docs/route.png（末帧）。

输入是地图文件（hk_metro.txt）和 `nav_cli --dump-path` 导出的路径文件
（每行 `node_id x y`）。不依赖 numpy，只用 Pillow，Windows / Linux 均可运行。

用法：
    python tools/render_demo.py
    python tools/render_demo.py hk_metro.txt --path docs/route_path.txt
"""
from __future__ import annotations

import argparse
import sys
from dataclasses import dataclass
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont

ROOT = Path(__file__).resolve().parent.parent

# ---- GitHub 深色配色 ----
BG = (13, 17, 23)          # #0d1117
EDGE = (42, 47, 56)        # #2a2f38 路网
ORANGE = (255, 122, 61)    # #ff7a3d 路径
GREEN = (63, 185, 80)      # 起点
RED = (248, 81, 73)        # 终点
TEXT = (230, 237, 243)
MUTED = (139, 148, 158)
CARD = (22, 27, 34)
BORDER = (56, 62, 70)

CANVAS_W, CANVAS_H = 1000, 620
MARGIN = 24

# 环境事实里的对比数字（与 CLI 实测输出一致）
COMPARISON = [
    ("Dijkstra", "41,758", "2.96"),
    ("A* + Euclidean", "30,121", "2.83"),
    ("A* + ALT", "1,303", "0.119"),
    ("CH (bidirectional)", "428", "0.049"),
]
CH_NOTE = "← 98× fewer than Dijkstra"


@dataclass
class MapData:
    n: int
    m: int
    coords: list[tuple[float, float]]
    edges: list[tuple[int, int, int]]


@dataclass
class PathPoint:
    node: int
    x: float
    y: float


def _die(msg: str) -> None:
    print(f"错误: {msg}", file=sys.stderr)
    raise SystemExit(1)


def load_map(path: Path) -> MapData:
    try:
        with path.open("r", encoding="utf-8") as f:
            header = f.readline().split()
            if len(header) != 2:
                _die(f"{path}: expected 'n m' header, got {header!r}")
            n, m = int(header[0]), int(header[1])
            coords: list[tuple[float, float]] = []
            for i in range(n):
                parts = f.readline().split()
                if len(parts) < 2:
                    _die(f"{path}: missing coordinates on line {i + 2}")
                coords.append((float(parts[0]), float(parts[1])))
            edges: list[tuple[int, int, int]] = []
            for j in range(m):
                parts = f.readline().split()
                if len(parts) < 3:
                    _die(f"{path}: missing edge on line {n + j + 2}")
                u, v, w = int(parts[0]), int(parts[1]), int(parts[2])
                if not (0 <= u < n and 0 <= v < n):
                    _die(f"{path}: edge endpoint out of range on line {n + j + 2}: ({u}, {v})")
                edges.append((u, v, w))
    except FileNotFoundError:
        _die(f"map file not found: {path}")
    except OSError as exc:
        _die(f"cannot read map file {path}: {exc}")
    except ValueError as exc:
        _die(f"invalid map file {path}: {exc}")
    return MapData(n, m, coords, edges)


def load_path(path: Path, n: int) -> list[PathPoint]:
    points: list[PathPoint] = []
    try:
        with path.open("r", encoding="utf-8") as f:
            for lineno, line in enumerate(f, 1):
                line = line.strip()
                if not line or line.startswith("#"):
                    continue
                parts = line.split()
                if len(parts) < 3:
                    _die(f"{path}: line {lineno} must be 'node_id x y'")
                node, x, y = int(parts[0]), float(parts[1]), float(parts[2])
                if not 0 <= node < n:
                    _die(f"{path}: node id {node} out of range (0..{n - 1}) on line {lineno}")
                points.append(PathPoint(node, x, y))
    except FileNotFoundError:
        _die(
            f"path file not found: {path}\n"
            f"      generate it first: ./build-demo/nav_cli hk_metro.txt --dump-path {path}"
        )
    except OSError as exc:
        _die(f"cannot read path file {path}: {exc}")
    if len(points) < 2:
        _die(f"path file {path} needs at least 2 nodes, got {len(points)}")
    return points


def load_font(size: int, *, mono: bool = False, bold: bool = False) -> ImageFont.FreeTypeFont:
    """按平台依次尝试系统字体，全部缺失时降级到 Pillow 默认字体。"""
    if mono:
        candidates = [
            "C:/Windows/Fonts/consola.ttf",
            "C:/Windows/Fonts/cour.ttf",
            "/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf",
        ]
    elif bold:
        candidates = [
            "C:/Windows/Fonts/segoeuib.ttf",
            "C:/Windows/Fonts/arialbd.ttf",
            "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf",
        ]
    else:
        candidates = [
            "C:/Windows/Fonts/segoeui.ttf",
            "C:/Windows/Fonts/arial.ttf",
            "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
        ]
    for candidate in candidates:
        p = Path(candidate)
        if p.exists():
            try:
                return ImageFont.truetype(str(p), size)
            except OSError:
                continue
    try:
        return ImageFont.load_default(size=size)
    except TypeError:  # Pillow < 10.1
        return ImageFont.load_default()


def text_width(draw: ImageDraw.ImageDraw, text: str, font: ImageFont.ImageFont) -> int:
    return int(draw.textlength(text, font=font))


def compute_transform(map_data: MapData) -> tuple[float, float, float, float, float]:
    """返回 (scale, min_x, max_y, offset_x, offset_y)：等比缩放 + 24px 边距 + 居中。"""
    xs = [p[0] for p in map_data.coords]
    ys = [p[1] for p in map_data.coords]
    min_x, max_x = min(xs), max(xs)
    min_y, max_y = min(ys), max(ys)
    span_x = max_x - min_x or 1.0
    span_y = max_y - min_y or 1.0
    scale = min((CANVAS_W - 2 * MARGIN) / span_x, (CANVAS_H - 2 * MARGIN) / span_y)
    offset_x = (CANVAS_W - span_x * scale) / 2.0
    offset_y = (CANVAS_H - span_y * scale) / 2.0
    return scale, min_x, max_y, offset_x, offset_y


def project(
    x: float, y: float, transform: tuple[float, float, float, float, float]
) -> tuple[float, float]:
    scale, min_x, max_y, offset_x, offset_y = transform
    return offset_x + (x - min_x) * scale, offset_y + (max_y - y) * scale


def draw_network(draw: ImageDraw.ImageDraw, map_data: MapData, transform: tuple) -> None:
    """一次性画出全部有向边（88,690 条，逐条 line 调用约 1 秒）。"""
    scale, min_x, max_y, offset_x, offset_y = transform
    for u, v, _w in map_data.edges:
        x1 = offset_x + (map_data.coords[u][0] - min_x) * scale
        y1 = offset_y + (max_y - map_data.coords[u][1]) * scale
        x2 = offset_x + (map_data.coords[v][0] - min_x) * scale
        y2 = offset_y + (max_y - map_data.coords[v][1]) * scale
        draw.line((x1, y1, x2, y2), fill=EDGE, width=1)


def draw_panel(
    draw: ImageDraw.ImageDraw,
    box: tuple[int, int, int, int],
    *,
    fill=CARD,
    outline=BORDER,
    radius: int = 8,
) -> None:
    draw.rounded_rectangle(box, radius=radius, fill=fill, outline=outline, width=1)


def make_static_overlay(title_font, subtitle_font, mono_font, title, subtitle, summary):
    """标题 + 路线摘要的静态底板（半透明，避免文字被路网干扰）。"""
    overlay = Image.new("RGBA", (CANVAS_W, CANVAS_H), (0, 0, 0, 0))
    d = ImageDraw.Draw(overlay)
    title_w = text_width(d, title, title_font)
    sub_w = text_width(d, subtitle, subtitle_font)
    panel_w = max(title_w, sub_w) + 28
    panel_h = 74
    d.rounded_rectangle(
        (16, 10, 16 + panel_w, 10 + panel_h),
        radius=10,
        fill=(13, 17, 23, 200),
        outline=(56, 62, 70, 220),
        width=1,
    )
    summary_w = text_width(d, summary, mono_font)
    d.rounded_rectangle(
        (16, CANVAS_H - 16 - 30, 16 + summary_w + 24, CANVAS_H - 16),
        radius=6,
        fill=(13, 17, 23, 190),
        outline=(56, 62, 70, 200),
        width=1,
    )
    return overlay


def format_comparison_lines() -> list[str]:
    lines = []
    for name, expanded, ms in COMPARISON:
        line = f"{name:<19}{expanded:>7} nodes expanded{ms:>8} ms"
        if name.startswith("CH"):
            line += f"   {CH_NOTE}"
        lines.append(line)
    return lines


def render(
    map_data: MapData,
    path_points: list[PathPoint],
    route_distance: int,
    frame_count: int,
    colors: int,
    fps_ms: int,
    hold_ms: int,
) -> tuple[Image.Image, list[Image.Image]]:
    transform = compute_transform(map_data)
    path_xy = [project(p.x, p.y, transform) for p in path_points]

    title = "nav-engine — Contraction Hierarchies query"
    subtitle = (
        f"Hong Kong road network · {map_data.n:,} nodes / {map_data.m:,} edges"
    )
    summary = (
        f"route: {len(path_points):,} nodes · distance {route_distance:,}"
    )

    # 标题字号自适应，保证标题面板不压到右上角终点
    title_font = load_font(23, bold=True)
    probe = ImageDraw.Draw(Image.new("RGB", (1, 1)))
    title_size = 23
    while text_width(probe, title, title_font) > 650 and title_size > 16:
        title_size -= 1
        title_font = load_font(title_size, bold=True)
    subtitle_font = load_font(14)
    mono_font = load_font(13, mono=True)
    summary_font = load_font(12, mono=True)

    # 末帧对比表字号自适应，避免表盘侵入左下角路径起点
    table_font = load_font(13, mono=True)
    table_lines = format_comparison_lines()
    table_w = max(text_width(probe, line, table_font) for line in table_lines)
    if table_w > 620:
        table_font = load_font(12, mono=True)
        table_lines = format_comparison_lines()
        table_w = max(text_width(probe, line, table_font) for line in table_lines)

    base = Image.new("RGB", (CANVAS_W, CANVAS_H), BG)
    draw_network(ImageDraw.Draw(base), map_data, transform)
    static_overlay = make_static_overlay(
        title_font, subtitle_font, summary_font, title, subtitle, summary
    )

    title_pos = (28, 16)
    subtitle_pos = (28, 50)
    summary_pos = (28, CANVAS_H - 40)
    table_line_h = 19
    table_pad = 14
    table_panel_h = len(table_lines) * table_line_h + 16
    table_panel = (
        CANVAS_W - 16 - table_w - table_pad * 2,
        CANVAS_H - 16 - table_panel_h,
        CANVAS_W - 16,
        CANVAS_H - 16,
    )

    frames: list[Image.Image] = []
    total = len(path_xy)
    for idx in range(frame_count):
        count = max(2, round(total * (idx + 1) / frame_count))
        count = min(count, total)
        img = base.convert("RGBA")
        d = ImageDraw.Draw(img)
        d.line(path_xy[:count], fill=ORANGE + (255,), width=3)
        sx, sy = path_xy[0]
        ex, ey = path_xy[-1]
        d.ellipse((sx - 7, sy - 7, sx + 7, sy + 7), fill=GREEN + (255,), outline=BG + (255,))
        d.ellipse((ex - 7, ey - 7, ex + 7, ey + 7), fill=RED + (255,), outline=BG + (255,))
        img = Image.alpha_composite(img, static_overlay)
        d = ImageDraw.Draw(img)

        d.text(title_pos, title, font=title_font, fill=TEXT + (255,))
        d.text(subtitle_pos, subtitle, font=subtitle_font, fill=MUTED + (255,))
        d.text(summary_pos, summary, font=summary_font, fill=MUTED + (255,))

        if idx == frame_count - 1:
            draw_panel(d, table_panel, fill=CARD + (245,))
            ty = table_panel[1] + 8
            for line_idx, line in enumerate(table_lines):
                is_ch = line_idx == len(table_lines) - 1
                d.text(
                    (table_panel[0] + table_pad, ty),
                    line,
                    font=table_font,
                    fill=(ORANGE if is_ch else TEXT) + (255,),
                )
                ty += table_line_h
        else:
            progress = f"drawing route … {round(100 * count / total):d}%"
            progress_font = load_font(13, mono=True)
            card_w = text_width(d, progress, progress_font) + 28
            card = (CANVAS_W - 16 - card_w, CANVAS_H - 16 - 30, CANVAS_W - 16, CANVAS_H - 16)
            draw_panel(d, card, fill=CARD + (235,))
            d.text((card[0] + 14, card[1] + 7), progress, font=progress_font, fill=TEXT + (255,))

        frames.append(img.convert("RGB"))

    return frames[-1], frames


def save_gif(frames: list[Image.Image], out: Path, colors: int, fps_ms: int, hold_ms: int) -> None:
    """统一调色板转 P 模式，避免逐帧局部调色板导致文件膨胀。"""
    palette_src = frames[-1].convert("P", palette=Image.ADAPTIVE, colors=colors)
    paletted = [
        frame.quantize(palette=palette_src, dither=Image.Dither.NONE) for frame in frames
    ]
    durations = [fps_ms] * (len(paletted) - 1) + [hold_ms]
    paletted[0].save(
        out,
        save_all=True,
        append_images=paletted[1:],
        duration=durations,
        loop=0,
        optimize=True,
        disposal=1,
    )


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="渲染 nav-engine 演示 GIF 与静态路线图（Pillow）"
    )
    parser.add_argument("map", nargs="?", default=str(ROOT / "hk_metro.txt"), help="地图文件")
    parser.add_argument(
        "--path",
        "--dump-path",
        dest="path",
        default=str(ROOT / "docs" / "route_path.txt"),
        help="nav_cli --dump-path 导出的路径文件",
    )
    parser.add_argument("--gif", default=str(ROOT / "docs" / "demo.gif"), help="输出 GIF")
    parser.add_argument("--png", default=str(ROOT / "docs" / "route.png"), help="输出 PNG")
    parser.add_argument("--frames", type=int, default=48, help="帧数（含末帧）")
    parser.add_argument("--colors", type=int, default=128, help="GIF 调色板颜色数")
    parser.add_argument("--fps-ms", type=int, default=70, help="动画帧时长（毫秒）")
    parser.add_argument("--hold-ms", type=int, default=1600, help="末帧停留（毫秒）")
    args = parser.parse_args(argv)

    if not 2 <= args.frames <= 100:
        _die(f"--frames must be in 2..100, got {args.frames}")
    if not 2 <= args.colors <= 256:
        _die(f"--colors must be in 2..256, got {args.colors}")

    map_path = Path(args.map)
    path_path = Path(args.path)
    gif_path = Path(args.gif)
    png_path = Path(args.png)

    map_data = load_map(map_path)
    path_points = load_path(path_path, map_data.n)

    # 用地图边权沿路径精确求和（= CH 的 dist）
    weights = {(u, v): w for u, v, w in map_data.edges}
    route_distance = 0
    for a, b in zip(path_points, path_points[1:]):
        w = weights.get((a.node, b.node))
        if w is None:
            _die(
                f"path edge {a.node}->{b.node} is not in map {map_path}; "
                "path file does not match the map?"
            )
        route_distance += w

    last_frame, frames = render(
        map_data,
        path_points,
        route_distance,
        args.frames,
        args.colors,
        args.fps_ms,
        args.hold_ms,
    )

    for out in (gif_path, png_path):
        out.parent.mkdir(parents=True, exist_ok=True)
    save_gif(frames, gif_path, args.colors, args.fps_ms, args.hold_ms)
    last_frame.save(png_path, optimize=True)

    gif_kb = gif_path.stat().st_size / 1024
    png_kb = png_path.stat().st_size / 1024
    print(f"Map : {map_path} ({map_data.n:,} nodes / {map_data.m:,} edges)")
    print(f"Path: {path_path} ({len(path_points):,} nodes, distance {route_distance:,})")
    print(
        f"GIF : {gif_path} ({gif_kb:.1f} KB, {len(frames)} frames, "
        f"{CANVAS_W}x{CANVAS_H}, {args.fps_ms}ms/frame, last frame {args.hold_ms}ms)"
    )
    print(f"PNG : {png_path} ({png_kb:.1f} KB, {CANVAS_W}x{CANVAS_H})")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
