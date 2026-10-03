#!/usr/bin/env python3
"""弓形覆盖路径规划：离散 Boustrophedon 分区 + 区内往返 + 区间 A*。

依赖：python -m pip install numpy matplotlib pillow
运行：python boustrophedon_demo.py
空矩形：python boustrophedon_demo.py --scene empty
保存：python boustrophedon_demo.py --gif demo.gif --png overview.png --no-show

教学模型：
  * free[y, x] == True 表示机器人中心可以访问的栅格；使用四邻接移动。
  * 覆盖率 = 已访问的不同自由栅格 / 全部自由栅格，不等同于圆形清洁盘的扫掠面积。
  * 逐行分区使用区间重叠关系；只有一对一连接才延续原子区域。
  * 在每个子区域内扫描全部行。栅格分辨率同时是相邻扫描行间距。
  * 区域顺序按到入口的 A* 路径长度贪心选择，不保证全局最短。
  * 未模拟机器人外形、障碍物膨胀、速度或最小转弯半径。
  * 地图必须是单个四连通自由区域；不连通时显式报错。

参考：Choset & Pignon, Coverage Path Planning: The Boustrophedon
Decomposition (1997), https://publications.ri.cmu.edu/
coverage-path-planning-the-boustrophedon-decomposition
"""

from __future__ import annotations

import argparse
import heapq
from dataclasses import dataclass
from pathlib import Path

import numpy as np

Point = tuple[int, int]  # (x, y)，数组访问顺序则是 [y, x]
Row = tuple[int, int, int]  # (y, x_left, x_right)，左右端点均包含


def make_map(scene: str) -> np.ndarray:
    """修改这里即可设置自己的地图；默认 30 列、20 行、两个矩形障碍。"""
    free = np.ones((20, 30), dtype=bool)
    if scene == "obstacles":
        free[4:10, 7:12] = False
        free[12:17, 19:25] = False
    return free


def row_intervals(row: np.ndarray) -> list[tuple[int, int]]:
    """把一行的 True 切成连续区间，例如 110111 -> [(0,1), (3,5)]。"""
    changes = np.diff(np.r_[0, row.astype(np.int8), 0])
    return list(zip(np.flatnonzero(changes == 1), np.flatnonzero(changes == -1) - 1))  # type: ignore


def decompose(free: np.ndarray) -> tuple[np.ndarray, list[list[Row]]]:
    """从下向上扫线：区间出生、消失、分裂或合并时结束/新建子区域。

    关键是比较相邻两行的连接关系，而不仅比较区间数量。
    labels[y,x] 为子区域编号，障碍物为 -1；cells 保存各区的扫描行。
    """
    labels = np.full(free.shape, -1, dtype=int)
    cells: list[list[Row]] = []
    previous: list[tuple[int, int, int]] = []  # (left, right, cell_id)
    for y, row in enumerate(free):
        intervals = row_intervals(row)
        parents = [
            [
                j
                for j, (pl, pr, _) in enumerate(previous)
                if max(left, pl) <= min(right, pr)
            ]
            for left, right in intervals
        ]
        child_count = [
            sum(j in group for group in parents) for j in range(len(previous))
        ]
        current = []
        for (left, right), group in zip(intervals, parents):
            if len(group) == 1 and child_count[group[0]] == 1:
                cell_id = previous[group[0]][2]
            else:
                cell_id = len(cells)
                cells.append([])
            labels[y, left : right + 1] = cell_id
            cells[cell_id].append((y, int(left), int(right)))
            current.append((left, right, cell_id))
        previous = current
    return labels, cells


def astar(free: np.ndarray, start: Point, goal: Point) -> list[Point] | None:
    """四邻接 A*：只负责两点间连接，不负责决定哪些区域需要覆盖。"""
    h, w = free.shape

    def distance(p):
        return abs(p[0] - goal[0]) + abs(p[1] - goal[1])

    queue = [(distance(start), 0, start)]
    cost = {start: 0}
    parent: dict[Point, Point] = {}
    while queue:
        _, g, point = heapq.heappop(queue)
        if g != cost[point]:
            continue
        if point == goal:
            path = [point]
            while point != start:
                point = parent[point]
                path.append(point)
            return path[::-1]
        x, y = point
        for dx, dy in ((1, 0), (0, 1), (-1, 0), (0, -1)):
            q = (x + dx, y + dy)
            nx, ny = q
            if not (0 <= nx < w and 0 <= ny < h and free[ny, nx]):
                continue
            ng = g + 1
            if ng < cost.get(q, float("inf")):
                cost[q] = ng
                parent[q] = point
                heapq.heappush(queue, (ng + distance(q), ng, q))
    return None


def sweep_cell(
    rows: list[Row], mask: np.ndarray, top_down: bool, start_right: bool
) -> list[Point]:
    """生成区内弓形路径：当前行左->右，下一行右->左，交替往返。

    相邻行边界不齐时，使用限制在本区域内的 A* 连接行端点。
    四种 (top_down, start_right) 组合对应四个入口方向。
    """
    ordered = rows[::-1] if top_down else rows
    path: list[Point] = []
    for i, (y, left, right) in enumerate(ordered):
        xs = range(left, right + 1)
        if start_right ^ bool(i % 2):
            xs = range(right, left - 1, -1)
        strip = [(x, y) for x in xs]
        if path:
            connector = astar(mask, path[-1], strip[0])
            if connector is None:
                raise RuntimeError("子区域内部不连通，请检查分区。")
            path.extend(connector[1:])
            path.extend(strip[1:])
        else:
            path.extend(strip)
    return path


@dataclass
class Plan:
    labels: np.ndarray
    cells: list[list[Row]]
    path: np.ndarray
    transit: np.ndarray  # transit[k]：到达 path[k] 的边是否为区间连接
    order: list[int]


def plan_coverage(free: np.ndarray, start: Point) -> Plan:
    """生成候选区内路径，并贪心选择最近的下一个区域入口。"""
    if free.ndim != 2 or not free.any():
        raise ValueError("地图需要是非空二维自由栅格图。")
    x, y = start
    h, w = free.shape
    if not (0 <= x < w and 0 <= y < h and free[y, x]):
        raise ValueError("起点必须位于自由栅格。")
    labels, cells = decompose(free)
    candidates = {
        cid: [
            sweep_cell(rows, labels == cid, top_down, start_right)
            for top_down in (False, True)
            for start_right in (False, True)
        ]
        for cid, rows in enumerate(cells)
    }
    path, transit, order = [start], [False], []
    while candidates:
        best = None
        for cid, routes in candidates.items():
            for index, route in enumerate(routes):
                link = astar(free, path[-1], route[0])
                if link is None:
                    continue
                key = (len(link), cid, index)  # 后两项保证结果可复现
                if best is None or key < best[0]:
                    best = (key, cid, route, link)
        if best is None:
            raise ValueError("存在起点无法到达的自由区域；无法用一条路径覆盖。")
        _, cid, route, link = best
        path.extend(link[1:])
        transit.extend([True] * (len(link) - 1))
        path.extend(route[1:])
        transit.extend([False] * (len(route) - 1))
        order.append(cid)
        del candidates[cid]
    return Plan(
        labels,
        cells,
        np.asarray(path, dtype=int),
        np.asarray(transit, dtype=bool),
        order,
    )


def validate(free: np.ndarray, plan: Plan) -> dict:
    """检查实际路径上的每一步，不能用计划覆盖面积代替实际访问率。"""
    p = plan.path
    if not np.all(free[p[:, 1], p[:, 0]]):
        raise AssertionError("路径进入障碍物。")
    if not np.all(np.abs(np.diff(p, axis=0)).sum(axis=1) == 1):
        raise AssertionError("路径存在跳跃或重复相邻点。")
    visited = np.zeros_like(free)
    visited[p[:, 1], p[:, 0]] = True
    if not np.array_equal(visited, free):
        raise AssertionError("存在遗漏的自由栅格。")
    return {
        "free_grids": int(free.sum()),
        "visited_grids": int(visited.sum()),
        "cells": len(plan.cells),
        "steps": len(p) - 1,
        "transit_steps": int(plan.transit.sum()),
        "revisits": len(p) - int(visited.sum()),
    }


def make_figure(free: np.ndarray, plan: Plan, resolution: float):
    """三联图：分区结果、完整路径、实际覆盖过程。图内英文以避免字体依赖。"""
    import matplotlib.pyplot as plt
    from matplotlib.collections import LineCollection
    from matplotlib.colors import ListedColormap
    from matplotlib.lines import Line2D
    from matplotlib.patches import Patch

    ink, blue, orange = "#183047", "#2563b8", "#d97706"
    wall, blank, covered = "#334155", "#edf1f5", "#a7dfc6"
    palette = [
        "#bfdbfe",
        "#bbf7d0",
        "#fde68a",
        "#ddd6fe",
        "#fed7aa",
        "#a5f3fc",
        "#fbcfe8",
        "#d9e3b0",
    ]
    plt.rcParams.update(
        {
            "font.family": "DejaVu Sans",
            "font.size": 10,
            "text.color": ink,
            "axes.labelcolor": ink,
            "xtick.color": "#64748b",
            "ytick.color": "#64748b",
        }
    )
    fig, axes = plt.subplots(1, 3, figsize=(14.8, 6.2), facecolor="#fafbfc")
    fig.subplots_adjust(left=0.045, right=0.985, bottom=0.28, top=0.78, wspace=0.18)
    fig.text(0.045, 0.935, "Boustrophedon coverage", fontsize=24, weight="bold")
    fig.text(
        0.045,
        0.875,
        "Split at connectivity changes. Sweep each cell. Connect the cells with A*.",
        fontsize=11,
        color="#526476",
    )

    h, w = free.shape
    extent = (0, w * resolution, 0, h * resolution)
    titles = (
        "1 / Sweep-line decomposition",
        "2 / Planned motion",
        "3 / Coverage replay",
    )
    for ax, title in zip(axes, titles):
        ax.set_title(title, loc="left", fontsize=12, weight="bold", pad=14)
        ax.set_xlim(extent[:2])
        ax.set_ylim(extent[2:])
        ax.set_aspect("equal")
        ax.set_xlabel("x [m]", fontsize=9)
        ax.set_ylabel("y [m]", fontsize=9, labelpad=2)
        ax.tick_params(labelsize=8, length=3)
        for spine in ax.spines.values():
            spine.set_color("#b9c4cf")
        # 栅格线帮助看清覆盖率按自由栅格统计，关闭 minor tick 本身。
        ax.set_xticks(np.arange(w + 1) * resolution, minor=True)
        ax.set_yticks(np.arange(h + 1) * resolution, minor=True)
        ax.grid(which="minor", color="white", linewidth=0.35, alpha=0.45)
        ax.tick_params(which="minor", length=0)

    colors = [wall] + [palette[i % len(palette)] for i in range(len(plan.cells))]
    axes[0].imshow(
        plan.labels + 1,
        origin="lower",
        extent=extent,
        cmap=ListedColormap(colors),
        vmin=0,
        vmax=len(plan.cells),
        interpolation="nearest",
    )
    for cid, rows in enumerate(plan.cells):
        # 标签放到子区域中间的一条扫描行，保证文字位于实际自由区域。
        y, left, right = rows[len(rows) // 2]
        axes[0].text(
            (left + right + 1) * resolution / 2,
            (y + 0.5) * resolution,
            f"C{cid + 1}",
            ha="center",
            va="center",
            fontsize=10,
            weight="bold",
            bbox={
                "boxstyle": "round,pad=0.26",
                "facecolor": "white",
                "alpha": 0.85,
                "edgecolor": "none",
            },
        )

    base = np.where(free, 1, 0)
    axes[1].imshow(
        base,
        origin="lower",
        extent=extent,
        cmap=ListedColormap([wall, "#f3f6f9"]),
        vmin=0,
        vmax=1,
        interpolation="nearest",
    )
    xy = (plan.path + 0.5) * resolution  # 路径通过栅格中心
    edges = np.stack([xy[:-1], xy[1:]], axis=1)
    is_link = plan.transit[1:]
    axes[1].add_collection(
        LineCollection(edges[~is_link], colors=blue, linewidths=1.2, zorder=4)  # type: ignore
    )
    axes[1].add_collection(
        LineCollection(
            edges[is_link],  # type: ignore
            colors=orange,
            linewidths=2.2,
            linestyles="dashed",
            zorder=5,  # type: ignore
        )
    )
    for i in range(7, len(edges), 15):
        if not is_link[i] and plan.path[i, 1] == plan.path[i + 1, 1]:
            a, b = edges[i]
            axes[1].annotate(
                "",
                xy=b,
                xytext=a,
                arrowprops={
                    "arrowstyle": "-|>",
                    "color": blue,
                    "lw": 1,
                    "mutation_scale": 8,
                },
                zorder=6,
            )
    for index, marker, color, text in (
        (0, "o", "#13866b", "S"),
        (-1, "s", "#b23d50", "F"),
    ):
        px, py = xy[index]
        axes[1].plot(
            px, py, marker=marker, color=color, ms=7, markeredgecolor="white", zorder=7
        )
        axes[1].annotate(
            text,
            (px, py),
            xytext=(7, 6),
            textcoords="offset points",
            weight="bold",
            fontsize=8,
            color=color,
            zorder=8,
        )

    coverage_image = axes[2].imshow(
        base,
        origin="lower",
        extent=extent,
        cmap=ListedColormap([wall, blank, covered]),
        vmin=0,
        vmax=2,
        interpolation="nearest",
    )
    sweep_trail = LineCollection([], colors=blue, linewidths=1.15, zorder=4)
    link_trail = LineCollection([], colors=orange, linewidths=2, zorder=5)
    axes[2].add_collection(sweep_trail)
    axes[2].add_collection(link_trail)
    (robot,) = axes[2].plot(
        [],
        [],
        "o",
        color="#c43b50",
        ms=9,
        markeredgecolor="white",
        markeredgewidth=1.5,
        zorder=8,
    )

    positions = [ax.get_position() for ax in axes]
    x0, x1, x2 = [p.x0 for p in positions]
    fig.text(x0, 0.205, "Each color is one sweep cell", color="#526476", fontsize=9)
    fig.legend(
        handles=[
            Line2D([], [], color=blue, lw=2, label="Cell sweep"),
            Line2D([], [], color=orange, lw=2, ls="--", label="A* transfer"),
        ],
        loc="lower left",
        bbox_to_anchor=(x1 - 0.004, 0.188),
        ncol=2,
        frameon=False,
        fontsize=9,
        columnspacing=1,
    )
    fig.legend(
        handles=[
            Patch(color=covered, label="Visited"),
            Patch(color=blank, label="Pending"),
            Patch(color=wall, label="Obstacle"),
        ],
        loc="lower left",
        bbox_to_anchor=(x2 - 0.004, 0.188),
        ncol=3,
        frameon=False,
        fontsize=9,
        columnspacing=0.8,
        handlelength=1,
    )
    nfree = int(free.sum())
    fig.text(
        x0,
        0.14,
        f"{len(plan.cells)} cells  /  {nfree} free grids",
        fontsize=14,
        weight="bold",
    )
    visit_order = " > ".join(f"C{cid + 1}" for cid in plan.order)
    fig.text(x0, 0.105, visit_order, fontsize=9, color="#526476")
    length = (len(xy) - 1) * resolution
    link_length = int(is_link.sum()) * resolution
    fig.text(x1, 0.14, f"{length:.1f} m total path", fontsize=14, weight="bold")
    fig.text(
        x1,
        0.105,
        f"Sweep {length - link_length:.1f} m  +  transfer {link_length:.1f} m",
        fontsize=9,
        color="#526476",
    )
    coverage_text = fig.text(x2, 0.14, "", fontsize=14, weight="bold")
    status_text = fig.text(x2, 0.105, "", fontsize=9, color="#526476")
    fig.text(
        0.045,
        0.038,
        f"GRID MODEL  |  pitch = {resolution:g} m  |  coverage = visited free grids / all free grids"
        "  |  point robot; no turning-radius constraint",
        fontsize=8.5,
        color="#667789",
    )

    # 首次访问时间用来重建任意动画帧；动画重播不会错误累计覆盖率。
    first_visit = np.full(free.shape, len(xy), dtype=int)
    np.minimum.at(first_visit, (plan.path[:, 1], plan.path[:, 0]), np.arange(len(xy)))

    def update(k: int):
        visited = free & (first_visit <= k)
        coverage_image.set_data(np.where(visited, 2, base))
        sweep_trail.set_segments(edges[:k][~is_link[:k]])  # type: ignore
        link_trail.set_segments(edges[:k][is_link[:k]])  # type: ignore
        robot.set_data([xy[k, 0]], [xy[k, 1]])
        count = int(visited.sum())
        coverage_text.set_text(f"{100 * count / nfree:5.1f}% grids visited")
        px, py = plan.path[k]
        stage = "TRANSFER" if plan.transit[k] else "SWEEP"
        if k == len(xy) - 1:
            stage = "COMPLETE"
        status_text.set_text(
            f"{count}/{nfree} grids  |  {stage}  |  C{plan.labels[py, px] + 1}"
        )
        return (
            coverage_image,
            sweep_trail,
            link_trail,
            robot,
            coverage_text,
            status_text,
        )

    update(0)
    return fig, update


def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("--scene", choices=("empty", "obstacles"), default="obstacles")
    parser.add_argument(
        "--resolution", type=float, default=0.4, help="栅格边长，单位 m"
    )
    parser.add_argument("--fps", type=int, default=20, help="动画每秒帧数")
    parser.add_argument("--stride", type=int, default=4, help="每动画帧前进多少路径点")
    parser.add_argument("--gif", type=Path, help="保存 GIF 到此路径")
    parser.add_argument("--png", type=Path, help="保存完成时的三联图到此路径")
    parser.add_argument("--no-show", action="store_true", help="不弹窗，适合服务器")
    args = parser.parse_args()
    if not np.isfinite(args.resolution) or args.resolution <= 0:
        parser.error("resolution 必须是有限正数")
    if args.fps < 1 or args.stride < 1:
        parser.error("fps 和 stride 必须大于零")
    free = make_map(args.scene)
    plan = plan_coverage(free, start=(0, 0))
    stats = validate(free, plan)
    print(f"scene={args.scene}; {stats}")
    print(f"自由栅格访问率: 100%; 路径长度: {stats['steps'] * args.resolution:.1f} m")
    print("检查通过：四邻接连续、未进入障碍、无遗漏自由栅格。")
    if args.no_show and not (args.gif or args.png):
        return

    if args.no_show:
        import matplotlib

        matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation, PillowWriter

    fig, update = make_figure(free, plan, args.resolution)
    if args.png:
        args.png.parent.mkdir(parents=True, exist_ok=True)
        update(len(plan.path) - 1)
        fig.savefig(args.png, dpi=150, facecolor=fig.get_facecolor())
        update(0)
        print(f"PNG: {args.png}")
    if args.gif or not args.no_show:
        # 开头、末尾停留片刻。stride 仅影响动画速度，不影响规划或覆盖统计。
        frames = (
            [0] * max(1, args.fps // 2)
            + list(range(0, len(plan.path), args.stride))
            + [len(plan.path) - 1] * args.fps
        )
        animation = FuncAnimation(
            fig,
            update,
            frames=frames,
            init_func=lambda: update(0),
            interval=1000 / args.fps,
            repeat=True,
            blit=False,
            cache_frame_data=False,
        )
        if args.gif:
            args.gif.parent.mkdir(parents=True, exist_ok=True)
            animation.save(args.gif, writer=PillowWriter(fps=args.fps), dpi=85)
            print(f"GIF: {args.gif}")
        if not args.no_show:
            plt.show()  # 保留 animation 引用，避免动画对象提前被回收。
    plt.close(fig)


if __name__ == "__main__":
    main()
