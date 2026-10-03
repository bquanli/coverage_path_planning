#!/usr/bin/env python3
"""弓形覆盖路径规划：适合 Python 新手阅读的完整演示。

需要 Python 3.10 或更高版本。
安装依赖：python -m pip install numpy matplotlib pillow
运行演示：python boustrophedon_demo_beginner.py
空白地图：python boustrophedon_demo_beginner.py --scene empty
仅做验证：python boustrophedon_demo_beginner.py --no-show
保存图片：python boustrophedon_demo_beginner.py --png overview.png --no-show
保存动画：python boustrophedon_demo_beginner.py --gif demo.gif --no-show

阅读顺序：make_map -> decompose -> astar -> sweep_cell -> plan_coverage。
validate 负责检查结果，make_figure 负责绘图，main 负责组织程序。

模型约定：
1. free[y, x] 为 True，表示机器人中心可以访问这个栅格。
2. Point 使用 (x, y)，但 NumPy 数组使用 [y, x] 访问。
3. 只允许四邻接移动：右、上、左、下，不允许斜向移动。
4. 覆盖率按实际访问的自由栅格数量计算，不是清洁盘的扫掠面积。
5. 贪心选择最近区域入口，不保证总路径最短。
6. 不模拟机器人外形、障碍膨胀和转弯半径。
7. 自由空间不连通时，程序明确报错。
"""  # noqa: EXE001

from __future__ import annotations

import argparse
import heapq
import math
from collections.abc import Callable
from dataclasses import dataclass
from pathlib import Path

import numpy as np
from matplotlib.artist import Artist
from matplotlib.figure import Figure
from numpy.typing import NDArray

# 类型别名只是为类型起一个简短的名字，不会创建数组或坐标。
Point = tuple[int, int]  # (x, y)
Row = tuple[int, int, int]  # (y, left, right)，包含左右端点
Interval = tuple[int, int]  # (left, right)
ActiveInterval = tuple[int, int, int]  # (left, right, cell_id)
BoolArray = NDArray[np.bool_]  # 布尔数组；维数由具体变量决定
IntArray = NDArray[np.int64]
PlotPoint = tuple[float, float]  # 绘图时的米制坐标
Segment = list[PlotPoint]  # 一条线段，包含起点和终点
UpdateFunction = Callable[[int], tuple[Artist, ...]]


def make_map(scene: str) -> BoolArray:
    """创建 20 行、30 列的地图；True 是自由空间，False 是障碍。"""
    free: BoolArray = np.ones((20, 30), dtype=np.bool_)

    if scene == "obstacles":
        # 切片包含起点，不包含终点。例如 4:10 表示索引 4 到 9。
        free[4:10, 7:12] = False
        free[12:17, 19:25] = False

    return free


def row_intervals(row: BoolArray) -> list[Interval]:
    """例如 [True, True, False, True] 对应 [(0, 1), (3, 3)]。"""
    intervals: list[Interval] = []
    left: int | None = None

    for x in range(len(row)):
        if row[x]:
            if left is None:
                left = x
        else:
            if left is not None:
                intervals.append((left, x - 1))
                left = None

    # 扫描到行末时，最后一个自由区间可能还没有结束。
    if left is not None:
        intervals.append((left, len(row) - 1))

    return intervals


def decompose(free: BoolArray) -> tuple[IntArray, list[list[Row]]]:
    """
    逐行扫描，把自由空间分解成子区域。

    free[y, x]：
        True  表示自由空间
        False 表示障碍物

    返回：
        labels：每个栅格所属的子区域编号，障碍物为 -1
        cells：每个子区域包含的扫描区间
    """
    # 创建与地图同样大小的数组，初始值全部为 -1
    labels: IntArray = np.full(free.shape, -1, dtype=np.int64)
    # cells[cell_id] 保存某个子区域的所有扫描区间
    # 每个扫描区间表示为 (y, left, right)
    cells: list[list[Row]] = []
    previous: list[ActiveInterval] = []

    for y in range(free.shape[0]):
        row = free[y]
        intervals = row_intervals(row)

        # parents[i]：当前区间 i 连接了上一行的哪些区间。
        # 保存的是 previous 的索引，不是子区域编号。
        parents: list[list[int]] = []
        for left, right in intervals:
            parent_indices: list[int] = []
            for j in range(len(previous)):
                previous_left = previous[j][0]
                previous_right = previous[j][1]
                overlap_left = max(left, previous_left)
                overlap_right = min(right, previous_right)
                if overlap_left <= overlap_right:
                    parent_indices.append(j)
            parents.append(parent_indices)

        # child_count[j]：上一行区间 j 有多少个子区间。
        child_count: list[int] = [0] * len(previous)
        for parent_indices in parents:
            for parent_index in parent_indices:
                child_count[parent_index] += 1

        current: list[ActiveInterval] = []
        for i in range(len(intervals)):
            left, right = intervals[i]
            parent_indices = parents[i]

            # None 表示还没有找到可以继承的区域编号。
            # 这样避免用另一个布尔变量间接判断 parent_index 是否已赋值。
            inherited_cell_id: int | None = None
            if len(parent_indices) == 1:
                parent_index = parent_indices[0]
                if child_count[parent_index] == 1:
                    inherited_cell_id = previous[parent_index][2]

            if inherited_cell_id is None:
                cell_id = len(cells)
                cells.append([])
            else:
                cell_id = inherited_cell_id

            for x in range(left, right + 1):
                labels[y, x] = cell_id
            cells[cell_id].append((y, left, right))
            current.append((left, right, cell_id))

        previous = current

    return labels, cells


def astar(free: BoolArray, start: Point, goal: Point) -> list[Point] | None:
    """用四邻接 A* 连接两点；返回的路径包含起点和终点。"""
    height, width = free.shape

    # 先检查端点，避免负索引或障碍物起点被误认为合法。
    for x, y in (start, goal):
        if x < 0 or x >= width or y < 0 or y >= height:
            return None
        if not free[y, x]:
            return None

    def distance_to_goal(point: Point) -> int:
        """曼哈顿距离：横向距离加纵向距离。"""
        horizontal_distance = abs(point[0] - goal[0])
        vertical_distance = abs(point[1] - goal[1])
        return horizontal_distance + vertical_distance

    # 优先队列元素：(估计总代价 f，已走代价 g，坐标)。
    # heapq 每次弹出最小项；f 相同时再按后续字段比较。
    queue: list[tuple[int, int, Point]] = []
    heapq.heappush(queue, (distance_to_goal(start), 0, start))
    cost: dict[Point, int] = {start: 0}
    parent: dict[Point, Point] = {}
    directions: list[Point] = [(1, 0), (0, 1), (-1, 0), (0, -1)]

    while len(queue) > 0:
        entry = heapq.heappop(queue)
        current_cost = entry[1]
        point = entry[2]

        # 同一个点可能先后以不同代价入队；跳过已经过期的记录。
        if current_cost != cost[point]:
            continue

        if point == goal:
            path: list[Point] = [point]
            while point != start:
                point = parent[point]
                path.append(point)
            # 上面从终点回溯到起点，所以需要把列表反转。
            path.reverse()
            return path

        x, y = point
        for dx, dy in directions:
            next_x = x + dx
            next_y = y + dy
            if next_x < 0 or next_x >= width:
                continue
            if next_y < 0 or next_y >= height:
                continue
            if not free[next_y, next_x]:
                continue

            next_point: Point = (next_x, next_y)
            next_cost = current_cost + 1
            old_cost = cost.get(next_point)
            if old_cost is not None and next_cost >= old_cost:
                continue

            cost[next_point] = next_cost
            parent[next_point] = point
            estimated_total = next_cost + distance_to_goal(next_point)
            heapq.heappush(queue, (estimated_total, next_cost, next_point))

    return None


def sweep_cell(
    rows: list[Row],
    mask: BoolArray,
    top_down: bool,
    start_right: bool,
) -> list[Point]:
    """逐行往返覆盖一个子区域，用区域内部的 A* 连接相邻扫描行。

    rows 按 y 从小到大保存；图中 y 向上增长。
    top_down=True：先扫描 y 最大的行，即从上向下。
    start_right=True：第一行从右端开始，向左移动。
    """
    ordered_rows = rows.copy()
    if top_down:
        ordered_rows.reverse()

    path: list[Point] = []
    right_to_left = start_right

    for y, left, right in ordered_rows:
        if right_to_left:
            x_values = range(right, left - 1, -1)
        else:
            x_values = range(left, right + 1)

        strip: list[Point] = []
        for x in x_values:
            strip.append((x, y))

        if len(path) == 0:
            path.extend(strip)
        else:
            connector = astar(mask, path[-1], strip[0])
            if connector is None:
                raise RuntimeError("子区域内部不连通，请检查分区。")

            # connector[0] 已经是 path 的最后一个点，不重复添加。
            for index in range(1, len(connector)):
                path.append(connector[index])
            # 连接完成后已经到达 strip[0]，也不重复添加。
            for index in range(1, len(strip)):
                path.append(strip[index])

        # 下一行换方向，形成往复的弓形路径。
        right_to_left = not right_to_left

    return path


@dataclass
class Plan:
    """dataclass 自动生成初始化方法，用来集中保存规划结果。"""

    labels: IntArray
    cells: list[list[Row]]
    path: IntArray  # 形状为 (路径点数量, 2)，每行是 (x, y)
    transit: BoolArray  # 一维数组；第 k 项表示到达 path[k] 的边是否为区域间连接
    order: list[int]


@dataclass
class RouteChoice:
    """记录当前找到的最佳候选，避免含义难辨的多层元组。"""

    key: tuple[int, int, int]
    cell_id: int
    route: list[Point]
    link: list[Point]


def plan_coverage(free: BoolArray, start: Point) -> Plan:
    """生成每个区域的四种覆盖路径，然后贪心选择最近的入口。"""
    if free.ndim != 2 or not free.any():
        raise ValueError("地图需要是非空二维自由栅格图。")

    x, y = start
    height, width = free.shape
    if x < 0 or x >= width or y < 0 or y >= height:
        raise ValueError("起点超出了地图范围。")
    if not free[y, x]:
        raise ValueError("起点必须位于自由栅格。")

    labels, cells = decompose(free)

    # 字典：区域编号 -> 该区域的四种候选路径。
    candidates: dict[int, list[list[Point]]] = {}
    for cell_id in range(len(cells)):
        rows = cells[cell_id]
        mask: BoolArray = labels == cell_id
        routes: list[list[Point]] = []
        for top_down in (False, True):
            for start_right in (False, True):
                route = sweep_cell(rows, mask, top_down, start_right)
                routes.append(route)
        candidates[cell_id] = routes

    path: list[Point] = [start]
    transit: list[bool] = [False]
    order: list[int] = []

    while len(candidates) > 0:
        best: RouteChoice | None = None
        current_position = path[-1]

        for cell_id in candidates:  # noqa: PLC0206
            routes = candidates[cell_id]
            for route_index in range(len(routes)):
                route = routes[route_index]
                link = astar(free, current_position, route[0])
                if link is None:
                    continue

                # 元组从左向右比较：先比连接长度，再比区域和候选编号。
                # 后两项用于平局时固定选择，使结果可复现。
                key = (len(link), cell_id, route_index)
                if best is None or key < best.key:
                    best = RouteChoice(key, cell_id, route, link)

        if best is None:
            raise ValueError("存在起点无法到达的自由区域；无法用一条路径覆盖。")

        for index in range(1, len(best.link)):
            path.append(best.link[index])
            transit.append(True)
        for index in range(1, len(best.route)):
            path.append(best.route[index])
            transit.append(False)

        order.append(best.cell_id)
        del candidates[best.cell_id]

    return Plan(
        labels=labels,
        cells=cells,
        path=np.asarray(path, dtype=np.int64),
        transit=np.asarray(transit, dtype=np.bool_),
        order=order,
    )


def validate(free: BoolArray, plan: Plan) -> dict[str, int]:
    """逐点检查：路径合法、四邻接连续，并且访问所有自由栅格。"""
    if plan.path.ndim != 2 or plan.path.shape[1] != 2:
        raise AssertionError("路径必须是 N 行、2 列的坐标数组。")
    if len(plan.path) == 0:
        raise AssertionError("路径不能为空。")
    if plan.transit.ndim != 1 or len(plan.transit) != len(plan.path):
        raise AssertionError("transit 必须是一维数组，并且与路径点数量相同。")

    height, width = free.shape
    visited: BoolArray = np.zeros_like(free)
    for index in range(len(plan.path)):
        x = int(plan.path[index, 0])
        y = int(plan.path[index, 1])
        if x < 0 or x >= width or y < 0 or y >= height:
            raise AssertionError("路径超出了地图范围。")
        if not free[y, x]:
            raise AssertionError("路径进入障碍物。")

        if index > 0:
            previous_x = int(plan.path[index - 1, 0])
            previous_y = int(plan.path[index - 1, 1])
            step_distance = abs(x - previous_x) + abs(y - previous_y)
            if step_distance != 1:
                raise AssertionError("路径存在跳跃或重复相邻点。")

        visited[y, x] = True

    free_grids = 0
    visited_grids = 0
    for y in range(height):
        for x in range(width):
            if free[y, x]:
                free_grids += 1
                if not visited[y, x]:
                    raise AssertionError("存在遗漏的自由栅格。")
            if visited[y, x]:
                visited_grids += 1

    transit_steps = 0
    for index in range(1, len(plan.transit)):
        if plan.transit[index]:
            transit_steps += 1

    return {
        "free_grids": free_grids,
        "visited_grids": visited_grids,
        "cells": len(plan.cells),
        "steps": len(plan.path) - 1,
        "transit_steps": transit_steps,
        "revisits": len(plan.path) - visited_grids,
    }


def make_figure(
    free: BoolArray, plan: Plan, resolution: float
) -> tuple[Figure, UpdateFunction]:
    """绘制分区、完整路径和覆盖回放，并返回更新某一帧的函数。"""
    # pyplot 在 main 选择后端之后才导入，以支持 --no-show。
    import matplotlib.pyplot as plt
    from matplotlib.axes import Axes
    from matplotlib.collections import LineCollection
    from matplotlib.colors import ListedColormap
    from matplotlib.lines import Line2D
    from matplotlib.patches import Patch

    ink = "#183047"
    blue = "#2563b8"
    orange = "#d97706"
    wall = "#334155"
    blank = "#edf1f5"
    covered = "#a7dfc6"
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

    # 分别创建三个 Axes，避免 subplots 多种返回形状带来的类型歧义。
    figure = plt.figure(figsize=(14.8, 6.2), facecolor="#fafbfc")
    axes: list[Axes] = []
    for subplot_number in range(1, 4):
        axes.append(figure.add_subplot(1, 3, subplot_number))
    decomposition_axes = axes[0]
    path_axes = axes[1]
    replay_axes = axes[2]

    figure.subplots_adjust(left=0.045, right=0.985, bottom=0.28, top=0.78, wspace=0.18)
    figure.text(0.045, 0.935, "Boustrophedon coverage", fontsize=24, weight="bold")
    figure.text(
        0.045,
        0.875,
        "Split at connectivity changes. Sweep each cell. Connect the cells with A*.",
        fontsize=11,
        color="#526476",
    )

    height, width = free.shape
    extent = (0.0, width * resolution, 0.0, height * resolution)
    titles = [
        "1 / Sweep-line decomposition",
        "2 / Planned motion",
        "3 / Coverage replay",
    ]
    x_ticks: list[float] = []
    y_ticks: list[float] = []
    for x in range(width + 1):
        x_ticks.append(x * resolution)
    for y in range(height + 1):
        y_ticks.append(y * resolution)

    for index in range(len(axes)):
        axis = axes[index]
        axis.set_title(titles[index], loc="left", fontsize=12, weight="bold", pad=14)
        axis.set_xlim(extent[0], extent[1])
        axis.set_ylim(extent[2], extent[3])
        axis.set_aspect("equal")
        axis.set_xlabel("x [m]", fontsize=9)
        axis.set_ylabel("y [m]", fontsize=9, labelpad=2)
        axis.tick_params(labelsize=8, length=3)
        for spine in axis.spines.values():
            spine.set_color("#b9c4cf")
        axis.set_xticks(x_ticks, minor=True)
        axis.set_yticks(y_ticks, minor=True)
        axis.grid(which="minor", color="white", linewidth=0.35, alpha=0.45)
        axis.tick_params(which="minor", length=0)

    colors: list[str] = [wall]
    for cell_id in range(len(plan.cells)):
        palette_index = cell_id % len(palette)
        colors.append(palette[palette_index])

    # labels 中障碍为 -1，加 1 后，障碍对应颜色表的第 0 项。
    decomposition_axes.imshow(
        plan.labels + 1,
        origin="lower",
        extent=extent,
        cmap=ListedColormap(colors),
        vmin=0,
        vmax=len(plan.cells),
        interpolation="nearest",
    )
    for cell_id in range(len(plan.cells)):
        rows = plan.cells[cell_id]
        middle_row = rows[len(rows) // 2]
        y, left, right = middle_row
        label_x = (left + right + 1) * resolution / 2
        label_y = (y + 0.5) * resolution
        decomposition_axes.text(
            label_x,
            label_y,
            f"C{cell_id + 1}",
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

    # 绘图编码：0 是障碍，1 是尚未访问，2 是已访问。
    base: IntArray = np.zeros(free.shape, dtype=np.int64)
    number_of_free_grids = 0
    for y in range(height):
        for x in range(width):
            if free[y, x]:
                base[y, x] = 1
                number_of_free_grids += 1
    path_axes.imshow(
        base,
        origin="lower",
        extent=extent,
        cmap=ListedColormap([wall, "#f3f6f9"]),
        vmin=0,
        vmax=1,
        interpolation="nearest",
    )

    # 整数坐标代表栅格索引；加 0.5 后位于栅格中心。
    plot_points: list[PlotPoint] = []
    for index in range(len(plan.path)):
        grid_x = int(plan.path[index, 0])
        grid_y = int(plan.path[index, 1])
        plot_points.append(((grid_x + 0.5) * resolution, (grid_y + 0.5) * resolution))

    sweep_segments: list[Segment] = []
    link_segments: list[Segment] = []
    for index in range(1, len(plot_points)):
        segment = [plot_points[index - 1], plot_points[index]]
        if plan.transit[index]:
            link_segments.append(segment)
        else:
            sweep_segments.append(segment)

    path_axes.add_collection(
        LineCollection(sweep_segments, colors=blue, linewidths=1.2, zorder=4)
    )
    path_axes.add_collection(
        LineCollection(
            link_segments,
            colors=orange,
            linewidths=2.2,
            linestyles="dashed",
            zorder=5,
        )
    )
    # 每隔若干条边，在水平覆盖路径上画一个方向箭头。
    for index in range(7, len(plot_points) - 1, 15):
        is_transfer = bool(plan.transit[index + 1])
        is_horizontal = plan.path[index, 1] == plan.path[index + 1, 1]
        if not is_transfer and is_horizontal:
            path_axes.annotate(
                "",
                xy=plot_points[index + 1],
                xytext=plot_points[index],
                arrowprops={
                    "arrowstyle": "-|>",
                    "color": blue,
                    "lw": 1,
                    "mutation_scale": 8,
                },
                zorder=6,
            )

    # 每个元组依次表示：路径索引、标记形状、颜色、文字。
    endpoint_styles = [
        (0, "o", "#13866b", "S"),
        (-1, "s", "#b23d50", "F"),
    ]
    for index, marker, color, label in endpoint_styles:
        position = plot_points[index]
        path_axes.plot(
            [position[0]],
            [position[1]],
            marker=marker,
            color=color,
            markersize=7,
            markeredgecolor="white",
            zorder=7,
        )
        path_axes.annotate(
            label,
            position,
            xytext=(7, 6),
            textcoords="offset points",
            weight="bold",
            fontsize=8,
            color=color,
            zorder=8,
        )

    coverage_image = replay_axes.imshow(
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
    replay_axes.add_collection(sweep_trail)
    replay_axes.add_collection(link_trail)
    robot_lines = replay_axes.plot(
        [],
        [],
        "o",
        color="#c43b50",
        markersize=9,
        markeredgecolor="white",
        markeredgewidth=1.5,
        zorder=8,
    )
    robot = robot_lines[0]

    first_panel_x = decomposition_axes.get_position().x0
    second_panel_x = path_axes.get_position().x0
    third_panel_x = replay_axes.get_position().x0
    figure.text(
        first_panel_x,
        0.205,
        "Each color is one sweep cell",
        color="#526476",
        fontsize=9,
    )
    figure.legend(
        handles=[
            Line2D([], [], color=blue, lw=2, label="Cell sweep"),
            Line2D([], [], color=orange, lw=2, ls="--", label="A* transfer"),
        ],
        loc="lower left",
        bbox_to_anchor=(second_panel_x - 0.004, 0.188),
        ncol=2,
        frameon=False,
        fontsize=9,
        columnspacing=1,
    )
    figure.legend(
        handles=[
            Patch(color=covered, label="Visited"),
            Patch(color=blank, label="Pending"),
            Patch(color=wall, label="Obstacle"),
        ],
        loc="lower left",
        bbox_to_anchor=(third_panel_x - 0.004, 0.188),
        ncol=3,
        frameon=False,
        fontsize=9,
        columnspacing=0.8,
        handlelength=1,
    )
    figure.text(
        first_panel_x,
        0.14,
        f"{len(plan.cells)} cells  /  {number_of_free_grids} free grids",
        fontsize=14,
        weight="bold",
    )
    order_names: list[str] = []
    for cell_id in plan.order:
        order_names.append(f"C{cell_id + 1}")
    visit_order = " > ".join(order_names)
    figure.text(first_panel_x, 0.105, visit_order, fontsize=9, color="#526476")

    total_length = (len(plan.path) - 1) * resolution
    link_length = len(link_segments) * resolution
    figure.text(
        second_panel_x,
        0.14,
        f"{total_length:.1f} m total path",
        fontsize=14,
        weight="bold",
    )
    figure.text(
        second_panel_x,
        0.105,
        f"Sweep {total_length - link_length:.1f} m  +  transfer {link_length:.1f} m",
        fontsize=9,
        color="#526476",
    )
    coverage_text = figure.text(third_panel_x, 0.14, "", fontsize=14, weight="bold")
    status_text = figure.text(third_panel_x, 0.105, "", fontsize=9, color="#526476")
    figure.text(
        0.045,
        0.038,
        f"GRID MODEL  |  pitch = {resolution:g} m"
        "  |  coverage = visited free grids / all free grids"
        "  |  point robot; no turning-radius constraint",
        fontsize=8.5,
        color="#667789",
    )

    # 记录每个栅格第一次被访问的路径索引。
    # 未访问的栅格初始化为路径长度，比所有合法索引都大。
    first_visit: IntArray = np.full(free.shape, len(plan.path), dtype=np.int64)
    for index in range(len(plan.path)):
        x = int(plan.path[index, 0])
        y = int(plan.path[index, 1])
        first_visit[y, x] = min(first_visit[y, x], index)

    def update(frame_index: int) -> tuple[Artist, ...]:
        """重建指定帧；重复播放或回到开头时，不会错误累加覆盖率。"""
        display_grid: IntArray = base.copy()
        visited_count = 0
        for y in range(height):
            for x in range(width):
                if free[y, x] and first_visit[y, x] <= frame_index:
                    display_grid[y, x] = 2
                    visited_count += 1
        coverage_image.set_data(display_grid)

        visible_sweep: list[Segment] = []
        visible_links: list[Segment] = []
        for index in range(1, frame_index + 1):
            segment = [plot_points[index - 1], plot_points[index]]
            if plan.transit[index]:
                visible_links.append(segment)
            else:
                visible_sweep.append(segment)
        sweep_trail.set_segments(visible_sweep)
        link_trail.set_segments(visible_links)

        robot_x, robot_y = plot_points[frame_index]
        robot.set_data([robot_x], [robot_y])
        percentage = 100 * visited_count / number_of_free_grids
        coverage_text.set_text(f"{percentage:5.1f}% grids visited")

        if frame_index == len(plan.path) - 1:
            stage = "COMPLETE"
        elif plan.transit[frame_index]:
            stage = "TRANSFER"
        else:
            stage = "SWEEP"
        grid_x = int(plan.path[frame_index, 0])
        grid_y = int(plan.path[frame_index, 1])
        cell_number = int(plan.labels[grid_y, grid_x]) + 1
        status_text.set_text(
            f"{visited_count}/{number_of_free_grids} grids"
            f"  |  {stage}  |  C{cell_number}"
        )

        # Matplotlib 的 Artist 指图像、线条、文字等可绘制对象。
        return (
            coverage_image,
            sweep_trail,
            link_trail,
            robot,
            coverage_text,
            status_text,
        )

    update(0)
    return figure, update


class Arguments(argparse.Namespace):
    """为命令行参数声明类型，让编辑器知道各个属性的含义。"""

    scene: str = "obstacles"
    resolution: float = 0.4
    fps: int = 20
    stride: int = 4
    gif: Path | None = None
    png: Path | None = None
    no_show: bool = False


def main() -> None:
    """读取参数，规划并检查路径，再按需显示或保存结果。"""
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("--scene", choices=("empty", "obstacles"), default="obstacles")
    parser.add_argument(
        "--resolution", type=float, default=0.4, help="栅格边长，单位 m"
    )
    parser.add_argument("--fps", type=int, default=20, help="动画每秒帧数")
    parser.add_argument("--stride", type=int, default=4, help="每帧前进多少路径点")
    parser.add_argument("--gif", type=Path, help="保存 GIF 到此路径")
    parser.add_argument("--png", type=Path, help="保存三联图到此路径")
    parser.add_argument("--no-show", action="store_true", help="不弹出窗口")
    args = Arguments()
    parser.parse_args(namespace=args)

    if not math.isfinite(args.resolution) or args.resolution <= 0:
        parser.error("resolution 必须是有限正数")
    if args.fps < 1 or args.stride < 1:
        parser.error("fps 和 stride 必须大于零")

    free = make_map(args.scene)
    plan = plan_coverage(free, start=(0, 0))
    stats = validate(free, plan)
    print(f"scene={args.scene}; {stats}")
    path_length = stats["steps"] * args.resolution
    print(f"自由栅格访问率: 100%; 路径长度: {path_length:.1f} m")
    print("检查通过：四邻接连续、未进入障碍、无遗漏自由栅格。")

    if args.no_show and args.gif is None and args.png is None:
        return

    if args.no_show:
        import matplotlib

        matplotlib.use("Agg")

    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation, PillowWriter

    figure, update = make_figure(free, plan, args.resolution)
    final_index = len(plan.path) - 1

    if args.png is not None:
        args.png.parent.mkdir(parents=True, exist_ok=True)
        update(final_index)
        figure.savefig(args.png, dpi=150, facecolor=figure.get_facecolor())
        update(0)
        print(f"PNG: {args.png}")

    if args.gif is not None or not args.no_show:
        # 帧列表允许重复：在开头和末尾各停留一会儿。
        frames: list[int] = []
        opening_frames = max(1, args.fps // 2)
        for _ in range(opening_frames):
            frames.append(0)
        for index in range(0, len(plan.path), args.stride):
            frames.append(index)  # noqa: PERF402
        for _ in range(args.fps):
            frames.append(final_index)

        def initialize_animation() -> tuple[Artist, ...]:
            return update(0)

        # 保留 animation 变量，直到 plt.show() 返回，避免动画提前被回收。
        animation = FuncAnimation(
            figure,
            update,
            frames=frames,
            init_func=initialize_animation,
            interval=1000 / args.fps,
            repeat=True,
            blit=False,
            cache_frame_data=False,
        )
        if args.gif is not None:
            args.gif.parent.mkdir(parents=True, exist_ok=True)
            animation.save(args.gif, writer=PillowWriter(fps=args.fps), dpi=85)
            print(f"GIF: {args.gif}")
        if not args.no_show:
            plt.show()

    plt.close(figure)


if __name__ == "__main__":
    main()
