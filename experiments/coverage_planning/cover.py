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

# =============================================================================
# 一、算法背景：这份代码在解决什么问题
# =============================================================================
#
# 【覆盖路径规划 Coverage Path Planning, CPP】
# 普通路径规划（A*、Dijkstra）解决的是"从 A 点走到 B 点"；覆盖路径规划解决的是
# "走遍整个区域"，典型应用是扫地机器人、割草机、农用机械、水下测绘、除雪车。
# 完整的 CPP 问题等价于区域上的旅行商问题（TSP），是 NP-hard 的，所以实用做法
# 都是"分而治之"的三步走：
#   第 1 步 分解 decomposition：把复杂区域切成若干"形状简单"的子区域；
#   第 2 步 区内覆盖：每个子区域用固定模式（弓形 / 螺旋）走完；
#   第 3 步 区间排序与连接：决定子区域的访问顺序，用普通路径规划把它们串起来。
# 本文件正好对应这三步：decompose -> sweep_cell -> plan_coverage 里的贪心排序。
#
# 【为什么一定要先"分解"】
# 如果不分解，直接在整张地图上做弓形往返，一行扫到障碍物就断开了，断点之后要么
# 漏掉、要么绕路，逻辑会非常混乱。但如果一个子区域满足"任意一条扫描线与它相交
# 都只得到一段连续线段"（这个性质叫**单调性 monotone**），那么"本行从左扫到右，
# 下一行从右扫到左"这条最朴素的规则，就天然保证该区域被完全覆盖且不重复。
# 分解的全部意义，就是人为制造出这个性质。
#
# 【Boustrophedon 分解】
# "boustrophedon" 是希腊语"牛耕式"，指耕牛来回往返的走法。
# 经典的**梯形分解**每遇到一个障碍物顶点就切一刀，切得太碎，导致机器人在子区域
# 之间反复转移。Choset & Pignon (1997) 提出的 Boustrophedon 分解做了关键改进：
# **只在扫描线与自由空间的交集"连通分量数目发生变化"的位置切刀**。这些位置叫
# 临界点 critical point。连通性变化只有四种事件：
#   IN / birth  新出现一段      -> 开一个新单元
#   OUT / death 一段消失了      -> 结束该单元
#   SPLIT       一段裂成多段    -> 结束父单元，为每个子段各开新单元
#   MERGE       多段并成一段    -> 结束所有父单元，开一个新单元
# 结果是单元数量显著减少，但每个单元仍然保持单调性。
# 这套思路后来被 Acar & Choset 推广为 **Morse 分解**：把"直线扫描线"换成任意
# Morse 函数的等值线，于是可以用同心圆、螺旋线等曲线来扫描。
#
# 【本文件的离散化】
# 连续版本要处理多边形几何、切点、浮点容差，很麻烦。这里走的是栅格路线：
#   扫描线             = 栅格的一整行（沿 y 从下往上推进，所以弓形是横向往返的）
#   扫描线与自由空间的交集 = 这一行里连续 True 的区间
#   连通性变化          = 相邻两行的区间之间的"重叠关系"发生变化
# 离散化之后不需要任何几何计算，只剩区间比较，所以 decompose 可以写得很短。
# 代价是精度受栅格分辨率限制，而且分辨率同时兼任"相邻扫描行间距"（作业幅宽）。
#
# 【延伸阅读】
# Choset & Pignon, "Coverage Path Planning: The Boustrophedon Decomposition"
#   (1997) https://publications.ri.cmu.edu/
# Galceran & Carreras, "A survey on coverage path planning for robotics" (2013)
#
#
# =============================================================================
# 二、Python / NumPy 语法速查（本文件中反复出现的写法）
# =============================================================================
#
# 【类型注解 type hint】 `x: int = 3`、`def f(a: int) -> str:`
#   冒号后面是"这个变量应该是什么类型"，箭头后面是"函数返回什么类型"。
#   Python 运行时**完全忽略**它们，不会做任何检查，纯粹是写给人和编辑器看的。
#   删掉所有注解，程序行为一模一样。
#
# 【`from __future__ import annotations`】
#   让所有注解延迟成字符串求值。好处是可以写 `list[int]`、`int | None` 这类
#   新语法而不用担心低版本报错，也不会因为注解里引用了还没定义的类而出错。
#
# 【`int | None`】 读作"要么是 int，要么是 None"，等价于旧写法 Optional[int]。
#
# 【元组 tuple 与列表 list】
#   (1, 2) 是元组，不可修改，可以作字典的键；[1, 2] 是列表，可以 append。
#   本文件用元组表示坐标 (x, y) 这种"位置固定、含义固定"的数据。
#
# 【元组比较】 (3, 1) < (3, 2) 为 True。
#   从左向右逐项比较，第一项分不出胜负才比第二项，和字典序一样。
#   plan_coverage 利用这一点实现"先比距离，平局再比编号"的确定性选择。
#
# 【`range(a, b)`】 生成 a, a+1, ..., b-1，**包含 a 不包含 b**。
#   `range(b, a - 1, -1)` 是倒着数：b, b-1, ..., a。第三个参数是步长。
#
# 【f-string】 f"共 {n} 个" 会把花括号里的表达式求值后插进字符串。
#   f"{x:.1f}" 表示保留 1 位小数，f"{x:5.1f}" 表示总宽 5 位、保留 1 位小数。
#
# 【`__name__ == "__main__"`】
#   直接运行本文件时 __name__ 是 "__main__"，被别的文件 import 时是模块名。
#   所以这句的作用是"只有直接运行才执行 main()"。
#
# --- NumPy 部分 ---
#
# 【ndarray】 NumPy 的多维数组。和 list 的关键区别：元素类型统一（dtype），
#   内存连续，支持整体运算，速度快得多。
#
# 【`shape`】 各维度的长度组成的元组。本文件地图的 shape 是 (20, 30)，
#   即 **20 行 30 列**，第 0 维是 y（行），第 1 维是 x（列）。
#   这就是"Point 用 (x, y) 但数组用 [y, x]"这条约定的来源，**最容易写反的地方**。
#
# 【`dtype`】 元素类型。np.bool_ 是布尔，np.int64 是 64 位整数。
#   显式写出来可以避免平台差异（例如 Windows 上默认整数是 32 位）。
#
# 【`np.ones(shape, dtype)` / `np.zeros` / `np.full(shape, v)`】
#   分别创建全 1 / 全 0 / 全部填充 v 的数组。
#   `np.zeros_like(a)` 创建一个和 a 形状、类型都相同的全 0 数组。
#
# 【切片赋值】 `free[4:10, 7:12] = False`
#   一次性把第 4~9 行、第 7~11 列的矩形区域全设为 False。
#   切片 `a:b` 同样是**含头不含尾**；逗号分隔不同维度。
#   这叫"广播 broadcasting"：右边的单个标量被自动铺满左边整个区域。
#
# 【布尔掩膜 boolean mask】 `labels == cell_id`
#   对整个数组逐元素比较，返回一个同形状的**布尔数组**，
#   元素为 True 的位置就是取值等于 cell_id 的格子。本文件用它把某个子区域
#   单独"抠"出来，当作一张只含该区域的小地图交给 A*。
#
# 【`a.any()` / `a.sum()`】 整个数组是否存在 True / 所有元素求和。
#   对布尔数组求和就是在数 True 的个数（True 当作 1）。
#
# 【`a.ndim`】 维数。二维数组是 2。
#
# 【`np.asarray(list_of_tuples)`】 把 Python 的列表转成 ndarray。
#   一个 N 元素的 (x, y) 元组列表会变成 shape 为 (N, 2) 的二维数组，
#   于是 `arr[:, 0]` 取出所有 x，`arr[:, 1]` 取出所有 y。
#
# 注：本文件为了便于新手逐行跟踪，很多地方故意用 Python 的 for 循环代替了
# NumPy 的向量化写法（例如 validate 里的双层循环）。真实项目中数据量大时，
# 应该改用向量化操作，速度会快几十到几百倍。
# =============================================================================

from __future__ import annotations

# argparse：解析命令行参数（--scene、--gif 之类）。
# heapq："堆"，一种能快速取出最小元素的数据结构，A* 的优先队列靠它实现。
# math：标准数学库，这里只用它判断浮点数是否有限。
# dataclass：装饰器，自动为类生成 __init__ 等样板方法。
# Path：面向对象的文件路径，比手工拼字符串更安全。
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
# 它们的唯一作用是让下面的函数签名更短、更好读；运行时没有任何开销。
Point = tuple[int, int]  # (x, y)
Row = tuple[int, int, int]  # (y, left, right)，包含左右端点
Interval = tuple[int, int]  # (left, right)
ActiveInterval = tuple[int, int, int]  # (left, right, cell_id)
BoolArray = NDArray[np.bool_]  # 布尔数组；维数由具体变量决定
IntArray = NDArray[np.int64]
PlotPoint = tuple[float, float]  # 绘图时的米制坐标
Segment = list[PlotPoint]  # 一条线段，包含起点和终点
# Callable[[int], ...] 表示"一个接收 int、返回 ... 的函数"。
# 函数在 Python 里是"一等公民"，可以像数值一样被传递和返回。
UpdateFunction = Callable[[int], tuple[Artist, ...]]


def make_map(scene: str) -> BoolArray:
    """创建 20 行、30 列的地图；True 是自由空间，False 是障碍。

    想换自己的地图，只要修改这个函数即可，其余代码不需要动。
    唯一的硬性要求：所有 True 的格子必须构成**一个四连通的整体**，
    否则 plan_coverage 会报错（因为一条连续路径走不到孤岛）。
    """
    # np.ones((20, 30)) 创建 20 行 30 列的数组。注意内层的括号：
    # shape 是**一个元组参数**，不是两个参数。
    # dtype=np.bool_ 让元素是 True/False 而不是 1.0/0.0。
    free: BoolArray = np.ones((20, 30), dtype=np.bool_)

    if scene == "obstacles":
        # 切片包含起点，不包含终点。例如 4:10 表示索引 4 到 9。
        # 逗号前是行（y）范围，逗号后是列（x）范围，和 Point 的 (x, y) 顺序相反。
        # 赋值右边只是一个 False，NumPy 会自动把它"广播"到整个矩形区域。
        free[4:10, 7:12] = False
        free[12:17, 19:25] = False

    return free


def row_intervals(row: BoolArray) -> list[Interval]:
    """把一行里连续的 True 切成若干区间。

    例如 [True, True, False, True] 对应 [(0, 1), (3, 3)]。

    返回的是**闭区间** (left, right)，两端都包含。这个开闭约定很重要：
    decompose 里的重叠判据、sweep_cell 里的 range 边界都依赖它，
    改成半开区间就必须连带改那些地方，否则出现差一（off-by-one）错误。
    """
    intervals: list[Interval] = []
    # left 是"当前正在累积的区间的起点"。
    # 用 None 表示"现在不在任何区间里"，比用 -1 当哨兵更不容易误读。
    left: int | None = None

    for x in range(len(row)):
        if row[x]:
            # 进入自由格：如果之前不在区间里，说明这里是一个新区间的左端。
            if left is None:
                left = x
        else:
            # 遇到障碍：如果之前在区间里，说明前一格 x-1 是该区间的右端。
            if left is not None:
                intervals.append((left, x - 1))
                left = None

    # 扫描到行末时，最后一个自由区间可能还没有结束。
    # 这种"循环结束后的收尾处理"是扫描类代码最常见的遗漏点。
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

    核心思想（对应文件头部的 Boustrophedon 背景）：
    从下往上一行一行看，**只在相邻两行的连通关系发生变化时才开新区域**。
    变化分四种：新生、消亡、分裂、合并。其余情况（一对一）延续原区域。

    为什么不能只比较"区间数量"？
    考虑两行    下行 `...###`  上行 `###...`：数量都是 1，但它们左右错开、完全不相邻。
    只数数量会把它们当成同一个区域，结果是一个根本走不通的"区域"。
    所以必须比较具体的**连接关系**，也就是下面的 parents / child_count。
    """
    # 创建与地图同样大小的数组，初始值全部为 -1
    # free.shape 是个元组，直接传给 np.full 就能得到同形状的数组。
    # -1 充当"障碍 / 尚未赋区域"的哨兵值；真实编号从 0 开始。
    labels: IntArray = np.full(free.shape, -1, dtype=np.int64)
    # cells[cell_id] 保存某个子区域的所有扫描区间
    # 每个扫描区间表示为 (y, left, right)
    # 这是一个"列表的列表"：外层下标是区域编号，内层是该区域的所有行。
    cells: list[list[Row]] = []
    # previous 是整个扫描线算法的**全部记忆**：只记住上一行。
    # 所以额外空间是 O(宽度)，地图再高也不会爆内存；
    # 代价是算法**无法回头修改**已经做过的分区决定。
    previous: list[ActiveInterval] = []

    # free.shape[0] 是行数（y 方向）。从 y=0 开始，配合绘图的 origin="lower"，
    # 视觉上就是从地图底部往上扫。
    for y in range(free.shape[0]):
        # free[y] 取出第 y 行，得到一个长度为宽度的一维布尔数组。
        row = free[y]
        intervals = row_intervals(row)

        # parents[i]：当前区间 i 连接了上一行的哪些区间。
        # 保存的是 previous 的索引，不是子区域编号。
        #
        # 为什么存索引而不存 cell_id？因为下面的 child_count 要统计的是
        # "上一行有几个**区间**认我作父"，用索引表达最直接；
        # 而且不依赖"同一行不会有两段共享 cell_id"这条不变量。
        parents: list[list[int]] = []
        for left, right in intervals:
            parent_indices: list[int] = []
            for j in range(len(previous)):
                previous_left = previous[j][0]
                previous_right = previous[j][1]
                # 两个闭区间 [left,right] 与 [pl,pr] 的交集还是一个闭区间：
                #   左端取两个左端的较大者（谁起步晚听谁的）
                #   右端取两个右端的较小者（谁结束早听谁的）
                # 交集非空等价于 左端 <= 右端。等价写法：left <= pr and pl <= right。
                overlap_left = max(left, previous_left)
                overlap_right = min(right, previous_right)
                # 这里必须是 <= 而不是 <：区间是闭的，"只共享一列"也算相邻。
                #
                # 为什么列方向重叠就等于四邻接连通？
                # 四邻接下，从上一行走到这一行必须是"正上方/正下方"的一步，
                # 也就是两格的**列号必须相同**。所以"存在可通行的边"等价于
                # "存在某个列号同时落在两段里"，恰好就是区间交集非空。
                # 斜着挻着（上行到第 6 列止，本行从第 7 列起）在八邻接下算连通，
                # 四邻接下不通，这个公式正确地判为不重叠。
                if overlap_left <= overlap_right:
                    parent_indices.append(j)
            parents.append(parent_indices)

        # child_count[j]：上一行区间 j 有多少个子区间。
        # 这是把 parents 表示的二部图**反着数了一遍**，不需要重做重叠判定。
        # [0] * n 生成 n 个 0 的列表，是计数数组的常见初始化写法。
        child_count: list[int] = [0] * len(previous)
        for parent_indices in parents:
            for parent_index in parent_indices:
                child_count[parent_index] += 1

        current: list[ActiveInterval] = []
        for i in range(len(intervals)):
            # 元组解包：把 (left, right) 一次性拆成两个变量。
            left, right = intervals[i]
            parent_indices = parents[i]

            # None 表示还没有找到可以继承的区域编号。
            # 这样避免用另一个布尔变量间接判断 parent_index 是否已赋值。
            #
            # 四种事件在这里汇合：
            #   len(parent_indices) == 0            -> 新生，新建
            #   len == 1 且 child_count == 1       -> 一对一，延续
            #   len == 1 且 child_count > 1        -> 父亲分裂，新建
            #   len >= 2                            -> 多个父亲合并，新建
            # 注意"分裂"是唯一光看 parents 看不出来的：站在子段的视角，它只有一个父亲，
            # 和一对一长得一模一样；必须问一句"我父亲还有别的孩子吗"才能区分。
            # 这就是 child_count 存在的全部理由。
            inherited_cell_id: int | None = None
            if len(parent_indices) == 1:
                parent_index = parent_indices[0]
                if child_count[parent_index] == 1:
                    # previous 的元素是 (left, right, cell_id)，下标 2 就是 cell_id。
                    inherited_cell_id = previous[parent_index][2]

            if inherited_cell_id is None:
                # 新建区域：编号就是当前已有区域数，然后追加一个空列表占位。
                # 所以编号按**创建顺序**分配，并不代表访问顺序。
                cell_id = len(cells)
                cells.append([])
            else:
                cell_id = inherited_cell_id

            # 把这一段的每一格都涂上区域编号。
            # （向量化写法是 labels[y, left:right + 1] = cell_id，这里展开成循环以便跟踪。）
            for x in range(left, right + 1):
                labels[y, x] = cell_id
            cells[cell_id].append((y, left, right))
            current.append((left, right, cell_id))

        # 扫描线向上推进一行：本行变成下一轮的"上一行"。
        previous = current

    return labels, cells


def astar(free: BoolArray, start: Point, goal: Point) -> list[Point] | None:
    """用四邻接 A* 连接两点；返回的路径包含起点和终点。

    A* = Dijkstra + 启发函数。它维护两个量：
        g：从起点走到当前点的**实际**代价
        h：从当前点到终点的**估计**代价（启发值）
    每次优先扩展 f = g + h 最小的点。只要 h 永远不高估真实代价（叫"可采纳"
    admissible），A* 就保证找到最短路径。四邻接网格上曼哈顿距离正好满足这个条件。

    在本文件里它被用在两个地方：
        1. sweep_cell：在**单个子区域内部**连接两条扫描行的端点；
        2. plan_coverage：在**整张地图上**从当前位置转移到下一个区域的入口。
    两者的区别只是传进来的 free 不同（区域掩膜 vs 全图）。

    找不到路径时返回 None，而不是抛异常，方便调用方用 if 分支处理。
    """
    # 元组解包：shape 是 (行数, 列数)，对应 (height, width)。
    height, width = free.shape

    # 先检查端点，避免负索引或障碍物起点被误认为合法。
    # （Python 的负索引是合法的：free[-1, -1] 会取到右上角而不是报错，
    #   这是栅格代码里非常难查的一类 bug，所以这里显式拦一道。）
    for x, y in (start, goal):
        if x < 0 or x >= width or y < 0 or y >= height:
            return None
        if not free[y, x]:
            return None

    # 定义在函数内部的函数叫"闭包"，它可以直接读取外层的 goal，
    # 不用把 goal 当参数到处传。
    def distance_to_goal(point: Point) -> int:
        """曼哈顿距离：横向距离加纵向距离。"""
        horizontal_distance = abs(point[0] - goal[0])
        vertical_distance = abs(point[1] - goal[1])
        return horizontal_distance + vertical_distance

    # 优先队列元素：(估计总代价 f，已走代价 g，坐标)。
    # heapq 每次弹出最小项；f 相同时再按后续字段比较。
    #
    # heapq 是"在普通 list 上维护堆性质"的函数集，没有单独的堆类型：
    #   heappush(lst, item) 入队，heappop(lst) 取出并删除最小项，两者都是 O(log n)。
    queue: list[tuple[int, int, Point]] = []
    heapq.heappush(queue, (distance_to_goal(start), 0, start))
    # cost[p]：目前已知的从 start 走到 p 的最小代价。
    # 用字典而不是二维数组，是因为只有被访问过的点才需要记录。
    cost: dict[Point, int] = {start: 0}
    # parent[p]：走到 p 的前一个点，用于最后回溯出整条路径。
    parent: dict[Point, Point] = {}
    # 四个方向：右、上、左、下。没有斜向，这就是"四邻接"。
    directions: list[Point] = [(1, 0), (0, 1), (-1, 0), (0, -1)]

    while len(queue) > 0:
        entry = heapq.heappop(queue)
        current_cost = entry[1]
        point = entry[2]

        # 同一个点可能先后以不同代价入队；跳过已经过期的记录。
        # （heapq 不支持"降低已在队元素的优先级"，所以通行做法是重复入队
        #   + 弹出时校验，这种写法叫"懒删除 lazy deletion"。）
        if current_cost != cost[point]:
            continue

        if point == goal:
            path: list[Point] = [point]
            while point != start:
                point = parent[point]
                path.append(point)
            # 上面从终点回溯到起点，所以需要把列表反转。
            # list.reverse() 是**原地**反转并返回 None；想要新列表要用 path[::-1]。
            path.reverse()
            return path

        x, y = point
        for dx, dy in directions:
            next_x = x + dx
            next_y = y + dy
            # 依次排除：越界、越界、撞障碍。continue 跳过本次循环。
            if next_x < 0 or next_x >= width:
                continue
            if next_y < 0 or next_y >= height:
                continue
            if not free[next_y, next_x]:
                continue

            next_point: Point = (next_x, next_y)
            # 每走一格代价固定为 1，所以路径长度就是步数。
            next_cost = current_cost + 1
            # dict.get(k) 在键不存在时返回 None，不会像 d[k] 那样抛 KeyError。
            old_cost = cost.get(next_point)
            # 既有记录更好或持平，就没必要再入队。
            if old_cost is not None and next_cost >= old_cost:
                continue

            cost[next_point] = next_cost
            parent[next_point] = point
            estimated_total = next_cost + distance_to_goal(next_point)
            heapq.heappush(queue, (estimated_total, next_cost, next_point))

    # 队列空了还没到终点，说明两点不连通。
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

    两个布尔参数的四种组合 = 四个不同的"入口角"（左下/右下/左上/右上）。
    plan_coverage 会把四种都生成出来，然后挑转移距离最短的那个。

    mask 是只包含本区域的布尔地图（由 labels == cell_id 得到），
    这样区内 A* 不可能跑到别的区域里去绕路。
    """
    # .copy() 必不可少：直接 reverse() 会改写调用方传进来的 cells[cell_id]，
    # 而同一个 rows 还要被另外三种组合复用。这是 Python 里最典型的可变对象陷阱。
    ordered_rows = rows.copy()
    if top_down:
        ordered_rows.reverse()

    path: list[Point] = []
    right_to_left = start_right

    for y, left, right in ordered_rows:
        # range 的第三个参数是步长。倒着走时终点写 left - 1，
        # 因为 range 不包含终点，写 left 会漏掉最左边一格。
        if right_to_left:
            x_values = range(right, left - 1, -1)
        else:
            x_values = range(left, right + 1)

        # strip 是本行要走的完整格子序列。
        strip: list[Point] = []
        for x in x_values:
            strip.append((x, y))

        if len(path) == 0:
            path.extend(strip)
        else:
            # 为什么需要 A*？因为相邻两行的左右端点可能不对齐。
            # 例如从 (0,6) 过渡到 (0,29)，扫完窄行右端后需要横向挪一段才到宽行起点。
            connector = astar(mask, path[-1], strip[0])
            if connector is None:
                # 正常情况下不可能触发：decompose 保证每个区域内部四连通。
                # 这是一道防御性断言，如果有人改坏了分区逻辑会在这里暴露。
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
    """dataclass 自动生成初始化方法，用来集中保存规划结果。

    写 @dataclass 后，Python 会根据下面的字段声明自动生成 __init__、__repr__ 等，
    省去手写 self.labels = labels 这类样板代码。
    比起返回一个多元素元组，用 dataclass 的好处是访问时有名字（plan.path）。
    """

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
    """生成每个区域的四种覆盖路径，然后贪心选择最近的入口。

    这是"分而治之三步走"的第三步：区间排序与连接。
    用的是**最近邻启发**：每次只看"到下一个区域入口的 A* 距离最短"。
    它快但不最优；严格做法是把每个单元当节点、转移代价当边权，解一个 TSP。
    """
    # .ndim 是维数，.any() 是"是否存在 True"。
    # 这两句把"传错形状"和"全是障碍"两种调用错误挡在门外。
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
    # 提前把所有候选算好，后面的贪心循环就只需要比较而不用重复生成。
    candidates: dict[int, list[list[Point]]] = {}
    for cell_id in range(len(cells)):
        rows = cells[cell_id]
        # 布尔掩膜：逐元素比较，得到一张只有本区域为 True 的小地图。
        mask: BoolArray = labels == cell_id
        routes: list[list[Point]] = []
        # 两层循环组合出四种入口方向。顺序固定，所以 routes 的下标含义稳定。
        for top_down in (False, True):
            for start_right in (False, True):
                route = sweep_cell(rows, mask, top_down, start_right)
                routes.append(route)
        candidates[cell_id] = routes

    # 路径从起点开始。transit[0] 填 False 只是占位：
    # 第 0 个点没有"到达它的边"，这一项在统计时会被跳过。
    path: list[Point] = [start]
    transit: list[bool] = [False]
    order: list[int] = []

    # 每轮选走一个区域并从 candidates 删除，直到全部覆盖完。
    while len(candidates) > 0:
        best: RouteChoice | None = None
        current_position = path[-1]  # 负索引 -1 表示最后一个元素

        for cell_id in candidates:  # noqa: PLC0206
            routes = candidates[cell_id]
            for route_index in range(len(routes)):
                route = routes[route_index]
                # route[0] 是这条候选路径的入口格。
                link = astar(free, current_position, route[0])
                if link is None:
                    continue

                # 元组从左向右比较：先比连接长度，再比区域和候选编号。
                # 后两项用于平局时固定选择，使结果可复现。
                # （否则字典遍历顺序或浮点误差一变，输出就变了，很难调试。）
                key = (len(link), cell_id, route_index)
                if best is None or key < best.key:
                    best = RouteChoice(key, cell_id, route, link)

        if best is None:
            raise ValueError("存在起点无法到达的自由区域；无法用一条路径覆盖。")

        # 两段都从下标 1 开始，因为第 0 个点已经在 path 末尾了。
        # transit 标记这一步是"转移"（True，绘图时画橙色虚线）还是"覆盖"（False）。
        for index in range(1, len(best.link)):
            path.append(best.link[index])
            transit.append(True)
        for index in range(1, len(best.route)):
            path.append(best.route[index])
            transit.append(False)

        order.append(best.cell_id)
        # del 从字典里移除一个键，保证每个区域只被选一次。
        del candidates[best.cell_id]

    return Plan(
        labels=labels,
        cells=cells,
        # 把 N 个 (x, y) 元组转成 shape=(N, 2) 的数组，
        # 之后就能用 plan.path[k, 0] / [k, 1] 或整列切片高效访问。
        path=np.asarray(path, dtype=np.int64),
        transit=np.asarray(transit, dtype=np.bool_),
        order=order,
    )


def validate(free: BoolArray, plan: Plan) -> dict[str, int]:
    """逐点检查：路径合法、四邻接连续，并且访问所有自由栅格。

    这个函数很重要：它检查的是**实际走出来的路径**，而不是"规划时以为覆盖了多少"。
    覆盖规划的 bug 几乎全部会在这三条断言之一上暴露。
    """
    if plan.path.ndim != 2 or plan.path.shape[1] != 2:
        raise AssertionError("路径必须是 N 行、2 列的坐标数组。")
    if len(plan.path) == 0:
        raise AssertionError("路径不能为空。")
    if plan.transit.ndim != 1 or len(plan.transit) != len(plan.path):
        raise AssertionError("transit 必须是一维数组，并且与路径点数量相同。")

    height, width = free.shape
    # np.zeros_like(free)：生成一个和 free 形状、dtype 都相同的全 False 数组。
    visited: BoolArray = np.zeros_like(free)
    for index in range(len(plan.path)):
        # int(...) 把 NumPy 整数转成 Python 整数，避免后续比较和格式化时类型意外。
        x = int(plan.path[index, 0])
        y = int(plan.path[index, 1])
        if x < 0 or x >= width or y < 0 or y >= height:
            raise AssertionError("路径超出了地图范围。")
        if not free[y, x]:
            raise AssertionError("路径进入障碍物。")

        if index > 0:
            previous_x = int(plan.path[index - 1, 0])
            previous_y = int(plan.path[index - 1, 1])
            # 曼哈顿距离恰好为 1，等价于"这一步是合法的四邻接移动"。
            # 为 0 说明原地踏步，大于 1 说明路径断开或斜着跳了。
            step_distance = abs(x - previous_x) + abs(y - previous_y)
            if step_distance != 1:
                raise AssertionError("路径存在跳跃或重复相邻点。")

        visited[y, x] = True

    # 逐格对比：每个自由格都必须被访问过。
    # （向量化写法是 np.array_equal(visited, free)，这里展开以便看清逻辑。）
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

    # 从 1 开始：transit[0] 是占位项，不对应任何一步。
    transit_steps = 0
    for index in range(1, len(plan.transit)):
        if plan.transit[index]:
            transit_steps += 1

    # 返回字典而不是元组，这样 print 出来自带字段名，也方便以后增删指标。
    # revisits：路径点数减去不同格子数，即重复踩过的次数。
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
    """绘制分区、完整路径和覆盖回放，并返回更新某一帧的函数。

    返回两个东西：画好的 Figure，以及一个 update(k) 函数。
    调用 update(k) 就把画面切换到"机器人走到第 k 个路径点"的状态。
    main 里的 FuncAnimation 就是反复调用它来生成动画的。

    图内文字全部用英文，是为了避免依赖系统中文字体（缺字体时会显示成豆腐块）。
    """
    # pyplot 在 main 选择后端之后才导入，以支持 --no-show。
    # matplotlib 的"后端 backend"决定图画到哪里（弹窗 / 文件），
    # 一旦 pyplot 被导入就难以再改，所以这里用了"函数内导入"。
    import matplotlib.pyplot as plt
    from matplotlib.axes import Axes
    from matplotlib.collections import LineCollection
    from matplotlib.colors import ListedColormap
    from matplotlib.lines import Line2D
    from matplotlib.patches import Patch

    # 颜色用 "#RRGGBB" 十六进制字符串表示，每两位分别是红、绿、蓝。
    ink = "#183047"
    blue = "#2563b8"
    orange = "#d97706"
    wall = "#334155"
    blank = "#edf1f5"
    covered = "#a7dfc6"
    # 调色板循环使用：区域数超过 8 个时会从头开始重复。
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
    # rcParams 是 matplotlib 的全局样式字典，在这里统一设字体和颜色。
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
    # Figure 是整张画布，Axes 是其中一个坐标系（子图）。
    # add_subplot(1, 3, n) 意思是"1 行 3 列网格中的第 n 个"，n 从 1 开始。
    figure = plt.figure(figsize=(14.8, 6.2), facecolor="#fafbfc")
    axes: list[Axes] = []
    for subplot_number in range(1, 4):
        axes.append(figure.add_subplot(1, 3, subplot_number))
    decomposition_axes = axes[0]
    path_axes = axes[1]
    replay_axes = axes[2]

    # 这些 0~1 的数字是"图形坐标"：0 是画布左/下边，1 是右/上边。
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
    # extent 把"栅格下标"换算成"米"：(x左, x右, y下, y上)。
    # resolution 是一个栅格的边长（米）。
    extent = (0.0, width * resolution, 0.0, height * resolution)
    titles = [
        "1 / Sweep-line decomposition",
        "2 / Planned motion",
        "3 / Coverage replay",
    ]
    # 栅格线位置：N 个格子需要 N+1 条线，所以 range 到 width + 1。
    x_ticks: list[float] = []
    y_ticks: list[float] = []
    for x in range(width + 1):
        x_ticks.append(x * resolution)
    for y in range(height + 1):
        y_ticks.append(y * resolution)

    # 三幅子图的公共样式一次性设好。
    for index in range(len(axes)):
        axis = axes[index]
        axis.set_title(titles[index], loc="left", fontsize=12, weight="bold", pad=14)
        axis.set_xlim(extent[0], extent[1])
        axis.set_ylim(extent[2], extent[3])
        # "equal" 保证栅格是正方形而不是被拉长的矩形。
        axis.set_aspect("equal")
        axis.set_xlabel("x [m]", fontsize=9)
        axis.set_ylabel("y [m]", fontsize=9, labelpad=2)
        axis.tick_params(labelsize=8, length=3)
        # spines 是子图的四条边框。
        for spine in axis.spines.values():
            spine.set_color("#b9c4cf")
        # minor tick（次要刻度）在每个栅格边界上，用来画白色网格线；
        # 刻度线本身长度设为 0，只留网格。
        axis.set_xticks(x_ticks, minor=True)
        axis.set_yticks(y_ticks, minor=True)
        axis.grid(which="minor", color="white", linewidth=0.35, alpha=0.45)
        axis.tick_params(which="minor", length=0)

    # 颜色表的第 0 项留给障碍，之后每个区域一个颜色。
    colors: list[str] = [wall]
    for cell_id in range(len(plan.cells)):
        # % 是取余，让下标在 0..len(palette)-1 之间循环。
        palette_index = cell_id % len(palette)
        colors.append(palette[palette_index])

    # labels 中障碍为 -1，加 1 后，障碍对应颜色表的第 0 项。
    # plan.labels + 1 是 NumPy 的整体运算：每个元素都加 1，生成一个新数组。
    # origin="lower" 让 y=0 画在下方，符合数学习惯（图像默认是 y=0 在上）。
    # interpolation="nearest" 禁用平滑，让栅格边界保持锐利。
    decomposition_axes.imshow(
        plan.labels + 1,
        origin="lower",
        extent=extent,
        cmap=ListedColormap(colors),
        vmin=0,
        vmax=len(plan.cells),
        interpolation="nearest",
    )
    # 在每个区域中间的一条扫描行上打标签，保证文字落在实际自由区域内。
    for cell_id in range(len(plan.cells)):
        rows = plan.cells[cell_id]
        # // 是整除，取中间那一行。
        middle_row = rows[len(rows) // 2]
        y, left, right = middle_row
        label_x = (left + right + 1) * resolution / 2
        label_y = (y + 0.5) * resolution
        decomposition_axes.text(
            label_x,
            label_y,
            # 对外显示从 1 开始编号（C1、C2...），内部从 0 开始。
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
    # 把三种状态编成整数，配合 ListedColormap 就能用一张图表达全部信息。
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
    # 不加 0.5 的话，路径线会画在栅格的左下角而不是正中间。
    plot_points: list[PlotPoint] = []
    for index in range(len(plan.path)):
        grid_x = int(plan.path[index, 0])
        grid_y = int(plan.path[index, 1])
        plot_points.append(((grid_x + 0.5) * resolution, (grid_y + 0.5) * resolution))

    # 把每一步分成两类线段：区内覆盖（蓝实线）和区间转移（橙虚线）。
    sweep_segments: list[Segment] = []
    link_segments: list[Segment] = []
    for index in range(1, len(plot_points)):
        segment = [plot_points[index - 1], plot_points[index]]
        if plan.transit[index]:
            link_segments.append(segment)
        else:
            sweep_segments.append(segment)

    # LineCollection 把成千上万条线段当作**一个**绘图对象，
    # 比循环调用 plot() 快很多。zorder 越大画得越靠上。
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
    # range(7, n, 15)：从第 7 条边开始，每隔 15 条画一个，避免箭头过密。
    for index in range(7, len(plot_points) - 1, 15):
        is_transfer = bool(plan.transit[index + 1])
        # y 坐标相同即这一步是横向的；只在横向段画箭头，视觉上最清楚。
        is_horizontal = plan.path[index, 1] == plan.path[index + 1, 1]
        if not is_transfer and is_horizontal:
            # annotate 的第一个参数是文字，这里传空串，只要箭头。
            # xytext -> xy 是箭头的方向。
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
    # 索引 0 是起点 Start，-1 是终点 Finish。
    endpoint_styles = [
        (0, "o", "#13866b", "S"),
        (-1, "s", "#b23d50", "F"),
    ]
    for index, marker, color, label in endpoint_styles:
        position = plot_points[index]
        # plot 期望的是序列，所以单个点也要包成列表 [x]、[y]。
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
            # offset points：相对于锚点偏移多少个"点"（字号单位），避免文字压在标记上。
            xytext=(7, 6),
            textcoords="offset points",
            weight="bold",
            fontsize=8,
            color=color,
            zorder=8,
        )

    # 第三幅图是动画回放，这里创建的对象都会在 update() 里被反复修改。
    # 注意这里把返回值存下来了（coverage_image 等），而前两幅没有存。
    coverage_image = replay_axes.imshow(
        base,
        origin="lower",
        extent=extent,
        cmap=ListedColormap([wall, blank, covered]),
        vmin=0,
        vmax=2,
        interpolation="nearest",
    )
    # 先用空列表创建，后面用 set_segments() 填入当前帧该显示的部分。
    sweep_trail = LineCollection([], colors=blue, linewidths=1.15, zorder=4)
    link_trail = LineCollection([], colors=orange, linewidths=2, zorder=5)
    replay_axes.add_collection(sweep_trail)
    replay_axes.add_collection(link_trail)
    # plot() 返回的是**列表**（可以一次画多条线），所以要取 [0]。
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

    # get_position().x0 取子图左边界在画布上的位置，
    # 用它把下方的文字和图例对齐到对应子图。
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
    # Line2D([], []) 创建一条没有数据的线，只用做图例里的色样。
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
    # Patch 是色块图例，对应 imshow 里的三种颜色。
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
    # " > ".join(list) 把字符串列表用分隔符拼成一个字符串。
    order_names: list[str] = []
    for cell_id in plan.order:
        order_names.append(f"C{cell_id + 1}")
    visit_order = " > ".join(order_names)
    figure.text(first_panel_x, 0.105, visit_order, fontsize=9, color="#526476")

    # 步数 = 路径点数 - 1（n 个点之间有 n-1 段），再乘栅格边长得到米数。
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
    # 这两个文字对象先留空，每帧由 update() 写入实时数值。
    coverage_text = figure.text(third_panel_x, 0.14, "", fontsize=14, weight="bold")
    status_text = figure.text(third_panel_x, 0.105, "", fontsize=9, color="#526476")
    # 相邻的字符串字面量会被 Python 自动拼接，不需要加号。
    # {resolution:g} 的 g 格式会自动去掉多余的 0（0.40 -> 0.4）。
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
    #
    # 为什么要这个表？因为动画会**重播**。如果用"每帧累加计数"的写法，
    # 第二遍播放时覆盖率会累计到超过 100%。有了首次访问时间，
    # 任意帧 k 的状态都能由 first_visit <= k 独立重建，与播放历史无关。
    first_visit: IntArray = np.full(free.shape, len(plan.path), dtype=np.int64)
    for index in range(len(plan.path)):
        x = int(plan.path[index, 0])
        y = int(plan.path[index, 1])
        # 重复踩过的格子只保留最小（最早）的索引。
        first_visit[y, x] = min(first_visit[y, x], index)

    # 内部函数（闭包）：它能直接读取外层的 base、plot_points、coverage_image 等，
    # 所以调用时只需要传一个帧号。这正是 FuncAnimation 期望的接口形式。
    def update(frame_index: int) -> tuple[Artist, ...]:
        """重建指定帧；重复播放或回到开头时，不会错误累加覆盖率。"""
        # .copy() 必须有：直接改 base 会污染下一帧的起始状态。
        display_grid: IntArray = base.copy()
        visited_count = 0
        for y in range(height):
            for x in range(width):
                if free[y, x] and first_visit[y, x] <= frame_index:
                    display_grid[y, x] = 2
                    visited_count += 1
        coverage_image.set_data(display_grid)

        # 只显示到当前帧为止的轨迹。
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

        # 三种状态：走完了 / 正在转移 / 正在覆盖。
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
        # 返回被修改过的 Artist 是 FuncAnimation 的约定（blit 模式下会用到）。
        return (
            coverage_image,
            sweep_trail,
            link_trail,
            robot,
            coverage_text,
            status_text,
        )

    # 先画出第 0 帧，这样即使不做动画（只存 PNG）画面也是完整的。
    update(0)
    # 把函数本身作为返回值交给调用方——Python 里函数是普通对象。
    return figure, update


class Arguments(argparse.Namespace):
    """为命令行参数声明类型，让编辑器知道各个属性的含义。

    argparse 默认返回一个万能的 Namespace，编辑器无法知道 args.fps 是什么类型。
    继承一个带字段声明的子类并传给 parse_args(namespace=...)，
    就能拿到补全和类型检查。这里的默认值会被命令行解析结果覆盖。
    """

    scene: str = "obstacles"
    resolution: float = 0.4
    fps: int = 20
    stride: int = 4
    gif: Path | None = None
    png: Path | None = None
    no_show: bool = False


def main() -> None:
    """读取参数，规划并检查路径，再按需显示或保存结果。"""
    # description=__doc__ 把文件头部的文档字符串当作 --help 的说明。
    # RawDescriptionHelpFormatter 保留其中的换行，不被自动重排。
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    # choices 限定取值范围，传别的值 argparse 会自动报错。
    parser.add_argument("--scene", choices=("empty", "obstacles"), default="obstacles")
    # type=float 让 argparse 自动把字符串转成浮点数。
    parser.add_argument(
        "--resolution", type=float, default=0.4, help="栅格边长，单位 m"
    )
    parser.add_argument("--fps", type=int, default=20, help="动画每秒帧数")
    parser.add_argument("--stride", type=int, default=4, help="每帧前进多少路径点")
    # type=Path 直接得到 Path 对象；不传该参数时值是 None。
    parser.add_argument("--gif", type=Path, help="保存 GIF 到此路径")
    parser.add_argument("--png", type=Path, help="保存三联图到此路径")
    # action="store_true" 表示开关型参数：写了就是 True，不写就是 False。
    # 命令行的 --no-show 在代码里自动变成 args.no_show（连字符改成下划线）。
    parser.add_argument("--no-show", action="store_true", help="不弹出窗口")
    args = Arguments()
    parser.parse_args(namespace=args)

    # 浮点数可能是 nan 或 inf，这两者都不能当分辨率；
    # 注意 nan 跟任何数比较都是 False，所以必须用 isfinite 单独判。
    if not math.isfinite(args.resolution) or args.resolution <= 0:
        parser.error("resolution 必须是有限正数")
    if args.fps < 1 or args.stride < 1:
        parser.error("fps 和 stride 必须大于零")

    # 主流程：造地图 -> 规划 -> 验证 -> 输出。
    free = make_map(args.scene)
    plan = plan_coverage(free, start=(0, 0))
    stats = validate(free, plan)
    print(f"scene={args.scene}; {stats}")
    path_length = stats["steps"] * args.resolution
    print(f"自由栅格访问率: 100%; 路径长度: {path_length:.1f} m")
    print("检查通过：四邻接连续、未进入障碍、无遗漏自由栅格。")

    # 既不显示也不保存，就没必要加载 matplotlib，直接返回。
    if args.no_show and args.gif is None and args.png is None:
        return

    # "Agg" 是纯文件后端，不需要图形界面，适合服务器。
    # 必须在 import pyplot **之前**调用 matplotlib.use()。
    if args.no_show:
        import matplotlib

        matplotlib.use("Agg")

    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation, PillowWriter

    figure, update = make_figure(free, plan, args.resolution)
    final_index = len(plan.path) - 1

    if args.png is not None:
        # parents=True 递归创建父目录；exist_ok=True 让目录已存在时不报错。
        args.png.parent.mkdir(parents=True, exist_ok=True)
        # 先跳到最后一帧截图（展示完整覆盖），再跳回第 0 帧供动画使用。
        update(final_index)
        figure.savefig(args.png, dpi=150, facecolor=figure.get_facecolor())
        update(0)
        print(f"PNG: {args.png}")

    if args.gif is not None or not args.no_show:
        # 帧列表允许重复：在开头和末尾各停留一会儿。
        # frames 里的每个元素会被当作参数传给 update()。
        # stride 只影响动画快慢，不影响规划结果和覆盖统计。
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
        # 这是 matplotlib 动画最常见的坑：如果不用变量接住，
        # Python 的垃圾回收会把它回收掉，画面就一动不动。
        animation = FuncAnimation(
            figure,
            update,
            frames=frames,
            init_func=initialize_animation,
            interval=1000 / args.fps,  # 帧间隔，单位是毫秒
            repeat=True,
            blit=False,
            cache_frame_data=False,
        )
        if args.gif is not None:
            args.gif.parent.mkdir(parents=True, exist_ok=True)
            # PillowWriter 靠 pillow 库写 GIF，不需要额外安装 ffmpeg。
            animation.save(args.gif, writer=PillowWriter(fps=args.fps), dpi=85)
            print(f"GIF: {args.gif}")
        if not args.no_show:
            # show() 会阻塞，直到用户关掉窗口。
            plt.show()

    # 释放图形占用的内存。批量生成图片时不关会造成内存持续增长。
    plt.close(figure)


# 直接运行本文件时 __name__ 是 "__main__"，被其他文件 import 时是模块名。
# 所以这段保证：当作模块导入时只拿到函数定义，不会意外触发整个演示。
if __name__ == "__main__":
    main()
