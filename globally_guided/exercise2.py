"""阶段 2 的练习文件：Visibility-PRM 建图 + 拓扑类枚举（论文 Algorithm 1）。

只需实现下面八个函数，不要改其他文件。推荐按声明顺序写，每一个都有独立的
--stage 可以验证：

    python globally_guided/check2.py --stage edge        # edge_feasible
    python globally_guided/check2.py --stage sample      # sample_state
    python globally_guided/check2.py --stage visible     # visible_guards
    python globally_guided/check2.py --stage add         # try_add_sample
    python globally_guided/check2.py --stage dfs         # enumerate_paths
    python globally_guided/check2.py --stage invariant   # build_prm 的图不变量
    python globally_guided/check2.py --stage topology    # 端到端的拓扑类数
    python globally_guided/check2.py                     # 全跑

看不出问题在哪的时候，先看统计量、再看图：
    python globally_guided/viz2.py --scenario single_static

=====================================================================
先读这段：阶段 2 和阶段 1 的难点不是同一类
=====================================================================

阶段 1 的三个函数都是**纯谓词**：没有状态、没有历史，错了就是数学错了，
而且一测就现形（要么抛异常、要么用例直接红）。

阶段 2 你在**增量维护一个数据结构**。这类 bug 的典型表现是：
程序不崩、输出看着也像那么回事，但少了一个拓扑类，或者多出两条其实一样的路径。
所以请先把 graph.py 开头那四条**不变量**读一遍，它们是你调试时唯一的抓手。

你在阶段 1 犯的五个错误，每一个在这里都有一个同形版本：

  阶段 1                          阶段 2 的同形错误
  ------------------------------  --------------------------------------------
  c1 * c1 以为是标量其实是数组      节点 id / 下标 / State 对象三者混用
  a - b 方向写反，不报错只给错答案    边的时间顺序写反，图惄惄少一半边
  除零检查写在除法之后             空分支没先处理（图里没 guard、一条路径都没找到）
  return 写在 for 里面提前退出      visible_guards 数到第一个就返回
  用 robot_radius 而不是 inflated  自己手算路径长度，而不用 Trajectory.length

还有一条上次没有、这次新增的：**随机算法必须可复现**。所有随机性都走显式传入的
rng，不要用 np.random.* 的全局状态。否则你会遇到「跑三次过两次」这种最难受的情况。
"""

from __future__ import annotations

import exercise
import numpy as np
from graph import Graph, NodeKind
from world import State, Trajectory, World


def state_collision_free(x: State, world: World) -> bool:
    """单个状态点本身是否无碰撞。

    这个函数是白送的——因为你阶段 1 的零长线段分支写对了，
    点碰撞检测就是「起点和终点重合的时空线段」。一行就能写完。
    （zero_length_free / zero_length_collision 那两个用例的回报在这里。）

    Returns:
        True 表示该点不在任何障碍里。
    """
    return exercise.segment_collision_free(x, x, world)


def edge_feasible(p: State, q: State, world: World) -> bool:
    """p 和 q 之间能不能连一条合法的边？运动学 + 碰撞都要过。

    这是阶段 2 的底层原语，地位相当于阶段 1 的 segment_collision_free。

    ⭐ **本阶段最大的坑，就在这一个函数里。**

    可见性在时空里**不是对称关系**：connection_valid 要求 b.t > a.t，
    所以 connection_valid(p, q) 和 connection_valid(q, p) 不可能同时成立。
    但「p 和 q 能不能连」这件事本身是对称的，和你怎么传参无关。

    所以请在**这个函数内部**按 .t 把两个点排好序，让调用方完全不用操心顺序。
    这是「让错误只可能发生在一个地方」，和上次用 inflated_radius() 而不是
    自己拼半径是同一个道理。写反了**不会报错**，只会让图少一半边。

    另外注意顺序：先做 O(1) 的运动学检查，再做 O(|障碍|) 的碰撞检查。
    这个函数会被调几十万次，顺序直接决定能不能跑。

    Returns:
        True 表示这条边可以存在。
    """
    # 这里的运动学就是时间约束+速度约束
    # 注意，这里不能时间相同(在 valid 中进行检测，这里不用再次检测)
    # 排序 → 运动学 → 碰撞
    if p.t > q.t:
        p, q = q, p

    if not exercise.connection_valid(p, q, world):
        return False

    return exercise.segment_collision_free(p, q, world)


# 这个函数的职责只有一个：定义"往哪撒点"，然后撒一个出来。
def sample_state(
    world: World, start: State, goal: State, rng: np.random.Generator
) -> State | None:
    """从可行状态分布 P_PRM 采一个样本（论文 Algorithm 1 第 8 行）。

    论文的 P_PRM 是一个考虑速度与加速度上限的前向扇形弧区域。先别照抄，
    本阶段只考虑**速度上限**（加速度要等阶段 4 有了样条才谈得上，
    因为折线在拐点不可导）。

    要求采出来的点同时满足：
    1. start.t <= t <= goal.t
    2. 从 start 赶得到：   ||p - start.xy|| <= v_max * (t - start.t)
    3. 还赶得上 goal：     ||goal.xy - p|| <= v_max * (goal.t - t)
    4. 点本身无碰撞

    条件 2 和 3 是两个圆盘的交集，是一个**透镜形**。最简单的做法是拒绝采样：
    先抽 t，再算出这个透镜形的包围盒，在盒子里均匀抽点，不合格就重抽。

    其实自己没有理解到，为什么两个圆盘，就会构成一个透镜形。。。
          透镜形就是两个圆盘重叠的部分，外观像一枚凸透镜，两侧边界各是一段圆弧。

    为什么要把条件 3 加上：不加的话，大量样本会落在「能走到但回不来」的
    地方，它们永远只能看见 1 个 guard，注定被丢弃。这是「定义域选错了」的
    另一个版本，和上次把 u 的上界 clamp 成 10 是同一类毛病。

    陷阱：
    - 所有随机数都用传进来的 rng，不要用 np.random.uniform 等全局接口。
    - 拒绝采样要有**次数上限**，否则透镜形为空或者几乎全被障碍盖住时会死循环。
    - 包围盒可能是空的（lo >= hi），要先挡掉再抽。
                     low, high
    Returns:
        一个合法的 State；实在采不到就返回 None（调用方负责跳过）。
    """
    for _ in range(1000):
        # 对于这个采样，自己想错了，自己以为 start\goal 框定的区域就是整个区域。。。
        # 实际上并不是，start和goal的xy虽然也可以框出一个矩形，但特殊的是，当  start.y/x == goal.y/x 时，就会退化为一条直线。
        t = rng.uniform(start.t, goal.t)
        xys = world.max_velocity * (t - start.t)
        xye = world.max_velocity * (goal.t - t)
        # x = rng.uniform(goal.x - xye, start.x + xys)
        # y = rng.uniform(goal.y - xye, start.y + xys)
        xmin = max(start.x - xys, goal.x - xye)
        xmax = min(start.x + xys, goal.x + xye)

        ymin = max(start.y - xys, goal.y - xye)
        ymax = min(start.y + xys, goal.y + xye)

        if xmin > xmax or ymin > ymax:
            continue

        x = rng.uniform(xmin, xmax)
        y = rng.uniform(ymin, ymax)
        new_state = State(x, y, t)

        dts = new_state.t - start.t
        dte = goal.t - new_state.t
        if dts <= 0 or dte <= 0:
            continue

        delta_start = new_state.xy - start.xy
        d2_start = delta_start @ delta_start
        if d2_start > xys**2:
            continue
        delta_end = goal.xy - new_state.xy
        d2_end = delta_end @ delta_end
        if d2_end > xye**2:
            continue
        if not state_collision_free(new_state, world):
            continue

        return new_state

    return None


# 必须扫完全部 guard，因为返回列表的长度（0 / 2 / 其他）决定 try_add_sample 走哪条分支，提前 break 会让长度偏小且不报任何错。
def visible_guards(x: State, graph: Graph, world: World) -> list[int]:
    """找出新样本 x 能无碰撞连到的所有 guard，论文里记作 L（Algorithm 1 第 9 行）。

    ⭐ **必须扫完全部 guard，不能提前返回。**

    因为返回值的**长度** 0 / 2 / 其他分别对应三条完全不同的分支，
    一旦看见第一个就 break，|L| 就是错的，而且**不会报错**。
    这是你上次「退化分支里写了 return 而不是 continue」的同形版本。

    Returns:
        guard 的 **id** 列表。顺序不重要，判卷会排序后比对。
    """
    L: list[int] = []
    for node in graph.nodes.values():
        if node.kind is not NodeKind.GUARD:
            continue
        if edge_feasible(x, node.state, world):
            L.append(node.id)

    return L


def try_add_sample(x: State, graph: Graph, world: World, num_samples: int = 20) -> bool:
    """Algorithm 1 第 10-32 行：尝试把一个样本加进图里。**本阶段的心脏。**

    按 |L| = len(visible_guards(x)) 分三条路：

    - **|L| = 0**：谁都看不见。它开辟了一块之前谁也够不着的区域 -> 升为新 guard。
    - **|L| = 2**：候选 connector，走下面的去重流程。
    - **其他（看见 1 个或 >= 3 个）**：直接丢弃。

      「看见 1 个就丢」是 Visibility-PRM 稀疏性的来源：它连不起任何东西，
      加进来只增加边数、不增加拓扑信息。别写成 len(L) >= 2。

    |L| = 2 分支的完整流程：

    1. 把两个 guard 按时间排序，记作 g0（早）和 g1（晚）。
    2. **论文没明说、但你必须自己定的规则**：要求 g0.t < x.t < g1.t。
       如果两个 guard 都在 x 之前（或都在之后），路径 g0 -> x -> g1 就得在
       某一段上倒着走时间，不是合法路径，应当丢弃。
    3. 构造新路径 tau = Path(g0, x, g1)（三个点的折线，记得带 world.time_scale）。
    4. 对 graph.connectors_between(g0, g1) 里的**每一个**旧 connector x_j：
       构造 tau_j = Path(g0, x_j, g1)，用阶段 1 的 uvd_equivalent 判断。
       - 若等价：说明拓扑类不是新的。此时比**长度**：
         新的更短 -> remove_connector(x_j) 后把 x 加进来（论文的「替换」）；
         否则 -> 丢弃 x，图不变。
       - 若和所有旧 connector 都不等价：这是一个**新的拓扑类**，加进图里。

    两个陷阱：
    - 去重只能在**共享同一对 guard** 的 connector 之间做。因为 uvd_equivalent 要求
      两条轨迹起点终点都相同，而 g0 -> x -> g1 和 g0 -> x_j -> g1 恰好满足。
      用 graph.connectors_between()，别去遍历全图。
    - 「是新类」还是「是旧类」这个布尔标志的极性很容易写反。你上次就把
      「碰撞」和「无碰撞」写反过一次。

    长度请用 Trajectory.length，它里面乘了 time_scale。自己写 hypot(dx, dy)
    不含时间维，会给出不同的替换决策。

    Returns:
        True 表示图被改动了（新增或替换）。判卷不看返回值，只看最终的图。
    """
    raise NotImplementedError("请实现 try_add_sample")


def build_prm(
    world: World,
    start: State,
    goal: State,
    num_samples: int,
    rng: np.random.Generator,
    sampler=None,
) -> Graph:
    """Algorithm 1 的主循环（第 1-3 行 + 第 8 行）。十来行的胶水。

    1. 建一个空 Graph，把 start 和 goal 都加为 guard。
       ⭐ 顺序要求：start 先、goal 后，因此它们的 id 必须是 0 和 1。
       enumerate_paths 和判卷都按这个约定来。
    2. 循环 num_samples 次：采一个样本，是 None 就跳过，否则 try_add_sample。

    sampler 参数：为 None 时用你的 sample_state；否则用传进来的，签名一致。
    这是专门为测试留的口子（见 graph.FixedSampler）。随机算法如果没有这个口子，
    你就只能靠肉眼看图调试。

    论文第 4-6 行的 ReintroduceSample（跨帧传播）属于阶段 3，这里每次从零重建就行。
    """
    raise NotImplementedError("请实现 build_prm")


def enumerate_paths(
    graph: Graph, start_id: int = 0, goal_id: int = 1, max_paths: int = 2000
) -> list[list[int]]:
    """带访问列表的 DFS，枚举从 start 到 goal 的所有路径（Algorithm 1 最后一步）。

    这张图是个 **DAG**：每条边在时间上严格向前，所以不可能成环，
    你根本不需要防环逻辑——这是时间维度白送的。

    但反过来，**你必须自己把时间方向筛出来**。graph.neighbors() 返回的是
    无向邻居，其中包括时间更早的那些。如果只靠一个 visited 集合防重复访问、
    不比较 .t，你会枚举出「倒着走时间」的假路径。backward_edge_trap 那个用例
    就是专门来抓这个的。

    其他要求：
    - 路径是节点 id 的列表，开头是 start_id、结尾是 goal_id。
    - 路径数会组合爆炸，用 max_paths 封顶。
    - 空分支要先挡：一条路径都找不到时返回空列表，不要报错。
    """
    raise NotImplementedError("请实现 enumerate_paths")


def distinct_trajectories(
    paths: list[list[int]], graph: Graph, world: World, num_samples: int = 20
) -> list[Trajectory]:
    """把 DFS 枚举出的路径去重，得到论文的几何轨迹集合 T*。

    为什么 DFS 之后还要再过一遍 UVD：try_add_sample 的构造只保证「段」
    两两不同类，不保证「整条路径」两两不同类——两条路径走了不同的 guard 对，
    完全可能最终是同一种绕法。所幸所有完整路径的起点终点都是 start 和 goal，
    UVD 的前提天然满足。

    ⭐ **概念上最容易翻车的一点：UVD 不是等价关系，它不满足传递性。**

    A 和 B 等价、B 和 C 等价，并不意味着 A 和 C 等价。所以「把路径分组」
    这个说法根本不严谨，你也写不出一个良定义的分组函数。实际做法只能是贪心：

        按代价从低到高排序，依次尝试加入结果集，
        只要和**已选中的任何一条**等价就丢弃。

    没想清楚这一点的话，你会写出一个结果依赖遍历顺序、每次跑都不一样的去重，
    然后花半天怀疑是采样的随机性在作祟。排序让结果变得确定，而且保证每个
    类里留下的是最短的那条。

    代价函数本阶段用最朴素的「状态空间弧长」就行（Trajectory.length）。
    真正的 J_geo + J_smooth 是阶段 4 的事。

    Returns:
        两两 UVD 不等价的 Trajectory 列表。
    """
    raise NotImplementedError("请实现 distinct_trajectories")
