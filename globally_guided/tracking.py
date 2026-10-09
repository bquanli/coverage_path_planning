"""阶段 3 的跨帧记账脚手架。这个文件是脚手架，不需要修改。

===============================================================
心智模型：阶段 3 给规划器装上了「记忆」
===============================================================

阶段 2 结束时，`build_prm` 是一个**无状态的单帧函数**：嗂一个世界快照，
吐一组通道。没有任何东西把这一帧的输出和上一帧的输出联系起来。

阶段 3 要回答的是第三个问题：「这一帧的这条，和上一帧的那条，是不是同一条？」
为此要引入两套东西：

* **复用**（ReintroduceSample）：新一帧的图不从零长，先把上一帧的节点
  按 (x, y, t - h) 放回去。
* **身份**（segment ID / trajectory ID）：给每座桥和每条轨迹发一个跨帧稳定的编号。

这个文件装的是这两套东西的**管道**：发号器、帧容器、几个小工具，
以及把你写的八个函数串成多帧闭环的 `run_frames`。
策略全在 exercise3.py 里，这里一行策略都没有。

---------------------------------------------------------------
一个「帧」到底是什么
---------------------------------------------------------------

一帧 = 一个控制周期。两帧之间过了 h 秒（论文里 h = 0.05 s）。

关键约定：**每一帧都有自己的时间原点**。当前帧的 t = 0 就是「现在」，
t = horizon 是规划窗口的末端。所以进入下一帧时有两件事要做：

  1. 世界往前走 h：每个障碍物的 t=0 圆心变成 p + v*h。
  2. 旧图往回走 h：每个节点的 (x, y, t) 变成 (x, y, t - h)。

这两件事方向相反，但说的是同一件事：时间原点往前挪了 h。
障碍物是「世界的状态」，原点前移它就显得走了；节点是「未来的计划」，
原点前移它就显得近了。搞混符号是本阶段第一个坑。
"""

from __future__ import annotations

import itertools
from dataclasses import dataclass, field

from graph import Graph
from world import State, Trajectory, World

# ------------------------------------------------------------------ 小工具


def states_close(a: State, b: State, tol: float = 1e-6) -> bool:
    """两个状态是不是「同一个点」。

    为什么不直接用 == ：State 是 frozen dataclass，== 是逐字段精确比较。
    t - h 这种浮点运算做两次未必位模一样，所以跨帧比较一律走这个函数。
    （这是阶段 2 「用 id 做身份、别用 State 做身份」的同一条教训，
    只是跨帧场景下你手上真的只剩坐标可用。）
    """
    return abs(a.x - b.x) <= tol and abs(a.y - b.y) <= tol and abs(a.t - b.t) <= tol


class IdAllocator:
    """单调递增的发号器。ID 一旦发出就永不回收。

    永不回收是故意的：如果一个编号被回收后重新发给别的东西，
    「跨帧同一个」这个语义就碎了。编号在这里是身份，不是下标。
    """

    def __init__(self, first: int = 1) -> None:
        self._next = int(first)

    def new_id(self) -> int:
        out = self._next
        self._next += 1
        return out

    @property
    def issued(self) -> int:
        """已经发出去多少个号。测试里用它查「是不是白白发了新号」。"""
        return self._next - 1

    def __repr__(self) -> str:
        return f"IdAllocator(next={self._next})"


def segment_trajectory(
    graph: Graph, connector_id: int, middle: State | None = None
) -> Trajectory:
    """把一个 connector 连同它的两个 guard 变成三点折线，也就是一个「段」。

    论文里的 segment 就是这个东西：g_early -> c -> g_late。
    segment ID 贴在 connector 上，因为一个 connector 就唯一决定了一个段。

    middle 不为 None 时，用它替换中间那个点，两端的 guard 不变。
    这个参数是专门为 assign_segment_ids 留的：比较「旧段搬过来」和「新段」时，
    两端一律用**新帧的** guard，只让中间点不同。这样 UVD 的前提
    （起点相同、终点相同）就天然成立，而且比的恰好是「这两座桥是不是
    同一种绕法」——和阶段 2 里 try_add_sample 做的判断一模一样。
    """
    node = graph.nodes[connector_id]
    if node.guards is None:
        raise ValueError(f"节点 {connector_id} 不是 connector")
    g_early, g_late = node.guards
    mid = node.state if middle is None else middle
    return Trajectory(
        states=[graph.state(g_early), mid, graph.state(g_late)],
        time_scale=graph.world.time_scale,
        name=f"seg{connector_id}",
    )


def path_segment_key(path: list[int], segment_ids: dict[int, int]) -> frozenset[int]:
    """一条路径的**段集合**，也就是论文说的「轨迹 -> 段集合」映射的键。

    path 里 guard 和 connector 是交替的，这里只挑 connector（即在
    segment_ids 里有登记的那些）。用 frozenset 而不是 tuple，是因为
    论文写的就是「由段 1 和 4 组成的轨迹」，强调的是成分、不是顺序。
    """
    return frozenset(segment_ids[n] for n in path if n in segment_ids)


def state_at_time(traj: Trajectory, t: float) -> State:
    """沿轨迹找出 t 时刻的位置。

    注意和 Trajectory.at(s) 的区别：at() 的参数是弧长进度 s，这里的参数是
    真实时间 t。机器人过了 h 秒之后走到哪里，问的是后者。
    """
    states = traj.states
    if t <= states[0].t:
        return states[0]
    for a, b in itertools.pairwise(states):
        if t <= b.t:
            span = b.t - a.t
            ratio = 0.0 if span == 0.0 else (t - a.t) / span
            return State(
                x=a.x + (b.x - a.x) * ratio,
                y=a.y + (b.y - a.y) * ratio,
                t=t,
            )
    return states[-1]


# -------------------------------------------------------------------- 帧


@dataclass
class Frame:
    """一个控制周期的全部产出。

    它就是论文里的 G⁻：下一帧要拿它做两件事——把 graph 的节点搬过去
    （复用），把 segment_ids / trajectory_ids 查一遍（身份）。
    """

    world: World
    start: State
    goal: State
    graph: Graph
    segment_ids: dict[int, int] = field(default_factory=dict)
    """connector 节点 id -> segment ID (alpha)。注意键是**本帧的节点 id**，
    值才是跨帧稳定的那个编号。两者千万别混。"""
    paths: list[list[int]] = field(default_factory=list)
    """去重之后剩下的路径（节点 id 序列），与 trajectories 一一对应。"""
    trajectories: list[Trajectory] = field(default_factory=list)
    trajectory_ids: list[int] = field(default_factory=list)
    """与 trajectories 一一对应的 trajectory ID (beta)。"""
    chosen: int | None = None
    """本帧选中的轨迹的 beta。注意存的是 **beta** 而不是下标——下标下一帧就作废了。"""

    def trajectory_map(self) -> dict[frozenset[int], int]:
        """「段集合 -> beta」的映射，下一帧 assign_trajectory_ids 的输入。"""
        return {
            path_segment_key(p, self.segment_ids): b
            for p, b in zip(self.paths, self.trajectory_ids)
        }

    @property
    def chosen_trajectory(self) -> Trajectory | None:
        for traj, beta in zip(self.trajectories, self.trajectory_ids):
            if beta == self.chosen:
                return traj
        return None

    def __repr__(self) -> str:
        return (
            f"Frame(guards={len(self.graph.guard_ids())}, "
            f"connectors={len(self.graph.connector_ids())}, "
            f"alphas={sorted(self.segment_ids.values())}, "
            f"betas={self.trajectory_ids}, chosen={self.chosen})"
        )


# ------------------------------------------------------------ 多帧驱动


def run_frames(
    world: World,
    start: State,
    goal_xy: tuple[float, float],
    num_frames: int,
    h: float,
    num_samples: int,
    rng,
    consistency_penalty: float = 0.0,
    warm: bool = True,
    uvd_samples: int = 20,
) -> list[Frame]:
    """把 exercise3 的八个函数串成闭环，跑 num_frames 帧。脚手架，不用改。

    一帧内的顺序就是 Algorithm 1 的顺序：

        1. 把上一帧的节点搬到本帧坐标系      reintroduce_states
        2. 先复用、再采新样本，建图             build_prm_warm
        3. 对上两帧的 guard                      match_guards
        4. 给每座桥发/继承 segment ID             assign_segment_ids
        5. 枚举 + 去重（阶段 2 的活）
        6. 给每条轨迹发/继承 trajectory ID         assign_trajectory_ids
        7. 带一致性惩罚地选一条                   select_guidance

    然后时间原点前移 h：世界往前走（advance_world），机器人沿选中的轨迹
    走到 t = h 的那个位置，在新帧里重新从 t = 0 算起。
    没找到任何轨迹时机器人原地不动。

    warm=False 是对照组：不做复用，每帧从零建图。配合
    consistency_penalty=0 就是「没有阶段 3」的基线。
    """
    import exercise2
    import exercise3

    frames: list[Frame] = []
    prev: Frame | None = None
    seg_alloc = IdAllocator()
    traj_alloc = IdAllocator()
    cur_world, cur_start = world, start

    for _ in range(num_frames):
        goal = State(goal_xy[0], goal_xy[1], cur_world.horizon)

        reintroduced: list[State] = []
        if warm and prev is not None:
            reintroduced = list(exercise3.reintroduce_states(prev.graph, cur_world, h))
        graph = exercise3.build_prm_warm(
            cur_world, cur_start, goal, num_samples, rng, reintroduced
        )

        guard_map = (
            exercise3.match_guards(prev.graph, graph, h) if prev is not None else {}
        )
        segment_ids = exercise3.assign_segment_ids(
            graph,
            prev.graph if prev is not None else None,
            guard_map,
            prev.segment_ids if prev is not None else {},
            seg_alloc,
            cur_world,
            h,
            uvd_samples,
        )

        raw_paths = exercise2.enumerate_paths(graph)
        trajs = exercise2.distinct_trajectories(
            raw_paths, graph, cur_world, uvd_samples
        )
        by_states = {tuple(graph.state(n) for n in p): p for p in raw_paths}
        paths = [by_states[tuple(tr.states)] for tr in trajs]

        betas = list(
            exercise3.assign_trajectory_ids(
                paths,
                segment_ids,
                prev.trajectory_map() if prev is not None else {},
                traj_alloc,
            )
        )
        index = exercise3.select_guidance(
            trajs,
            betas,
            prev.chosen if prev is not None else None,
            consistency_penalty,
        )
        chosen = betas[index] if 0 <= index < len(betas) else None

        frame = Frame(
            world=cur_world,
            start=cur_start,
            goal=goal,
            graph=graph,
            segment_ids=segment_ids,
            paths=paths,
            trajectories=trajs,
            trajectory_ids=betas,
            chosen=chosen,
        )
        frames.append(frame)

        picked = frame.chosen_trajectory
        next_xy = state_at_time(picked, h) if picked is not None else cur_start
        cur_start = State(next_xy.x, next_xy.y, 0.0)
        cur_world = exercise3.advance_world(cur_world, h)
        prev = frame

    return frames
