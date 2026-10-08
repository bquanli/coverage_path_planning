"""阶段 2 的验收用例。脚手架，不需要修改。

阶段 1 的用例全是「给输入、比对唯一正确答案」，因为那三个函数是确定性的。
阶段 2 有随机采样，这套做法只对其中一半管用，所以用例分四类：

1. **确定性单元用例**（EDGE / VISIBLE / ADD / DFS）——样本是手写注入的，
   答案唯一。这是你的主力，和阶段 1 一样硬。
2. **性质测试**（SAMPLE / 图不变量）——不依赖任何期望答案，只断言
   「无论什么场景、什么种子，这条都必须成立」。性价比最高的一类。
3. **多种子统计用例**（TOPOLOGY）——PRM 只是概率完备的，偶尔漏一类是正常的，
   但「偶尔」必须被量化，否则你分不清是算法错了还是运气差。
4. **观察用例**（expected_classes=None）——不判对错，只打印结果让你看。
"""

from __future__ import annotations

from dataclasses import dataclass, field

from world import State, World, obstacle

# 所有场景共用：机器人半径 0.2、障碍半径 0.3 => R = 0.5。
START = State(-3.0, 0.0, 0.0)
GOAL = State(3.0, 0.0, 3.0)


def make_world(*obstacles, max_velocity: float = 3.0, horizon: float = 3.0,
               time_scale: float = 1.0) -> World:
    return World(obstacles=tuple(obstacles), robot_radius=0.2, horizon=horizon,
                 max_velocity=max_velocity, time_scale=time_scale)


EMPTY = make_world()
STATIC = make_world(obstacle(0.0, 0.0))
TWO_GAP = make_world(obstacle(0.0, 1.0), obstacle(0.0, -1.0))
# 行人在 t=1.5 正好走到原点，和机器人迸个正着。
CROSSING = make_world(obstacle(0.0, -4.5, vy=3.0))
BIG = make_world(obstacle(0.0, 0.0, radius=1.5))


# =====================================================================
# 1. edge_feasible
# =====================================================================

@dataclass(frozen=True)
class EdgeCase:
    name: str
    description: str
    world: World
    p: State
    q: State
    expected: bool


EDGE_CASES: list[EdgeCase] = [
    EdgeCase("forward_free", "空世界、时间向前、速度达标。",
             EMPTY, State(0.0, 0.0, 0.0), State(1.0, 0.0, 1.0), True),
    EdgeCase("reversed_arguments", "和上一个是**同一条边**，只是两个参数调了个个。"
             "可见性不应该依赖你怎么传参。内部忘了按时间排序就会挂在这里。",
             EMPTY, State(1.0, 0.0, 1.0), State(0.0, 0.0, 0.0), True),
    EdgeCase("same_time", "两端同时刻但不同位置：瞬移，永远不可连。",
             EMPTY, State(0.0, 0.0, 1.0), State(1.0, 0.0, 1.0), False),
    EdgeCase("same_state", "完全重合的两个状态。按「时间严格递增」仍不可连。",
             EMPTY, State(0.0, 0.0, 1.0), State(0.0, 0.0, 1.0), False),
    EdgeCase("too_fast", "5 m/s > max_velocity = 3。运动学不过关。",
             EMPTY, State(0.0, 0.0, 0.0), State(5.0, 0.0, 1.0), False),
    EdgeCase("blocked_by_obstacle", "运动学没问题，但穿过了静态障碍的圆心。",
             STATIC, State(-2.0, 0.0, 0.0), State(2.0, 0.0, 2.0), False),
    EdgeCase("around_obstacle", "绕开了，最近距离 1.5 > R。",
             STATIC, State(-2.0, 1.5, 0.0), State(2.0, 1.5, 2.0), True),
    EdgeCase("obstacle_gone_by_then", "投影到 2D 看是撞的，但行人那时候已经走远了。"
             "把障碍沿时间扫成静态区域的实现会挂在这里。",
             make_world(obstacle(0.0, 0.0, vy=4.0)),
             State(-2.0, 0.0, 0.0), State(2.0, 0.0, 2.0), True),
]


# =====================================================================
# 2. sample_state（性质测试）
# =====================================================================

@dataclass(frozen=True)
class SampleScenario:
    name: str
    description: str
    world: World
    start: State
    goal: State


SAMPLE_SCENARIOS: list[SampleScenario] = [
    SampleScenario("empty", "没有障碍，透镜形区域很大。", EMPTY, START, GOAL),
    SampleScenario("static", "中间一个静态障碍，采样要避开它。", STATIC, START, GOAL),
    SampleScenario("tight", "速度上限恰好够用，透镜形很瘦。采样容易在这里跑出范围。",
                   make_world(max_velocity=2.2), START, GOAL),
    SampleScenario("crossing", "横穿行人。采样点自身也得是无碰撞的。", CROSSING, START, GOAL),
]


# =====================================================================
# 3. visible_guards
# =====================================================================

@dataclass(frozen=True)
class VisibleCase:
    name: str
    description: str
    world: World
    guards: tuple[State, ...]
    x: State
    expected_indices: tuple[int, ...]
    """期望看见的 guard 在 guards 元祖里的**下标**（按添加顺序，就是它们的 id）。"""


VISIBLE_CASES: list[VisibleCase] = [
    VisibleCase("sees_both", "空世界里的中点，起终点都看得见 -> |L| = 2。",
                EMPTY, (START, GOAL), State(0.0, 0.0, 1.5), (0, 1)),
    VisibleCase("detour_sees_both", "绕到 y=1.5 上去，绕开了障碍，仍然两个都看得见。",
                STATIC, (START, GOAL), State(0.0, 1.5, 1.5), (0, 1)),
    VisibleCase("blocked_middle", "正在障碍正后方，两条连线都穿心而过 -> |L| = 0。",
                STATIC, (START, GOAL), State(0.0, 0.0, 1.5), ()),
    VisibleCase("only_goal", "离起点太远、速度不够，只看得见终点 -> |L| = 1，丢弃。",
                EMPTY, (START, GOAL), State(5.0, 0.0, 2.0), (1,)),
    VisibleCase("sees_nothing", "跑到谁都够不着的地方 -> |L| = 0，升为新 guard。",
                EMPTY, (START, GOAL), State(10.0, 5.0, 1.5), ()),
    VisibleCase("sees_three", "三个 guard 全看得见 -> |L| = 3。这是个人造的图（真实的 PRM 里"
                "guard 互相不可见），目的是让你必须真的数完。",
                EMPTY, (START, GOAL, State(0.0, 2.0, 0.8)),
                State(0.5, 0.5, 1.8), (0, 1, 2)),
    VisibleCase("three_guards_sees_two", "图里有三个 guard，只看得见其中两个。"
                "数完全部再分类，看见第一个就 break 的实现会挂。",
                EMPTY, (START, GOAL, State(10.0, 5.0, 1.5)),
                State(0.0, 0.0, 1.5), (0, 1)),
]


# =====================================================================
# 4. try_add_sample（按顺序注入样本，断言最终的图）
# =====================================================================

@dataclass(frozen=True)
class AddCase:
    name: str
    description: str
    world: World
    samples: tuple[State, ...]
    expected_guards: int
    expected_connectors: int
    expected_connector_states: tuple[State, ...] | None = None
    """若不为 None，还要求最终留下的 connector 坐标恰好是这些（不讲顺序）。"""
    extra_guards: tuple[State, ...] = ()
    """除起终点之外预先放好的 guard。用来造出 |L| >= 3 这种局面。"""


_LEFT = State(0.0, 1.5, 1.5)       # 从 y>0 绕
_LEFT_SHORT = State(0.0, 1.2, 1.5)  # 同一拓扑类，但更短 -> 应该替换掉 _LEFT
_LEFT_LONG = State(0.0, 1.8, 1.5)   # 同一拓扑类，但更长 -> 应该被丢弃
_RIGHT = State(0.0, -1.5, 1.5)      # 从 y<0 绕 -> 新拓扑类
_LONELY = State(10.0, 5.0, 1.5)     # 谁都看不见 -> 新 guard
_ONE_ONLY = State(5.0, 0.0, 2.0)    # 只看得见终点 -> 丢弃

ADD_CASES: list[AddCase] = [
    AddCase("empty_one_connector", "空世界，一个中点成为 connector。",
            EMPTY, (State(0.0, 0.0, 1.5),), 2, 1),
    AddCase("second_is_duplicate", "空世界里的两个中点必然 UVD 等价，只能留一个。",
            EMPTY, (State(0.0, 0.0, 1.5), State(0.0, 0.5, 1.4)), 2, 1),
    AddCase("left_then_right", "左绕和右绕是两个拓扑类，两个都要留。",
            STATIC, (_LEFT, _RIGHT), 2, 2, (_LEFT, _RIGHT)),
    AddCase("shorter_replaces", "同一拓扑类里来了更短的，要**替换**旧的。"
            "connector 数不变，但坐标换成了新的。",
            STATIC, (_LEFT, _LEFT_SHORT), 2, 1, (_LEFT_SHORT,)),
    AddCase("longer_rejected", "同一拓扑类里来了更长的，直接丢弃，图不变。",
            STATIC, (_LEFT, _LEFT_LONG), 2, 1, (_LEFT,)),
    AddCase("lonely_becomes_guard", "|L| = 0 -> 升为新 guard。",
            EMPTY, (_LONELY,), 3, 0),
    AddCase("single_visible_discarded", "|L| = 1 -> 丢弃，图完全不变。"
            "这条规则是 Visibility-PRM 稀疏性的来源。",
            EMPTY, (_ONE_ONLY,), 2, 0),
    AddCase("three_visible_discarded", "|L| = 3 -> 同样丢弃。只有恰好 2 才是 connector，"
            "写成 len(L) >= 2 就会挂在这里。",
            EMPTY, (State(0.5, 0.5, 1.8),), 3, 0, (),
            extra_guards=(State(0.0, 2.0, 0.8),)),
    AddCase("full_sequence", "完整走一遍：左绕、更短的左绕、更长的左绕、右绕、"
            "看不见的、只看见一个的。最终：3 个 guard + 2 个 connector。",
            STATIC, (_LEFT, _LEFT_SHORT, _LEFT_LONG, _RIGHT, _LONELY, _ONE_ONLY),
            3, 2, (_LEFT_SHORT, _RIGHT)),
]


# =====================================================================
# 5. enumerate_paths（手搭的图）
# =====================================================================

@dataclass(frozen=True)
class DfsCase:
    name: str
    description: str
    guards: tuple[State, ...]
    connectors: tuple[tuple[State, int, int], ...]
    """(connector 的状态, guard 下标 a, guard 下标 b)。guard 下标就是它们的 id。"""
    expected_paths: tuple[tuple[int, ...], ...]
    """期望的全部路径（节点 id 序列）。不讲顺序，但集合要完全相等。"""


# id 分配：guard 先按顺序 0,1,2...，然后 connector 接着编。
DFS_CASES: list[DfsCase] = [
    DfsCase("single_connector", "最简单的图：start -> c -> goal。",
            (START, GOAL), ((State(0.0, 0.0, 1.5), 0, 1),),
            ((0, 2, 1),)),
    DfsCase("two_parallel", "同一对 guard 之间两座桥 -> 两条路径。",
            (START, GOAL), ((State(0.0, 1.5, 1.5), 0, 1),
                            (State(0.0, -1.5, 1.5), 0, 1)),
            ((0, 2, 1), (0, 3, 1))),
    DfsCase("chain_through_guard", "中间多了一个 guard，路径要穿两个 connector。"
            "注意这条路径的长度是 5，不是 3。",
            (START, GOAL, State(0.0, 2.0, 1.5)),
            ((State(-1.5, 1.0, 0.7), 0, 2), (State(1.5, 1.0, 2.3), 2, 1)),
            ((0, 3, 2, 4, 1),)),
    DfsCase("dead_end", "有一个 guard 挂在旁边但通不到终点，不应该产生路径。",
            (START, GOAL, State(0.0, 2.0, 1.5)),
            ((State(0.0, 0.0, 1.5), 0, 1), (State(-1.5, 1.0, 0.7), 0, 2)),
            ((0, 3, 1),)),
    DfsCase("backward_edge_trap",
            "g2 的时间（2.6）比 g3（0.6）晚。无向图上从 start 到 goal 有两条简单路径，"
            "但其中一条要沿着 2.6 -> 1.6 -> 0.6 倒着走时间，不合法。"
            "只靠 visited 集合防环、不筛时间方向的 DFS 会在这里多吞一条。",
            (START, GOAL, State(0.0, 2.0, 2.6), State(0.0, -2.0, 0.6)),
            ((State(-1.5, 1.0, 1.3), 0, 2),
             (State(0.0, 0.0, 1.6), 2, 3),
             (State(1.5, -1.0, 1.8), 3, 1),
             (State(-1.5, -1.0, 0.3), 0, 3)),
            ((0, 7, 3, 6, 1),)),
    DfsCase("no_path", "起终点之间根本没桥。空分支要先挡掉，别让它报错。",
            (START, GOAL), (), ()),
]


# =====================================================================
# 6. 端到端的拓扑类数（多种子统计）
# =====================================================================

@dataclass(frozen=True)
class TopologyScenario:
    name: str
    description: str
    world: World
    expected_classes: int | None
    min_rate: float = 1.0
    start: State = START
    goal: State = GOAL
    num_samples: int = 150


TOPOLOGY_SCENARIOS: list[TopologyScenario] = [
    TopologyScenario("empty", "没有障碍，只有一种走法：直连。", EMPTY, 1, 1.0),
    TopologyScenario("single_static", "一个静态障碍 -> 左绕 / 右绕，恰好 2 类。",
                     STATIC, 2, 1.0),
    TopologyScenario(
        "crossing_pedestrian",
        "一个横穿行人。论文 Fig. 1 画了四种行为（左绕/右绕/抢行/让行），"
        "但在这个无边界的场景里正确答案是 **2**。"
        "在状态空间里行人是一根从底面贯到顶面的斜管，"
        "「抢行」和「从行人身后绕」其实是同一个通道。想不明白就画出来看。",
        CROSSING, 2, 1.0),
    TopologyScenario("two_static_gap", "两个静态障碍夹出三条走廊 -> 3 类。"
                     "中间那条很窄，采样偶尔会错过，所以只要求 80% 的种子找全。",
                     TWO_GAP, 3, 0.8),
    TopologyScenario("big_blocker", "一个很大的障碍，会逼出额外的 guard。"
                     "结果不稳定，**不判对错**，只是让你看看图长成什么样。",
                     BIG, None),
]


def find_topology_scenario(name: str) -> TopologyScenario:
    for scenario in TOPOLOGY_SCENARIOS:
        if scenario.name == name:
            return scenario
    known = ", ".join(s.name for s in TOPOLOGY_SCENARIOS)
    raise SystemExit(f"没有这个场景：{name}。可选：{known}")
