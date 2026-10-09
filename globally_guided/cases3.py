"""阶段 3 的验收用例。脚手架，不需要修改。

阶段 2 的用例分四类（确定性单元 / 性质测试 / 多种子统计 / 观察），
这里多了第五类，而且它是本阶段真正的重头戏：

5. **时序断言**（CLOSED_SCENARIOS）——连续跑 N 帧，断言某个编号全程不变。

为什么必须有这一类：阶段 3 的 bug 在**单帧**上是看不出来的。
每一帧的图都满足四条不变量、每一帧的拓扑类数都对、每一帧选的也都是当前最短的，
然而机器人在原地画龙。只有把多帧串起来看，错误才有形状。
"""

from __future__ import annotations

from dataclasses import dataclass

from cases2 import CROSSING, EMPTY, GOAL, START, STATIC, TWO_GAP, make_world
from graph import Graph
from world import State, World, obstacle

H = 0.05   # 论文 Table I 的采样周期
GOAL_XY = (3.0, 0.0)

# 空场景，但角落里有个障碍，用来测「搬过来之后撞上了」。
CORNER = make_world(obstacle(0.0, 1.5))
# 行人从下往上走，用来测 advance_world 的符号。
WALKER = make_world(obstacle(0.0, -4.5, vy=3.0))


def build_graph(
    world: World,
    guards: tuple[State, ...],
    connectors: tuple[tuple[State, int, int], ...] = (),
) -> Graph:
    """手搭一张图。guard 先按顺序编 0,1,2...，connector 接着编。"""
    graph = Graph(world)
    for state in guards:
        graph.add_guard(state)
    for state, a, b in connectors:
        graph.add_connector(state, a, b)
    return graph


# =====================================================================
# 1. advance_world
# =====================================================================

@dataclass(frozen=True)
class AdvanceCase:
    name: str
    description: str
    world: World
    h: float
    expected_positions: tuple[tuple[float, float], ...]
    """推进之后每个障碍物的 t=0 圆心，按原顺序。"""


ADVANCE_CASES: list[AdvanceCase] = [
    AdvanceCase("no_obstacles", "空世界，什么都不该发生。空分支先挡。",
                EMPTY, H, ()),
    AdvanceCase("static_unchanged", "静态障碍速度为零，推多久都在原地。",
                STATIC, H, ((0.0, 0.0),)),
    AdvanceCase("sign_check", "⭐ 符号陷阱。行人 vy=+3，推 0.05 s 后 y 应该从 -4.5 变成 -4.35。"
                "写成 p - v*h 会得到 -4.65，程序不报错，只是行人开始倒着走。",
                WALKER, H, ((0.0, -4.35),)),
    AdvanceCase("big_step", "推 1.5 s，行人正好走到原点。",
                WALKER, 1.5, ((0.0, 0.0),)),
    AdvanceCase("zero_step", "h = 0 是恒等变换。",
                WALKER, 0.0, ((0.0, -4.5),)),
    AdvanceCase("two_obstacles", "两个静态障碍，顺序不能乱。",
                TWO_GAP, H, ((0.0, 1.0), (0.0, -1.0))),
]


# =====================================================================
# 2. shift_state
# =====================================================================

@dataclass(frozen=True)
class ShiftCase:
    name: str
    description: str
    state: State
    h: float
    expected: State | None


SHIFT_CASES: list[ShiftCase] = [
    ShiftCase("normal", "普通节点，只有 t 变。",
              State(1.0, 2.0, 2.0), H, State(1.0, 2.0, 1.95)),
    ShiftCase("xy_untouched", "空间坐标一定不要动。节点没走，是时间原点走了。",
              State(-3.0, 1.5, 1.0), 0.3, State(-3.0, 1.5, 0.7)),
    ShiftCase("exactly_zero", "⭐ 边界：t - h 恰好等于 0，返回 None。"
              "这个位置已经被新帧的 start guard 占了，它和 start 的时间差为 0，"
              "edge_feasible 一定拒绝。写成 < 0 会把它留下来白占地方。",
              State(0.0, 0.0, 0.05), H, None),
    ShiftCase("already_past", "t < h，已经滑出窗口。",
              State(0.0, 0.0, 0.02), H, None),
    ShiftCase("start_node", "上一帧的 start 节点 t = 0，搬过来必然过期。",
              State(-3.0, 0.0, 0.0), H, None),
    ShiftCase("zero_h", "h = 0 时原样返回（注意 t=1.0 > 0，不该被当成过期）。",
              State(0.0, 0.0, 1.0), 0.0, State(0.0, 0.0, 1.0)),
]


# =====================================================================
# 3. carry_connector
# =====================================================================

@dataclass(frozen=True)
class CarryCase:
    name: str
    description: str
    world: World
    guards: tuple[State, ...]
    connectors: tuple[tuple[State, int, int], ...]
    connector_id: int
    h: float
    expected: State | None


CARRY_CASES: list[CarryCase] = [
    CarryCase("alive", "没过期就和 shift_state 一样，不要多做事。",
              EMPTY, (START, GOAL), ((State(0.0, 0.8, 1.5), 0, 1),), 2, H,
              State(0.0, 0.8, 1.45)),
    CarryCase("resampled", "⭐ connector 滑到了 t=0.02，搬过来过期。不要丢，"
              "取段折线 start -> c -> goal 的弧长中点做替身，再减 h。",
              EMPTY, (START, GOAL), ((State(0.0, 0.8, 0.02), 0, 1),), 2, H,
              State(0.41784, 0.68858, 0.38506)),
    CarryCase("segment_gone", "整个段都快滑出窗口了（两个 guard 的 t 都极小），"
              "中点也过期，这才真的返回 None。",
              EMPTY, (State(-3.0, 0.0, 0.0), State(0.0, 0.0, 0.04)),
              ((State(-1.5, 0.5, 0.02), 0, 1),), 2, H, None),
]


# =====================================================================
# 4. reintroduce_states
# =====================================================================

@dataclass(frozen=True)
class ReintroCase:
    name: str
    description: str
    world: World
    """**本帧的**世界（已经 advance 过）。"""
    guards: tuple[State, ...]
    connectors: tuple[tuple[State, int, int], ...]
    h: float
    expected: tuple[State, ...]
    """期望输出的集合（不讲顺序）。顺序要求单独断言：guard 全在 connector 前。"""


_EXTRA_GUARD = State(0.0, 2.2, 1.6)

REINTRO_CASES: list[ReintroCase] = [
    ReintroCase("skips_seeds", "⭐ 节点 0 和 1 是上一帧的 start / goal，都要跳过。"
                "start 自己就过期了你可能没感觉，goal 在 t=3 搬过来是 2.95，**没过期**，"
                "不显式挡就会每帧在终点旁边多长出一个流浪 guard。",
                EMPTY, (START, GOAL), ((State(0.0, 0.0, 1.5), 0, 1),), H,
                (State(0.0, 0.0, 1.45),)),
    ReintroCase("guard_and_connector", "额外的 guard 和 connector 都要搬。",
                EMPTY, (START, GOAL, _EXTRA_GUARD),
                ((State(0.0, 1.0, 0.9), 0, 2), (State(0.0, 1.2, 2.2), 2, 1)), H,
                (State(0.0, 2.2, 1.55), State(0.0, 1.0, 0.85),
                 State(0.0, 1.2, 2.15))),
    ReintroCase("expired_connector_resampled", "过期的 connector 走 carry_connector，"
                "丢掉它等于丢掉一条通道的身份。",
                EMPTY, (START, GOAL), ((State(0.0, 0.8, 0.02), 0, 1),), H,
                (State(0.41784, 0.68858, 0.38506),)),
    ReintroCase("collision_dropped", "节点本身没问题，但本帧的世界里那里站着一个障碍。"
                "上一帧安全不代表这一帧安全。",
                CORNER, (START, GOAL), ((State(0.0, 1.5, 1.5), 0, 1),), H, ()),
]


# =====================================================================
# 5. build_prm_warm
# =====================================================================

@dataclass(frozen=True)
class WarmCase:
    name: str
    description: str
    world: World
    start: State
    goal: State
    reintroduced: tuple[State, ...]
    samples: tuple[State | None, ...]
    """嗂给 FixedSampler 的新样本。None 表示采样失败。"""
    num_samples: int
    expected_guards: int
    expected_connectors: int


WARM_CASES: list[WarmCase] = [
    WarmCase("seeds_only", "reintroduced 为空时应该和阶段 2 的 build_prm 一模一样。",
             EMPTY, START, GOAL, (), (State(0.0, 0.0, 1.5),), 1, 2, 1),
    WarmCase("reintro_costs_no_budget", "⭐ num_samples = 0，但复用的节点还是要进图。"
             "复用的节点不占采样预算：它们不是采来的，是上一帧花过预算挖出来的。"
             "写成「总共采 num_samples 个」的话这里会得到一张空图。",
             EMPTY, START, GOAL, (State(0.0, 0.0, 1.5),), (), 0, 2, 1),
    WarmCase("reintro_then_sample", "先复用再采样。复用的左绕先进图，"
             "新样本是右绕，最终两个拓扑类都在。",
             STATIC, START, GOAL, (State(0.0, 1.5, 1.5),),
             (State(0.0, -1.5, 1.5),), 1, 2, 2),
    WarmCase("none_sample_skipped", "采样器返回 None 是合法的（透镜形里采不到点），"
             "要跳过而不是报错。和阶段 2 的 build_prm 同一条要求。",
             EMPTY, START, GOAL, (), (None, State(0.0, 0.0, 1.5)), 2, 2, 1),
]


# =====================================================================
# 6. match_guards
# =====================================================================

@dataclass(frozen=True)
class GuardMapCase:
    name: str
    description: str
    world: World
    prev_guards: tuple[State, ...]
    new_guards: tuple[State, ...]
    h: float
    expected: dict[int, int]


GUARDMAP_CASES: list[GuardMapCase] = [
    GuardMapCase("seeds_only", "只有 start / goal。",
                 EMPTY, (START, GOAL), (START, GOAL), H, {0: 0, 1: 1}),
    GuardMapCase("seeds_moved", "⭐ 本阶段最致命的坑。机器人走了 h 秒，start 换了位置；"
                 "goal 又被推回了 t = horizon。它们的坐标**讲定对不上**，"
                 "但它们跨帧就是同一个东西（机器人和目的地），必须硬编码 0->0、1->1。"
                 "按坐标匹配的话所有段每帧都会拿新 ID，整套机制归零。",
                 EMPTY, (START, GOAL), (State(-2.9, 0.0, 0.0), GOAL), H,
                 {0: 0, 1: 1}),
    GuardMapCase("extra_matched", "额外的 guard 按「搬家后坐标重合」匹配。",
                 EMPTY, (START, GOAL, State(0.0, 2.0, 1.5)),
                 (START, GOAL, State(0.0, 2.0, 1.45)), H, {0: 0, 1: 1, 2: 2}),
    GuardMapCase("extra_died", "上一帧的 guard 这一帧没活下来（撞了/过期/被降成 connector）。"
                 "匹配不上就**不要放进映射**，别强行配对。",
                 EMPTY, (START, GOAL, State(0.0, 2.0, 1.5)), (START, GOAL), H,
                 {0: 0, 1: 1}),
    GuardMapCase("extra_moved_away", "新 guard 和搬家后的旧 guard 差了 0.5 m，不是同一个。",
                 EMPTY, (START, GOAL, State(0.0, 2.0, 1.5)),
                 (START, GOAL, State(0.0, 2.5, 1.45)), H, {0: 0, 1: 1}),
    GuardMapCase("forgot_shift", "新 guard 的 t 和旧 guard **完全相同**（都是 1.5）。"
                 "忘了减 h 的实现会在这里误匹配。",
                 EMPTY, (START, GOAL, State(0.0, 2.0, 1.5)),
                 (START, GOAL, State(0.0, 2.0, 1.5)), H, {0: 0, 1: 1}),
]


# =====================================================================
# 7. assign_segment_ids
# =====================================================================

_LEFT = State(0.0, 1.5, 1.5)
_RIGHT = State(0.0, -1.5, 1.5)
_NEW_START = State(-2.9, 0.0, 0.0)


@dataclass(frozen=True)
class SegIdCase:
    name: str
    description: str
    world: World
    prev_guards: tuple[State, ...] | None
    prev_connectors: tuple[tuple[State, int, int], ...]
    prev_segment_ids: dict[int, int]
    new_guards: tuple[State, ...]
    new_connectors: tuple[tuple[State, int, int], ...]
    h: float
    expected: dict[int, int]
    """{新 connector 节点 id: alpha}。发号器从 10 开始，所以 >= 10 的就是新号。"""


_PREV_BOTH = ((_LEFT, 0, 1), (_RIGHT, 0, 1))

SEGID_CASES: list[SegIdCase] = [
    SegIdCase("first_frame", "prev_graph 是 None，所有 connector 都发新号。空分支先挡。",
              STATIC, None, (), {}, (START, GOAL), _PREV_BOTH, H,
              {2: 10, 3: 11}),
    SegIdCase("survived", "两座桥原样搬过来，各自继承自己的 alpha。",
              STATIC, (START, GOAL), _PREV_BOTH, {2: 1, 3: 2},
              (_NEW_START, GOAL),
              ((State(0.0, 1.5, 1.45), 0, 1), (State(0.0, -1.5, 1.45), 0, 1)),
              H, {2: 1, 3: 2}),
    SegIdCase("replaced_shorter", "同一拓扑类里来了更短的，connector 位置变了但 alpha 继承。"
              "这就是论文 Fig. 3(b)。",
              STATIC, (START, GOAL), ((_LEFT, 0, 1),), {2: 1},
              (_NEW_START, GOAL), ((State(0.0, 1.2, 1.45), 0, 1),), H, {2: 1}),
    SegIdCase("far_but_same_class", "新桥在 y=2.5，离旧桥（y=1.5）足足 1 m，"
              "但还是从上方绕，所以是同一类，alpha 继承。"
              "判据是 **UVD**，不是坐标接近。写成距离阈值的实现会挂在这里。",
              STATIC, (START, GOAL), ((_LEFT, 0, 1),), {2: 1},
              (_NEW_START, GOAL), ((State(0.0, 2.5, 1.45), 0, 1),), H, {2: 1}),
    SegIdCase("new_class", "上一帧只有左绕，这一帧是右绕。不同绕法，发新号。"
              "这是 Fig. 3(a)。",
              STATIC, (START, GOAL), ((_LEFT, 0, 1),), {2: 1},
              (_NEW_START, GOAL), ((State(0.0, -1.5, 1.45), 0, 1),), H, {2: 10}),
    SegIdCase("no_double_claim", "⭐ 上一帧只有一座桥，这一帧有两座都是左绕。"
              "一个旧 alpha 最多只能被一个新 connector 继承，否则身份就不是身份了。"
              "按节点 id 从小到大试，先到先得。",
              STATIC, (START, GOAL), ((_LEFT, 0, 1),), {2: 1},
              (_NEW_START, GOAL),
              ((State(0.0, 1.5, 1.45), 0, 1), (State(0.0, 1.8, 1.45), 0, 1)),
              H, {2: 1, 3: 10}),
    SegIdCase("guard_pair_changed", "新桥跨的是另一对 guard（中间多了一个新 guard）。"
              "条件 A 不满足，根本不用去做 UVD。",
              STATIC, (START, GOAL), ((_LEFT, 0, 1),), {2: 1},
              (_NEW_START, GOAL, State(0.0, 2.6, 1.45)),
              ((State(-1.5, 1.4, 0.75), 0, 2),), H, {3: 10}),
]


# =====================================================================
# 8. assign_trajectory_ids
# =====================================================================

@dataclass(frozen=True)
class TrajIdCase:
    name: str
    description: str
    paths: tuple[tuple[int, ...], ...]
    segment_ids: dict[int, int]
    prev_map: dict[frozenset[int], int]
    expected: tuple[int, ...]


TRAJID_CASES: list[TrajIdCase] = [
    TrajIdCase("empty", "一条路径都没有。空分支先挡，返回空列表。",
               (), {}, {}, ()),
    TrajIdCase("first_frame", "prev_map 为空，按顺序发新号。",
               ((0, 2, 1), (0, 3, 1)), {2: 1, 3: 2}, {}, (10, 11)),
    TrajIdCase("inherit", "段集合对得上就继承。",
               ((0, 2, 1), (0, 3, 1)), {2: 1, 3: 2},
               {frozenset({1}): 7, frozenset({2}): 8}, (7, 8)),
    TrajIdCase("partial", "一条认识、一条是新的。",
               ((0, 2, 1), (0, 3, 1)), {2: 1, 3: 2},
               {frozenset({1}): 7}, (7, 10)),
    TrajIdCase("multi_segment", "路径穿了两个 connector，键是 {1, 4}。"
               "这就是论文那个 1 -> {1, 4} 的例子。",
               ((0, 2, 4, 3, 1),), {2: 1, 3: 4},
               {frozenset({1, 4}): 1}, (1,)),
    TrajIdCase("order_irrelevant", "同一个段集合，节点顺序不同。用 frozenset 就自动对上，"
               "用 tuple 就会误判成新轨迹。",
               ((0, 3, 4, 2, 1),), {2: 1, 3: 4},
               {frozenset({1, 4}): 1}, (1,)),
    TrajIdCase("no_double_claim", "⭐ 两条路径的段集合完全一样（都只用段 1）。"
               "一个 beta 不能被两条轨迹同时领走，后来的拿新号。",
               ((0, 2, 1), (0, 4, 1)), {2: 1, 4: 1},
               {frozenset({1}): 7}, (7, 10)),
]


# =====================================================================
# 9. select_guidance
# =====================================================================

@dataclass(frozen=True)
class SelectCase:
    name: str
    description: str
    lengths: tuple[float, ...]
    """用直线轨迹造出这些弧长。"""
    trajectory_ids: tuple[int, ...]
    previous_id: int | None
    penalty: float
    expected: int


SELECT_CASES: list[SelectCase] = [
    SelectCase("empty", "这一帧一条通道都没找到（行人堵死了路口）。返回 -1，不要报错。",
               (), (), None, 1.0, -1),
    SelectCase("shortest", "没有惩罚时就是选最短的。",
               (5.0, 3.0, 7.0), (1, 2, 3), None, 0.0, 1),
    SelectCase("first_frame", "previous_id 是 None，没人能免惩罚，等价于纯比长度。",
               (5.0, 3.0, 7.0), (1, 2, 3), None, 10.0, 1),
    SelectCase("previous_wins", "上一帧选的是 3 号（长 5.0），现在 2 号只短了 0.3，"
               "不够以超过 1.0 的惩罚。保持原选择——这就是滞环。",
               (4.7, 5.0), (2, 3), 3, 1.0, 1),
    SelectCase("clearly_better_wins", "同样的惩罚，但新的短了 2.0，**显著**更优，"
               "该换就得换。滞环不是死扣。",
               (3.0, 5.0), (2, 3), 3, 1.0, 0),
    SelectCase("tie_lowest_index", "代价完全相等时取下标小的。结果必须是确定的。",
               (5.0, 5.0), (1, 2), None, 0.0, 0),
    SelectCase("penalty_on_others", "上一帧选的是 2 号（下标 1）。惩罚加在**其他所有**轨迹上，"
               "不是把奖励加在上一条上。两种写法数值等价，但别两边都加了。",
               (4.0, 4.5), (1, 2), 2, 1.0, 1),
]


# =====================================================================
# 10. 多帧闭环（时序断言）
# =====================================================================

@dataclass(frozen=True)
class ClosedScenario:
    name: str
    description: str
    world: World
    warm: bool
    penalty: float
    max_switches: int | None
    """chosen 允许改变的最大次数。None = 只观察不判分。"""
    frames: int = 10
    num_samples: int = 150
    seeds: int = 8


CLOSED_SCENARIOS: list[ClosedScenario] = [
    ClosedScenario("empty_single_class", "只有一条通道，编号全程不该变。"
                   "这里挂掉的话多半是 carry_connector 没在两个调用方都用上："
                   "最短的桥会滑向起点，滑到 t < h 就过期，编号跟着跳。",
                   EMPTY, True, 0.0, 0),
    ClosedScenario("static_warm_consistent", "左绕右绕长度接近，没一致性惩罚就会来回摆。"
                   "复用 + 惩罚都到位的话，10 帧一次都不该换。",
                   STATIC, True, 1.0, 0),
    ClosedScenario("crossing_warm_consistent", "行人横穿，世界每帧都在变，"
                   "但拓扑结构没变，所以选择也不该变。",
                   CROSSING, True, 1.0, 0),
    ClosedScenario("two_gap_ids_stable", "三条走廊。即使不加惩罚，"
                   "只要 alpha/beta 继承对了，选择就是稳的。",
                   TWO_GAP, True, 0.0, 1),
    ClosedScenario("static_cold_baseline", "对照组：不复用、不惩罚，也就是「没有阶段 3」。"
                   "**不判对错**，只是把切换次数打出来让你看。数字越大，"
                   "说明这一阶段做的事越有价值。",
                   STATIC, False, 0.0, None),
]


def find_closed_scenario(name: str) -> ClosedScenario:
    for scenario in CLOSED_SCENARIOS:
        if scenario.name == name:
            return scenario
    known = ", ".join(s.name for s in CLOSED_SCENARIOS)
    raise SystemExit(f"没有这个场景：{name}。可选：{known}")
