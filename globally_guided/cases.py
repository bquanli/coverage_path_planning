"""验收用例。这个文件是脚手架，不需要修改。

所有场景的机器人半径 0.2、障碍半径 0.3，所以碰撞阈值 R = 0.5。
用例的数值都刻意留了较大余量，不存在浮点边界问题。
"""

from __future__ import annotations

from dataclasses import dataclass

from world import Obstacle, State, Trajectory, World, obstacle, trajectory

ROBOT_RADIUS = 0.2
OBSTACLE_RADIUS = 0.3
# R = ROBOT_RADIUS + OBSTACLE_RADIUS = 0.5


def make_world(*obstacles: Obstacle, max_velocity: float = 3.0) -> World:
    return World(
        obstacles=tuple(obstacles),
        robot_radius=ROBOT_RADIUS,
        horizon=3.0,
        max_velocity=max_velocity,
    )


# --------------------------------------------------------------------------
# 阶段 segment：时空线段碰撞检测
# --------------------------------------------------------------------------


@dataclass(frozen=True)
class SegmentCase:
    name: str
    description: str
    world: World
    a: State
    b: State
    expected: bool  # True = 无碰撞


_STATIC_AT_ORIGIN = make_world(obstacle(0.0, 0.0))

SEGMENT_CASES: list[SegmentCase] = [
    SegmentCase(
        name="far_away",
        description="静态障碍在原点，线段从上方 2m 处掉头过。",
        world=_STATIC_AT_ORIGIN,
        a=State(-2.0, 2.0, 0.0),
        b=State(2.0, 2.0, 2.0),
        expected=True,
    ),
    SegmentCase(
        name="through_center",
        description="静态障碍在原点，线段正对穿过圆心。",
        world=_STATIC_AT_ORIGIN,
        a=State(-2.0, 0.0, 0.0),
        b=State(2.0, 0.0, 2.0),
        expected=False,
    ),
    SegmentCase(
        name="interior_minimum",
        description=(
            "最近距离 0.4 < R 出现在线段**中间**，而两个端点都距离 2m 以上。"
            "只检查端点的实现会在这里挂。"
        ),
        world=_STATIC_AT_ORIGIN,
        a=State(-2.0, 0.4, 0.0),
        b=State(2.0, 0.4, 2.0),
        expected=False,
    ),
    SegmentCase(
        name="just_clear",
        description="同上，但最近距离 0.6 > R。与 interior_minimum 成对，卡住阈值。",
        world=_STATIC_AT_ORIGIN,
        a=State(-2.0, 0.6, 0.0),
        b=State(2.0, 0.6, 2.0),
        expected=True,
    ),
    SegmentCase(
        name="obstacle_moves_away",
        description=(
            "和 through_center 是**同一条线段**，只是障碍以 3m/s 向上走开了。"
            "在 2D 投影上看是撞的，在时空里并不撞。"
        ),
        world=make_world(obstacle(0.0, 0.0, vy=3.0)),
        a=State(-2.0, 0.0, 0.0),
        b=State(2.0, 0.0, 2.0),
        expected=True,
    ),
    SegmentCase(
        name="relative_rest_collision",
        description=(
            "障碍与机器人速度完全相同，相对静止、始终相距 0.4 < R。"
            "此时距离平方的二次项系数为 0，闭式解要小心除零。"
        ),
        world=make_world(obstacle(-2.0, 0.4, vx=2.0)),
        a=State(-2.0, 0.0, 0.0),
        b=State(2.0, 0.0, 2.0),
        expected=False,
    ),
    SegmentCase(
        name="relative_rest_free",
        description="同上的退化形式，但相距 0.8 > R。",
        world=make_world(obstacle(-2.0, 0.8, vx=2.0)),
        a=State(-2.0, 0.0, 0.0),
        b=State(2.0, 0.0, 2.0),
        expected=True,
    ),
    SegmentCase(
        name="start_inside",
        description="线段起点恰好在圆心上，最小值出现在 u = 0 的边界。",
        world=_STATIC_AT_ORIGIN,
        a=State(0.0, 0.0, 0.0),
        b=State(3.0, 0.0, 1.5),
        expected=False,
    ),
    SegmentCase(
        name="zero_length_free",
        description="零长线段（a == b），位于远处。",
        world=_STATIC_AT_ORIGIN,
        a=State(2.0, 2.0, 1.0),
        b=State(2.0, 2.0, 1.0),
        expected=True,
    ),
    SegmentCase(
        name="zero_length_collision",
        description="零长线段，但落在障碍里。",
        world=_STATIC_AT_ORIGIN,
        a=State(0.1, 0.0, 1.0),
        b=State(0.1, 0.0, 1.0),
        expected=False,
    ),
    SegmentCase(
        name="no_obstacles",
        description="空世界。",
        world=make_world(),
        a=State(-2.0, 0.0, 0.0),
        b=State(2.0, 0.0, 2.0),
        expected=True,
    ),
    SegmentCase(
        name="second_obstacle_blocks",
        description="第一个障碍很远、第二个挡路。忘了遍历全部障碍就会挂。",
        world=make_world(obstacle(0.0, 5.0), obstacle(0.0, 0.0)),
        a=State(-2.0, 0.0, 0.0),
        b=State(2.0, 0.0, 2.0),
        expected=False,
    ),
]


# --------------------------------------------------------------------------
# 阶段 connection：运动学可行性
# --------------------------------------------------------------------------


@dataclass(frozen=True)
class ConnectionCase:
    name: str
    description: str
    world: World
    a: State
    b: State
    expected: bool


_EMPTY = make_world(max_velocity=3.0)

CONNECTION_CASES: list[ConnectionCase] = [
    ConnectionCase(
        name="forward_slow",
        description="1 秒走 1 米，轻松满足限速。",
        world=_EMPTY, a=State(0.0, 0.0, 0.0), b=State(1.0, 0.0, 1.0),
        expected=True,
    ),
    ConnectionCase(
        name="stand_still",
        description="原地等待 1 秒，速度 0。",
        world=_EMPTY, a=State(0.0, 0.0, 0.0), b=State(0.0, 0.0, 1.0),
        expected=True,
    ),
    ConnectionCase(
        name="backward_in_time",
        description="终点时间早于起点。时间不能倒流。",
        world=_EMPTY, a=State(0.0, 0.0, 1.0), b=State(1.0, 0.0, 0.0),
        expected=False,
    ),
    ConnectionCase(
        name="zero_duration",
        description="两端同时刻但不同位置，意味着无穷大速度。",
        world=_EMPTY, a=State(0.0, 0.0, 1.0), b=State(1.0, 0.0, 1.0),
        expected=False,
    ),
    ConnectionCase(
        name="zero_duration_same_point",
        description="退化到一个点。按「时间严格递增」的规则仍然不合法。",
        world=_EMPTY, a=State(0.0, 0.0, 1.0), b=State(0.0, 0.0, 1.0),
        expected=False,
    ),
    ConnectionCase(
        name="too_fast",
        description="平均速度 4 m/s，超过了 max_velocity = 3。",
        world=_EMPTY, a=State(0.0, 0.0, 0.0), b=State(4.0, 0.0, 1.0),
        expected=False,
    ),
    ConnectionCase(
        name="just_under_limit",
        description="2.97 m/s。",
        world=_EMPTY, a=State(0.0, 0.0, 0.0), b=State(2.97, 0.0, 1.0),
        expected=True,
    ),
    ConnectionCase(
        name="just_over_limit",
        description="3.03 m/s。注意是欧氏距离而不是单轴差。",
        world=_EMPTY, a=State(0.0, 0.0, 0.0), b=State(0.0, 3.03, 1.0),
        expected=False,
    ),
]


# --------------------------------------------------------------------------
# 阶段 uvd：拓扑等价判据
# --------------------------------------------------------------------------


@dataclass(frozen=True)
class UvdCase:
    name: str
    description: str
    world: World
    tau_1: Trajectory
    tau_2: Trajectory
    expected: bool  # True = UVD 等价


# 所有 UVD 用例共用起点 (-3, 0, 0) 和终点 (3, 0, 3)。
LEFT = trajectory([(-3.0, 0.0, 0.0), (0.0, 1.5, 1.5), (3.0, 0.0, 3.0)], name="left")
RIGHT = trajectory([(-3.0, 0.0, 0.0), (0.0, -1.5, 1.5), (3.0, 0.0, 3.0)], name="right")
FAR_LEFT = trajectory(
    [(-3.0, 0.0, 0.0), (0.0, 2.5, 1.5), (3.0, 0.0, 3.0)], name="far_left"
)
# xy 投影完全相同（都是 y = 0 的直线），只是通过中点的时刻不同。
FAST = trajectory([(-3.0, 0.0, 0.0), (0.0, 0.0, 0.7), (3.0, 0.0, 3.0)], name="fast")
SLOW = trajectory([(-3.0, 0.0, 0.0), (0.0, 0.0, 2.3), (3.0, 0.0, 3.0)], name="slow")

UVD_CASES: list[UvdCase] = [
    UvdCase(
        name="static_left_vs_right",
        description="静态障碍在原点，一条左绕一条右绕。经典的两个拓扑类。",
        world=make_world(obstacle(0.0, 0.0)),
        tau_1=LEFT, tau_2=RIGHT,
        expected=False,
    ),
    UvdCase(
        name="static_both_left",
        description="两条都从左边绕，只是绕得一个宽一个窄。形状不同不等于拓扑不同。",
        world=make_world(obstacle(0.0, 0.0)),
        tau_1=LEFT, tau_2=FAR_LEFT,
        expected=True,
    ),
    UvdCase(
        name="crossing_speed_up_vs_slow_down",
        description=(
            "横穿的行人；一条加速抢行、一条减速让行。"
            "两条轨迹在 xy 平面上的投影**完全重合**，只有时间不同。"
            "这是整个阶段的试金石：只要哪里退化成了 2D 判断，这条必挂。"
        ),
        world=make_world(obstacle(0.0, -3.75, vy=2.5)),
        tau_1=FAST, tau_2=SLOW,
        expected=False,
    ),
    UvdCase(
        name="obstacle_already_gone",
        description=(
            "轨迹与 static_left_vs_right **完全一样**，唯一区别是障碍以 5m/s 飞走了。"
            "把障碍沿时间扫成一块静态区域来判断的实现会在这里挂。"
        ),
        world=make_world(obstacle(0.0, 0.0, vy=5.0)),
        tau_1=LEFT, tau_2=RIGHT,
        expected=True,
    ),
    UvdCase(
        name="identical",
        description="轨迹和自己比。所有横档都是零长线段。",
        world=make_world(obstacle(0.0, 0.0)),
        tau_1=LEFT, tau_2=LEFT,
        expected=True,
    ),
    UvdCase(
        name="empty_world",
        description="没有障碍时，任意两条轨迹都等价。形状差异本身不产生拓扑差异。",
        world=make_world(),
        tau_1=LEFT, tau_2=RIGHT,
        expected=True,
    ),
]


ALL_CASES = {
    "segment": SEGMENT_CASES,
    "connection": CONNECTION_CASES,
    "uvd": UVD_CASES,
}


def find_uvd_case(name: str) -> UvdCase:
    for case in UVD_CASES:
        if case.name == name:
            return case
    available = ", ".join(case.name for case in UVD_CASES)
    raise SystemExit(f"找不到 UVD 用例 {name!r}。可选：{available}")
