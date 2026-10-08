"""时空世界的基础数据结构。这个文件是脚手架，不需要修改。

约定：
- 状态空间 X = R^2 x [0, T]，一个状态点写作 State(x, y, t)。
- 障碍物做匀速直线运动，o(t) = position + velocity * t。
- 机器人和障碍都用圆盘建模。碰撞判据统一写成
      || p - o(t) || < r + r_obs
  也就是把机器人缩成质点、把障碍半径膨胀 r。
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np
from numpy.typing import NDArray

Vec2 = NDArray[np.float64]


@dataclass(frozen=True)
class State:
    """状态空间中的一个点。"""

    x: float
    y: float
    t: float

    @property
    def xy(self) -> Vec2:
        return np.array([self.x, self.y], dtype=float)

    def __repr__(self) -> str:
        return f"State({self.x:.3f}, {self.y:.3f}, t={self.t:.3f})"


@dataclass(frozen=True)
class Obstacle:
    """匀速运动的圆盘障碍。"""

    position: Vec2
    velocity: Vec2
    radius: float = 0.3

    def at(self, t: float) -> Vec2:
        """t 时刻的圆心位置。"""
        return np.asarray(self.position, dtype=float) + np.asarray(
            self.velocity, dtype=float
        ) * float(t)


def obstacle(px: float, py: float, vx: float = 0.0, vy: float = 0.0,
             radius: float = 0.3) -> Obstacle:
    """构造 Obstacle 的便捷函数。"""
    return Obstacle(
        position=np.array([px, py], dtype=float),
        velocity=np.array([vx, vy], dtype=float),
        radius=radius,
    )


@dataclass(frozen=True)
class World:
    """一个场景：障碍集合 + 机器人参数。

    time_scale: 把秒折算成米的系数。碰撞检测不受它影响（逐时刻判断），
        但「路径长度」「可见性」「采样邻域」都会受影响，所以显式地写出来。
        阶段 1 用不到，阶段 2 调 PRM 采样时会用到。
    """

    obstacles: tuple[Obstacle, ...] = ()
    robot_radius: float = 0.2
    horizon: float = 3.0
    max_velocity: float = 3.0
    time_scale: float = 1.0

    def inflated_radius(self, obs: Obstacle) -> float:
        """碰撞阈值 R = r + r_obs。"""
        return self.robot_radius + obs.radius


@dataclass
class Trajectory:
    """时空折线，按（带 time_scale 的）累积弧长用 s in [0, 1] 参数化。

    这里的参数化方式是一个有意识的设计选择（按索引 / 按弧长 / 按时间
    会给出不同的 UVD 判定结果），详见 README。
    """

    states: list[State]
    time_scale: float = 1.0
    name: str = ""
    _cumulative: list[float] = field(default_factory=list, repr=False)

    def __post_init__(self) -> None:
        if len(self.states) < 2:
            raise ValueError("轨迹至少需要两个状态点")
        cumulative = [0.0]
        for a, b in zip(self.states, self.states[1:]):
            dx = b.x - a.x
            dy = b.y - a.y
            dt = (b.t - a.t) * self.time_scale
            cumulative.append(cumulative[-1] + math.sqrt(dx * dx + dy * dy + dt * dt))
        self._cumulative = cumulative

    @property
    def length(self) -> float:
        return self._cumulative[-1]

    @property
    def start(self) -> State:
        return self.states[0]

    @property
    def goal(self) -> State:
        return self.states[-1]

    def at(self, s: float) -> State:
        """按累积弧长取 s in [0, 1] 对应的状态点。"""
        s = min(max(float(s), 0.0), 1.0)
        if self.length == 0.0:
            return self.states[0]
        target = s * self.length
        for index in range(len(self.states) - 1):
            lo, hi = self._cumulative[index], self._cumulative[index + 1]
            if target <= hi or index == len(self.states) - 2:
                span = hi - lo
                ratio = 0.0 if span == 0.0 else (target - lo) / span
                a, b = self.states[index], self.states[index + 1]
                return State(
                    x=a.x + (b.x - a.x) * ratio,
                    y=a.y + (b.y - a.y) * ratio,
                    t=a.t + (b.t - a.t) * ratio,
                )
        return self.states[-1]

    def samples(self, count: int) -> list[State]:
        """在 s = 0, 1/count, ..., 1 上取 count + 1 个点。"""
        return [self.at(i / count) for i in range(count + 1)]


def trajectory(points: list[tuple[float, float, float]], time_scale: float = 1.0,
               name: str = "") -> Trajectory:
    """从 (x, y, t) 三元组列表构造 Trajectory 的便捷函数。"""
    return Trajectory(
        states=[State(x, y, t) for x, y, t in points],
        time_scale=time_scale,
        name=name,
    )
