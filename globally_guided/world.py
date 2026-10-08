"""时空世界的基础数据结构。这个文件是脚手架，不需要修改。

===============================================================
心智模型：World 不是「地图」，是「碰撞谓词的参数包」
===============================================================

读这个文件时最容易产生的困惑是：怎么没有栅格、没有车道、没有地图图层？
答案是：**一个世界该怎么表示，完全由算法会向它提哪些问题决定。**

本阶段（以及后面的 PRM、样条优化）向世界提的问题，自始至终只有一个：

    「从状态 a 直线走到状态 b，这条时空线段会不会撞上什么？」

World 存在的唯一目的，就是让这个问题能被回答。凡是回答它用不上的东西
（栅格、道路拓扑、地面材质、传感器模型……）就都不该出现在这里。反过来，
回答它必需的东西只有两样：**障碍物在任意时刻占据哪里**，以及
**机器人自己有多大、能走多快**。World 里正好就只有这两样。

至于栅格：栅格是碰撞检测的一种**离散化实现**，不是世界本身。在这里它反而
是劣势：动态障碍要用栅格表示，就得存一摈时间切片，又占内存又粗；而
「圆盘 + 匀速直线」这种解析表示，占据查询是闭式的、精度无限、时间维天然连续。
至于车道、道路、任务语义：那些影响的是起点终点怎么定、代价函数怎么写，
不影响「撞不撞」，所以属于更上层，不进 World。

---------------------------------------------------------------
状态空间长什么样
---------------------------------------------------------------

状态空间 X = R^2 x [0, T]：底面是二维工作空间，竖轴是时间。一个状态点写作
State(x, y, t)，读作「t 时刻人在 (x, y)」。在这个盒子里：

- 一个静止的圆盘障碍  = 一根**竖直**的圆柱管；
- 一个匀速运动的行人  = 一根**斜**的圆柱管，斜率就是它的速度；
- 机器人的一条轨迹    = 从底面往上走的一条曲线，**只能向上**（时间不能倒流），
  而且倾斜程度有上限（速度不能超限）。

论文的全部洞见就藏在这张图里：这些管子把盒子凿出了若干个互不相通的「通道」。
从左边绕、从右边绕、抢在行人前面过、让行人先走——是四条本质不同的通道，
而不是同一个问题的四个局部最优。梯度下降没法从一个通道跳到另一个通道，
所以才需要一个全局层先把通道枚举出来。

---------------------------------------------------------------
World 里的四个字段，分属三类不同的信息
---------------------------------------------------------------

  环境（外生的，我不能改）      obstacles
  机器人（自身的，决定什么可行）  robot_radius, max_velocity
  规划口径（我定的，算法约定）    horizon, time_scale

把这三类放进同一个 dataclass，是因为它们共同回答那一个谓词；但理解的时候
要分开看：换一个场景只换第一类，换一台车只换第二类，调参只换第三类。
本阶段的 26 个用例全部只在换第一类。

---------------------------------------------------------------
机器人只用两个数字描述，这不是偋懒，是分层
---------------------------------------------------------------

机器人没有朝向、没有转弯半径、没有轮距、没有加速度上限，只有半径和最大速度。
原因是全局引导层的职责就只是「决定走哪个通道」，而不是「输出能执行的指令」。
运动学可行性被故意推到后面：阶段 4 的三次样条把折线磨平滑，阶段 5 的 MPCC
带着真实动力学去跟踪它。全局层一开始就背上完整动力学，慢且没必要。

- robot_radius：圆盘建模 => 姿态无关 => 碰撞检测只是点到点的距离比较。
  这是一个巨大的简化：矩形机器人要算两个凸多边形的最近距离，还得考虑转角。
- max_velocity：在 (x, y, t) 空间里，速度上限就是轨迹的**斜率上限**，
  几何上是一个从当前点往上张开的圆锥（光锥）。这是时空表示下最自然、也最弱的
  一条动力学约束，它保证 PRM 连出来的边「时间上可达」。加速度上限没进来，
  是因为折线在拐点不可导，谈加速度要等到阶段 4 有了样条才有意义。

---------------------------------------------------------------
一个常见误解：「障碍有多大」不是障碍自己的属性
---------------------------------------------------------------

碰撞阈值 R = r_robot + r_obs 是**一对**（机器人, 障碍）的属性，所以
inflated_radius() 写在 World 上而不是 Obstacle 上。这就是 Minkowski 和：
把机器人缩成质点、把障碍膨胀 r_robot，两个圆盘的相交问题就变成了
点跟一个圆的关系问题。这个变换是后面所有闭式解的前提。
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np
from numpy.typing import NDArray

Vec2 = NDArray[np.float64]


@dataclass(frozen=True)
class State:
    """状态空间中的一个点：「t 时刻我在 (x, y)」。

    注意 t 是状态的一部分，而不是「轨迹的第几个点」这种索引。同一个 (x, y)
    配上不同的 t 是两个完全不同的状态：一个可能安全，另一个正好撞上行人。
    本阶段的试金石用例 crossing_speed_up_vs_slow_down 考的就是这一点。

    frozen=True：值语义、可哈希。阶段 2 的 PRM 会把状态当字典键和集合元素用，
    而且节点会被多条边共享，不可变能省掉一类很难调的 bug。
    """

    x: float
    y: float
    t: float

    @property
    def xy(self) -> Vec2:
        """只取空间分量。做向量运算时用，**不要**把 t 也塞进欧氏距离里：
        米和秒不同量纲，要混算必须先过 time_scale（见 World.time_scale）。
        """
        return np.array([self.x, self.y], dtype=float)

    def __repr__(self) -> str:
        return f"State({self.x:.3f}, {self.y:.3f}, t={self.t:.3f})"


@dataclass(frozen=True)
class Obstacle:
    """匀速运动的圆盘障碍。在状态空间里它是一根斜圆柱管。

    对外的全部接口其实只有 at(t)：「告诉我任意时刻你在哪」。position/velocity
    只是 at() 的一种实现。

    为什么假设匀速：这是一个性能决策，不是物理假设。机器人沿直线走、障碍也
    沿直线走 => 相对位置对线段参数 u 是线性的 => 距离平方是 u 的二次函数
    => 最近点有**闭式解**，不用采样。这条性质是整个规划器跑得动的根源（碰撞
    检测会被调几十万次），也是论文的主要局限之一（原文用匀速 + 卡尔曼预测，
    行人突然变向就意味着管子形状错了，拓扑分析的前提随之动摇）。

    将来若要支持变速轨迹，at(t) 的接口不用改，但闭式解就没了。

    顺便：静态障碍（墙、货架）在这套表示里就是 velocity = (0, 0) 的障碍，
    不需要单独的数据结构。这也是「不要栅格」的另一个理由。
    """

    position: Vec2   # t = 0 时的圆心，单位 m
    velocity: Vec2   # 恒定速度，单位 m/s；在状态空间里就是这根管子的倾斜方向
    radius: float = 0.3   # 障碍自身半径；实际用的阈值请走 World.inflated_radius()

    def at(self, t: float) -> Vec2:
        """t 时刻的圆心位置。这就是「世界对外暴露的占据查询」。"""
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
    """一个场景：障碍集合 + 机器人参数 + 规划口径。

    把它当成「一次规划所需的全部上下文」：有了它，任意一条时空线段的死活
    就是确定的。没有全局变量，没有隐藏状态——这是它能被 26 个用例并行测试、
    也能被 PRM 重复调用几十万次的前提。frozen=True 是在结构上把这件事钉死。

    字段详解（按「环境 / 机器人 / 规划口径」三类理解）：

    obstacles（环境）
        场景的全部内容。换一个场景就是换这一项。空元祖是合法的（开阔场地，
        此时任何线段都无碰撞）。注意它是 tuple
        而不是 list，配合 frozen=True 一起保证场景不可变。

    robot_radius（机器人）
        机器人外接圆半径。它不单独出现在任何公式里，总是以 r + r_obs 的形式
        出现，所以请用 inflated_radius() 而不要自己相加。

    max_velocity（机器人）
        速度上限，单位 m/s。它的唯一用处是判定一条边「时间上走不走得到」，
        也就是 connection_valid 里的 dist / dt <= max_velocity。注意它约束的是
        **平均**速度：折线段内部是匀速的，所以平均即瞬时。
        论文里这个量是 v̄ = 2 m/s，本阶段用例用 3 m/s。

    horizon（规划口径）
        状态空间 R^2 x [0, T] 里的 T，也就是那个盒子的高度。规划只做有限时域，
        是因为行人预测只在有限时域内可信；T 之外的事不属于这个 World。
        本阶段不会去截断轨迹（用例自己保证落在范围内），阶段 2 采样时会用到。

    time_scale（规划口径）
        把秒折算成米的系数。这是整个文件里最容易被忽略、但概念上最重要的一项：
        (x, y, t) 这三个坐标的量纲不同（米、米、秒），所以状态空间里根本
        **没有天然的距离**。一旦你问「这两个状态有多远」「这条路有多长」
        「邻域半径取多少」，你就已经在隐式地做单位换算了。time_scale 把这个
        换算显式写出来：它定义了状态空间的度量。

        碰撞检测不受它影响（因为碰撞是在**同一时刻**比二维距离，没有跨维度
        混算），所以阶段 1 用不到；到了阶段 2，路径长度、可见性、采样邻域
        都会用到。现在就把它写出来，比到时候再回头改强。
    """

    obstacles: tuple[Obstacle, ...] = ()
    robot_radius: float = 0.2
    horizon: float = 3.0
    max_velocity: float = 3.0
    time_scale: float = 1.0

    def inflated_radius(self, obs: Obstacle) -> float:
        """碰撞阈值 R = r + r_obs。

        写在 World 上是有意的：这是「这台机器人跟这个障碍」的关系，不属于
        任一方。有了它之后，机器人就可以被当成质点，所有碰撞问题都变成
        「点到圆心的距离是否小于 R」。
        """
        return self.robot_radius + obs.radius


# 这个是属于机器人的轨迹，并不是障碍物的轨迹！
@dataclass
class Trajectory:
    """时空折线，按（带 time_scale 的）累积弧长用 s in [0, 1] 参数化。

    为什么要多一层 s，而不是直接拿 t 当参数？因为 UVD（论文 Definition 1）需要
    把两条轨迹的「对应点」连起来比较，而「对应」是个拓扑概念、不是时间概念：
    两条轨迹的时间跨度可以不一样，按 t 对齐根本对不上。s 把两条轨迹都归一化成
    「从头走到尾的进度」，进度相同的两点就是一对对应点。

    由此得出本阶段最容易踩的坑：tau_1(s) 和 tau_2(s) 的**时间是不同的**，
    所以连接它们的那根「横档」本身就是一条时空线段，必须用
    segment_collision_free 来查，而不能在某个固定时刻做二维线段检测。

    参数化方式是一个有意识的设计选择：按索引 / 按弧长 / 按时间均分，会给出
    不同的对应点，也就可能给出不同的 UVD 判定结果。这里用累积弧长，详见 README。
    """

    states: list[State]
    time_scale: float = 1.0   # 只影响弧长怎么算，进而影响 s 怎么切分
    name: str = ""
    _cumulative: list[float] = field(default_factory=list, repr=False)

    def __post_init__(self) -> None:
        if len(self.states) < 2:
            raise ValueError("轨迹至少需要两个状态点")
        # 预计算每个顶点处的累积弧长。注意 dt 乘了 time_scale：这正是「把秒
        # 换算成米」的地方，否则这个 sqrt 里三个量纲不一的数相加没有意义。
        cumulative = [0.0]
        for a, b in zip(self.states, self.states[1:]):
            dx = b.x - a.x
            dy = b.y - a.y
            dt = (b.t - a.t) * self.time_scale
            cumulative.append(cumulative[-1] + math.sqrt(dx * dx + dy * dy + dt * dt))
        self._cumulative = cumulative

    @property
    def length(self) -> float:
        """状态空间中的总弧长（含时间维），不是地面上走的路程。"""
        return self._cumulative[-1]

    @property
    def start(self) -> State:
        return self.states[0]

    @property
    def goal(self) -> State:
        return self.states[-1]

    def at(self, s: float) -> State:
        """按累积弧长取 s in [0, 1] 对应的状态点。

        返回的是完整的 State，包括 t。写 UVD 时别只拿 .xy。
        """
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
        """在 s = 0, 1/count, ..., 1 上取 count + 1 个点。

        count 就是 UVD 的离散化精度。取得太小会把两个本该分开的拓扑类当成一个
        （README 里那个 --num-samples 3 的现象）。这是 UVD 用精度换速度的地方。
        """
        return [self.at(i / count) for i in range(count + 1)]


def trajectory(points: list[tuple[float, float, float]], time_scale: float = 1.0,
               name: str = "") -> Trajectory:
    """从 (x, y, t) 三元组列表构造 Trajectory 的便捷函数。"""
    return Trajectory(
        states=[State(x, y, t) for x, y, t in points],
        time_scale=time_scale,
        name=name,
    )
