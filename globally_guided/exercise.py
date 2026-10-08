"""你的练习文件：只需实现下面三个函数，不要改其他文件。

推荐顺序：segment_collision_free -> connection_valid -> uvd_equivalent。

    python globally_guided/check.py --stage segment
    python globally_guided/check.py --stage connection
    python globally_guided/check.py --stage uvd
    python globally_guided/check.py            # 全跑

卷不动的时候用可视化看一眼：
    python globally_guided/viz.py --case crossing_speed_up_vs_slow_down
"""

from __future__ import annotations

from world import State, Trajectory, World


def segment_collision_free(a: State, b: State, world: World) -> bool:
    """时空线段 a -> b 是否与所有障碍物都不碰撞。

    线段用 u in [0, 1] 参数化：
        p(u) = (1 - u) * a.xy + u * b.xy
        t(u) = (1 - u) * a.t  + u * b.t
    对每个障碍 obs，碰撞当且仅当存在 u 使
        || p(u) - obs.at(t(u)) || < world.inflated_radius(obs)

    这是整个项目的底层原语，后面所有阶段都会反复调用它。值得写扎实。

    提示与陷阱：
    - 不要用采样。障碍匀速 => p(u) - obs.at(t(u)) 对 u 是线性的
      => 距离平方是 u 的二次函数，最小值有闭式解，既快又不会漏检。
    - 最近点可能落在线段**内部**，只检查两个端点会漏判。
    - 二次项系数可能为 0（机器人与障碍相对静止），注意不要除零。
    - 闭式解得到的 u* 要夹到 [0, 1] 里。
    - 退化情况：a == b（零长线段）也要给出正确答案。
    - 没有障碍物时返回 True。

    Returns:
        True 表示无碰撞。
    """
    raise NotImplementedError("请实现 segment_collision_free")


def connection_valid(a: State, b: State, world: World) -> bool:
    """a -> b 这段连接在运动学上可行吗？

    这是论文 Algorithm 1 里 ConnectionInvalid 的前置语义，阶段 2 会直接用到。
    放在这里是为了把「时间不能倒流」这个时空语义在第一阶段就立住。

    要求：
    - 时间严格递增：b.t > a.t。相等也不行（意味着无穷大速度）。
    - 平均速度 || b.xy - a.xy || / (b.t - a.t) <= world.max_velocity。

    注意本函数**不**负责碰撞，只管运动学。

    Returns:
        True 表示可行。
    """
    raise NotImplementedError("请实现 connection_valid")


def uvd_equivalent(tau_1: Trajectory, tau_2: Trajectory, world: World,
                   num_samples: int = 20) -> bool:
    """UVD 判据 H(tau_1, tau_2, O)，论文 Definition 1。

    在 s = 0, 1/n, ..., 1 上取两条轨迹的对应点，若**所有**连线
    tau_1(s) -- tau_2(s) 都无碰撞，则两条轨迹 UVD 等价。

    提示与陷阱：
    - 用 Trajectory.at(s) 或 Trajectory.samples(num_samples) 取点。
    - num_samples 是「区间数」，所以一共有 num_samples + 1 个 s。
    - ⚠️ 最容易错的地方：这根「横档」连线的两端时间是**不同**的
      （tau_1(s).t != tau_2(s).t），它本身就是一条时空线段，
      必须用 segment_collision_free 检查，而不是在某个固定时刻做 2D 判断。
      如果这里做错了，crossing_speed_up_vs_slow_down 那个用例一定会挂。
    - 前提条件（起点相同、终点相同）已经由 check.py 替你断言过了。

    Returns:
        True 表示两条轨迹拓扑等价（即 H = 1）。
    """
    raise NotImplementedError("请实现 uvd_equivalent")
