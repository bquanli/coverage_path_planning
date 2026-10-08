# 阶段 1：时空表示与 UVD 拓扑判据

复现 *Globally Guided Trajectory Planning in Dynamic Environments*（arXiv:2303.07751）的第一个阶段，
对应论文第 III-A 节和 Definition 1，也就是 `../论文详解-GloballyGuidedTrajectoryPlanning.md`
里的 4.1–4.2。

这一阶段要建立的全部能力只有一句话：**在 (x, y, t) 状态空间里，判断两条轨迹是不是
「本质上同一种绕行方式」。** PRM、样条优化、MPCC 都是后面的事。

## 文件

| 文件 | 说明 |
|---|---|
| `exercise.py` | ⭐ **你要写的文件**，三个函数 |
| `world.py` | `State` / `Obstacle` / `World` / `Trajectory`，脚手架 |
| `cases.py` | 26 个验收用例，脚手架 |
| `check.py` | 判卷 |
| `viz.py` | rerun 三维可视化 |

## 怎么跑

下面用 `.venv/bin/python`；`uv run python ...` 等价。

```bash
.venv/bin/python globally_guided/check.py --stage segment
.venv/bin/python globally_guided/check.py --stage connection
.venv/bin/python globally_guided/check.py --stage uvd
.venv/bin/python globally_guided/check.py              # 全跑
.venv/bin/python globally_guided/check.py --stage uvd -v   # 失败时列出每根横档

.venv/bin/python globally_guided/viz.py --list
.venv/bin/python globally_guided/viz.py --case crossing_speed_up_vs_slow_down
.venv/bin/python globally_guided/viz.py --case static_left_vs_right --save /tmp/uvd.rrd  # 无头
```

## 要实现的三个函数

按这个顺序写，每一个都有独立的 `--stage` 可以验证。

### 1. `segment_collision_free(a, b, world)`

时空线段 `a -> b` 是否与所有障碍都不碰撞。

**这是整个项目的底层原语。** 阶段 2 的 Visibility-PRM 要用它做可见性检查，
阶段 3 的节点重引入要用它重新校验旧边，阶段 4 的样条也要用它抽检。
它会被调用几十万次，所以值得现在就写对、写快。

核心思路：障碍匀速，所以 `p(u) - o(t(u))` 对 `u` 是线性的，距离平方就是 `u` 的一个
二次函数，最小值有闭式解。**不要用采样。**

### 2. `connection_valid(a, b, world)`

时间严格递增，且平均速度不超限。十几行的事，但它把「时间不能倒流」这条语义立在了第一阶段。
这就是论文 Algorithm 1 第 13、15 行的 `ConnectionInvalid`，阶段 2 直接拿来用。

### 3. `uvd_equivalent(tau_1, tau_2, world, num_samples)`

论文 Definition 1。在 `s = 0, 1/n, ..., 1` 上取两条轨迹的对应点，若所有连线都无碰撞则等价。

⚠️ **这里有一个坐坑点。** 连线的两端时间是不同的（`tau_1(s).t != tau_2(s).t`），
所以它本身就是一条**时空**线段，必须用 `segment_collision_free` 来检查，
而不是在某个固定时刻做二维线段检测。想明白这一点，就说明你真的理解了「状态空间」。

## 用例设计说明

所有场景里机器人半径 0.2、障碍半径 0.3，所以碰撞阈值 `R = 0.5`。
数值都留了较大余量，不存在浮点边界问题。几组用例是成对设计的：

- `interior_minimum` / `just_clear`：最近距离 0.4 vs 0.6，卡死阈值。
  前者的两个端点都在 2m 以外，**只检查端点的实现会在这里挂**。
- `through_center` / `obstacle_moves_away`：完全同一条线段，只是障碍动不动。
- `relative_rest_collision`：机器人与障碍速度相同，二次项系数为 0。**不防除零会在这里崩。**
- `static_left_vs_right` / `obstacle_already_gone`：轨迹一模一样，只是障碍一个不动、
  一个以 5m/s 飞走，答案相反。把障碍沿时间扫成静态区域的实现会在后者挂。
- `crossing_speed_up_vs_slow_down`：两条轨迹的 xy 投影**完全重合**，只有通过时刻不同。
  这是整个阶段的试金石：哪怕只有一处退化成了 2D 判断，它就一定挂。

## 一个值得玩一下的现象

全部跑通之后，试试这个：

```bash
.venv/bin/python globally_guided/check.py --stage uvd --num-samples 3
```

你会看到 `static_left_vs_right` 和 `crossing_speed_up_vs_slow_down` 被判成了「等价」。

**这不是 bug。** UVD 的定义要求对**所有** `s` 检查，而实现上只能在有限个 `s` 上抽检
（论文原话：*"In practice, we check collisions for s at discrete intervals"*）。
采样太稀时，横档会从障碍两侧跨过去而漏判。用 `viz.py --num-samples 3` 看一眼就很清楚了。

这个漏判的代价是「把两个本该分开的拓扑类当成一个」，在阶段 2 会直接表现为
「PRM 少找到一种驾驶行为」。这篇论文选 UVD 而不是同调不变量，换来的就是速度，
付出的也就是这个精度。默认的 `num_samples=20` 对本阶段所有用例都是稳定的
（实测 n >= 8 就全部稳定）。

## 两个设计决策

脚手架替你做了两个选择，它们在论文里都没写，但实现时跑不掉：

**轨迹的 `s` 怎么参数化。** `Trajectory.at(s)` 用的是累积弧长均分。
换成按索引均分或按时间均分，会得到不同的对应点，也就可能得到不同的 UVD 判定。
Zhou 等人的原实现（文献 19）是把每条路径均分成 N 段。

**时间轴的尺度。** `World.time_scale` 把秒折算成米。碰撞检测不受它影响（逐时刻判断），
所以本阶段用不到；但到了阶段 2，「路径长度」「采样邻域」都会用到它。
现在显式写出来，比到时候再回头改强。

## 做完之后

全部 26 个用例通过后，再去对照官方实现
[tud-amr/guidance_planner](https://github.com/tud-amr/guidance_planner)，
重点看它的 `graph_search.cpp` / `paths.cpp` 怎么做 UVD 判定。
**先自己写完再看。**

下一阶段：Visibility-PRM 建图 + DFS 枚举拓扑类（论文 Algorithm 1）。
guard / connector 的划分、「恰好看见 2 个 guard」的规则、以及用本阶段的 UVD 做去重。
