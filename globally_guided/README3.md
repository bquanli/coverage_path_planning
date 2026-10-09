# 阶段 3：拓扑信息的跨帧传播

对应论文第 III-C 节和 **Algorithm 1 第 4-6 行**，也就是
`../论文详解-GloballyGuidedTrajectoryPlanning.md` 里的 4.4。

一句话概括：**把引导层从一个无状态的单帧函数，改造成一个有记忆的、跑在闭环里的东西。**

三个阶段回答的是三个递进的问题：

| | 问题 | 性质 |
|---|---|---|
| 阶段 1 | 给两条轨迹，同类吗？ | 纯函数，无状态 |
| 阶段 2 | 给一个世界，列出所有类的代表 | 随机算法，单帧 |
| 阶段 3 | 给一串世界快照，让类的编号跨帧稳定 | 有状态，多帧 |

## 文件

| 文件 | 说明 |
|---|---|
| `exercise3.py` | ⭐ **你要写的文件**，九个函数 |
| `tracking.py` | `IdAllocator` / `Frame` / `run_frames` 等跨帧记账脚手架 |
| `cases3.py` | 验收用例，脚手架 |
| `check3.py` | 判卷 |

前两阶段的 `world.py` / `graph.py` / `exercise.py` / `exercise2.py` 继续用，一行不改。

## 怎么跑

```bash
.venv/bin/python globally_guided/check3.py --stage advance    # advance_world
.venv/bin/python globally_guided/check3.py --stage shift      # shift_state
.venv/bin/python globally_guided/check3.py --stage carry      # carry_connector
.venv/bin/python globally_guided/check3.py --stage reintro    # reintroduce_states
.venv/bin/python globally_guided/check3.py --stage warm       # build_prm_warm
.venv/bin/python globally_guided/check3.py --stage guardmap   # match_guards
.venv/bin/python globally_guided/check3.py --stage segid      # assign_segment_ids
.venv/bin/python globally_guided/check3.py --stage trajid     # assign_trajectory_ids
.venv/bin/python globally_guided/check3.py --stage select     # select_guidance
.venv/bin/python globally_guided/check3.py --stage closed     # 多帧闭环
.venv/bin/python globally_guided/check3.py                    # 全跑
.venv/bin/python globally_guided/check3.py -v                 # 失败时的详细诊断
```

总共 54 个计分用例，另有 1 个只打印不判对错的观察场景。建议按声明顺序写：
前三个（`advance` / `shift` / `carry`）是热身，`reintro` 和 `warm` 是「复用」这一半，
`guardmap` / `segid` / `trajid` 是「身份」那一半，`select` 是收尾。
`closed` 是端到端的，前面全绿了再去碰。

## 新出现的一类测试：时序断言

这是阶段 3 和前两阶段最大的区别。

- 阶段 1 的错误：**测试立刻变红**。函数是纯谓词，错了就是数学错了。
- 阶段 2 的错误：**程序照跑，少一个通道**。要靠四条图不变量和 `stats()` 来抓。
- 阶段 3 的错误：**每一帧单独看全对，连着跑起来机器人在拖。**

最后这一类用单帧断言根本抓不到：图满足四条不变量、拓扑类数也对、每帧选的也确实是当前最短的。
所以 `--stage closed` 改成了另一种形式：连续跑 10 帧 x 8 个种子，
断言 `chosen` 这个编号全程不变。

里面有一个叫 `static_cold_baseline` 的场景，**不判对错**，只把切换次数打出来。
它关掉了复用和一致性惩罚，也就是「没有阶段 3」的样子。参考数字：
8 个种子的切换次数是 `[0, 7, 4, 3, 0, 6, 4, 5]`，而开了之后全是 0。
那个 7 就是论文要解决的问题：10 帧里换了 7 次主意。

## 两个必须先想清楚的点

**符号。** 两帧之间过了 h 秒，这件事在两个地方体现，而且符号相反：
障碍物是 `p -> p + v*h`（它真的往前走了），节点是 `t -> t - h`（它离现在更近了）。
两者说的是同一件事：时间原点往前挪了 h。每一帧都有自己的 t = 0。

**start 和 goal 的坐标讲定会变。** 机器人走了，goal 被推回了新窗口的末端。
所以 `match_guards` 里 `0 -> 0`、`1 -> 1` 必须硬编码：它们跨帧就是同一个东西
（机器人和目的地），和坐标无关。按坐标匹配的话，所有段每帧都会拿新 ID，
整套机制归零——而且单帧看不出任何异常。

## 三个坑

**过期判定是 `t - h <= 0` 而不是 `< 0`。** `t - h == 0` 意味着这个航点恰好就是「现在」，
而那个位置已经被机器人自己占了。留着它只是白占地方，因为它和 start 的时间差为 0，
`edge_feasible` 一定拒绝。

**复用的节点必须 guard 在前、connector 在后。** 这条没有任何计算，纯粹是顺序问题，
但弄错了代价很大：connector 先进图时它的两个 guard 还不在，`|L|` 会是 0 或 1，
于是它要么被误立为 guard（整张图的结构都变了），要么被丢掉（直接少一个拓扑类）。
两种都不报错。

**`carry_connector` 有两个调用方，两边必须用同一条规则。** 只在 `reintroduce_states`
里重采样、在 `assign_segment_ids` 里用 `shift_state`，结果是节点活下来了、
身份没跟上：复用生效但编号照跳。图看起来完全正常，只有 `closed` 里的
`empty_single_class` 会报错。

## 怎么调试

阶段 1 靠打印中间量，阶段 2 靠 `graph.stats()` 和三维图。这里靠**时间线**：

```python
import numpy as np, tracking
from cases3 import H, GOAL_XY
from cases2 import START, STATIC

frames = tracking.run_frames(STATIC, START, GOAL_XY, 10, H, 150,
                             np.random.default_rng(0), consistency_penalty=1.0)
for i, f in enumerate(frames):
    print(i, f)          # Frame.__repr__ 里就有 alphas / betas / chosen
```

看两样东西：`alphas` 该是一组固定的小数字（比如永远是 `[1, 2]`），
`chosen` 该是一条直线。如果 `alphas` 每帧都在涨（`[1,2]` -> `[3,4]` -> `[5,6]`），
说明继承完全没生效——八成是 `match_guards` 没硬编码 `0 -> 0` / `1 -> 1`。

这是本阶段的诊断口诀：**alpha 持续上涨 ⟹ 身份全丢了，先查 guard 对应。**

## 做完之后

54 个用例全过之后，再去对照官方实现
[tud-amr/guidance_planner](https://github.com/tud-amr/guidance_planner)，
重点看它的 `graph_search.cpp` 里怎么做 ID 传播的。**先自己写完再看。**

下一阶段：控制点优化 + 三次样条 + 引导轨迹选择（论文 III-D/E，文档 4.5-4.6）。
那一阶段会把 `select_guidance` 里这个简化的代价换成完整的式 (9)，
并且把分段线性的折线变成能被跟踪的二阶连续曲线。
