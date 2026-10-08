# 阶段 2：Visibility-PRM 建图 + 拓扑类枚举

对应论文第 III-B 节和 **Algorithm 1**，也就是 `../论文详解-GloballyGuidedTrajectoryPlanning.md`
里的 4.3。

一句话概括这一阶段要建立的能力：**把「无碰撞空间里有几条通道」这个连续的拓扑问题，
转化成一个有限图上的枚举问题。**

阶段 1 回答的是「两条给定的轨迹是不是同一个通道」，是**判别**；
这一阶段要回答「通道一共有哪些」，是**生成**。难度不在一个量级。

输入是 `World` 加起点终点两个 `State`，输出是一组两两 UVD 不等价的 `Trajectory`。
注意输出的措辞：是**每个拓扑类挑一个代表**，不是「最优路径」。这是它和 A\*、RRT\*
最根本的区别——要最优路径用不着 PRM，正因为要的是「有几种选择」，才必须有这么一个
既能覆盖整个状态空间、又不会给出成千上万条近似重复路径的结构。

## 文件

| 文件 | 说明 |
|---|---|
| `exercise2.py` | ⭐ **你要写的文件**，八个函数 |
| `graph.py` | `Node` / `Graph` / `FixedSampler`，脚手架 |
| `cases2.py` | 验收用例，脚手架 |
| `check2.py` | 判卷 |
| `viz2.py` | rerun 三维可视化 |

阶段 1 的 `world.py` / `exercise.py` 继续用，不用改。

## 怎么跑

```bash
.venv/bin/python globally_guided/check2.py --stage edge       # edge_feasible
.venv/bin/python globally_guided/check2.py --stage sample     # sample_state
.venv/bin/python globally_guided/check2.py --stage visible    # visible_guards
.venv/bin/python globally_guided/check2.py --stage add        # try_add_sample
.venv/bin/python globally_guided/check2.py --stage dfs        # enumerate_paths
.venv/bin/python globally_guided/check2.py --stage invariant  # 图不变量
.venv/bin/python globally_guided/check2.py --stage topology   # 端到端类数
.venv/bin/python globally_guided/check2.py                    # 全跑
.venv/bin/python globally_guided/check2.py -v                 # 失败时的详细诊断

.venv/bin/python globally_guided/viz2.py --list
.venv/bin/python globally_guided/viz2.py --scenario single_static
.venv/bin/python globally_guided/viz2.py --scenario big_blocker --seed 2 --save /tmp/prm.rrd
```

总共 43 个计分用例，另有 1 个只打印不判对错的观察场景。建议先 `edge` -> `sample` -> `visible` -> `add` -> `dfs`，
这五个全绿了再去碰 `invariant` 和 `topology`（它们是端到端的，靠前面五个支撑）。

## 先把四条不变量记住

这是阶段 2 和阶段 1 最大的区别。阶段 1 的函数是纯谓词，错了就是数学错了，
一测就现形。阶段 2 你在增量维护一个数据结构，**图建错了程序照样跑完**，
只是少找到一个通道。所以先把这四条写在注释里，再写代码：

1. **图是二分的**：边只存在于 guard 和 connector 之间。
2. **每个 connector 的度恰好是 2**，连着两个不同的 guard。
3. **任意两个 guard 互相不可见**（起点/终点这对种子除外）。
4. **每条边在时间上严格向前**，所以整张图自动是个 DAG。

`--stage invariant` 就是逐条查这四条的。它不依赖任何期望答案，所以对任何场景、
任何种子都成立——这是随机算法里性价比最高的一类测试。

## 你在阶段 1 犯的错，在这里会怎么复发

| 阶段 1 | 阶段 2 的同形错误 | 被哪个用例抓 |
|---|---|---|
| `c1 * c1` 以为是标量其实是数组 | 节点 id / 下标 / `State` 三者混用 | 全线 |
| `a - b` 方向写反，不报错只给错答案 | 边的时间顺序写反 | `reversed_arguments` |
| 除零检查写在除法之后 | 空分支没先挡（一条路径也没找到） | `no_path` |
| `return` 写在 for 里面提前退出 | `visible_guards` 数到第一个就返回 | `sees_three` |
| 用 `robot_radius` 而不是 `inflated_radius` | 自己手算路径长度而不用 `Trajectory.length` | `shorter_replaces` |

还有一条上次没有、这次新增的：**随机算法必须可复现。**
所有随机数都走显式传入的 `rng = np.random.default_rng(seed)`，
不要用 `np.random.*` 的全局状态。否则你会遇到「跑三次过两次」这种最难受的情况。

## 三个值得提前知道的坑

**可见性在时空里不是对称关系。** `connection_valid(a, b)` 要求 `b.t > a.t`，
所以它和 `connection_valid(b, a)` 不可能同时成立。但「两点能不能连」这件事本身
是对称的。把排序锁在 `edge_feasible` 内部，别的地方都不碰。这是阶段 1
「用 `inflated_radius()` 而不是自己拼半径」的同一个道理。

**`|L| = 2` 还要求一前一后。** 论文没明说，但你必须自己定：两个 guard 必须
一个在 `x` 之前、一个在之后。都在之前的话，`g0 -> x -> g1` 就得在某一段上倒着走时间。

**UVD 不是等价关系，它不满足传递性。** A 与 B 等价、B 与 C 等价，不意味着
A 与 C 等价。所以「把路径分组」这个说法根本不严谨，实际做法只能是贪心：
按代价从低到高排序，依次尝试加入结果集，只要和已选中的任何一条等价就丢弃。
没想清楚这一点，你会写出一个结果依赖遍历顺序、每次跑都不一样的去重。

## 怎么调试

阶段 1 靠打印中间量（`c0` / `c1` / `A` / `B` / `u`），那套在这里会失效——
一堆节点坐标打印出来你什么也发现不了。这里分两层：

**先看统计量。** `graph.stats()` 给出 guard 数、connector 数、边数、
guard 对数。它们能帮你快速定位到是哪一段坏了。一个例子：**如果 guard 数
接近样本数**，说明可见性检查几乎总在返回空——多半是时间顺序写反了。

**然后看图。** `viz2.py` 把 guard、connector、边、以及每一个拓扑类的代表轨迹
用不同颜色画进 rerun。拓扑 bug 的典型症状是「一个静态障碍的场景只找到 1 类
而不是 2 类」，这种事只能看出来，推不出来。配上 `--num-samples 10` 调小样本数，
能看清楚图是怎么一步步长起来的。

## 关于 `crossing_pedestrian` 那个场景

论文 Fig. 1 把一个横穿行人的场景画成了**四**种行为（左绕、右绕、抢行、让行），
但在这个无边界的场景里正确答案是 **2**。

原因值得自己画一遍想清楚：在状态空间里，行人是一根从底面贯到顶面的斜管，
机器人必须在某个时刻 `t*` 穿过 `x = 0`，而行人那时在 `y = 3t* - 4.5`。
机器人只能从行人上方或下方绕。「抢行」（早点过，行人还在很下面）其实就是
「从上方绕」，是同一个通道。四种行为要真的分开，得有道路边界把 y 方向堵住。

这也是一个提醒：拓扑类数是**场景的属性**，不是算法的参数。别把论文插图里的
数字当成普适答案。

## 做完之后

43 个用例全过之后，再去对照官方实现
[tud-amr/guidance_planner](https://github.com/tud-amr/guidance_planner)，
重点看 `graph_search.cpp` 里 guard/connector 的划分和替换逻辑。**先自己写完再看。**

下一阶段：segment ID / trajectory ID 的跨帧传播（论文 III-C，文档 4.4）。
核心是 Algorithm 1 第 4-6 行的 `ReintroduceSample`：把上一帧的节点按
`(x, y, t - h)` 重新放回图里，让引导轨迹在连续迭代间保持一致，机器人不再反复横跳。
