"""阶段 2 的图数据结构。这个文件是脚手架，不需要修改。

===============================================================
心智模型：这张图是「状态空间里有哪几条通道」的离散快照
===============================================================

阶段 1 的三个函数都是**纯谓词**：给两个状态，回答行不行，没有记忆。
阶段 2 开始你要**增量维护一个数据结构**，难点从「公式对不对」变成
「每轮循环之后，这张图必须满足什么性质」。

所以先把四条不变量记住，它们是你调试时唯一的抓手：

1. **图是二分的**：边只存在于 guard 和 connector 之间。没有 guard-guard 边，
   也没有 connector-connector 边。
2. **每个 connector 的度恰好是 2**，连着两个**不同**的 guard。它是按定义
   构造出来的，不是后来长出来的。
3. **任意两个 guard 互相不可见**（起点/终点这对种子除外）。这是
   「|L| = 0 才升为 guard」这条规则的直接推论，也是整张图能保持在
   几十个节点量级、而不是上千个的原因。
4. **每条边在时间上严格向前**。于是整张图自动是一个 DAG，
   你的 DFS 根本不需要防环逻辑——这是时间维度白送的。

语义上：guard 代表「状态空间里互相看不见的若干个区域」，connector 代表
「把两个区域搭起来的一座桥」。两个区域之间可以有多座桥，而**那些桥是否
属于同一个 UVD 类**，就是阶段 2 真正要回答的问题。

---------------------------------------------------------------
为什么边不存方向
---------------------------------------------------------------

因为方向是**可以从 State.t 算出来的**，存两份迟早会不一致。邻接表存无向边，
需要方向时比一下 .t 即可。这是「让错误只可能发生在一个地方」的老办法：
时间顺序只在 edge_feasible 内部处理一次，别的地方都不碰。
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum

from world import State, Trajectory, World


class NodeKind(Enum):
    GUARD = "guard"
    CONNECTOR = "connector"


@dataclass
class Node:
    """图中的一个节点。

    id 是**身份**，state 是**坐标**。请一律用 id 做比较、做字典键、做集合元素；
    State 是一组浮点数，拿它当身份用迟早会被浮点误差坑到。
    （这是阶段 1「c1 * c1 以为是标量其实是数组」那类错误的同形版本：
    搞混了「一个东西是什么」和「一个东西长什么样」。）
    """

    id: int
    state: State
    kind: NodeKind
    guards: tuple[int, int] | None = None
    """connector 专用：它连接的两个 guard 的 id，按**时间先后**排序。
    guard 节点这一项是 None。"""


class Graph:
    """Visibility-PRM 的图。

    只负责「存」和「查」，不负责任何算法决策——要不要把某个样本加进来、
    加成 guard 还是 connector、要不要替换旧的 connector，全都是 exercise2 的事。
    """

    def __init__(self, world: World) -> None:
        self.world = world
        self._nodes: dict[int, Node] = {}
        self._adj: dict[int, set[int]] = {}
        self._by_guard_pair: dict[tuple[int, int], list[int]] = {}
        self._next_id = 0

    # ---------- 写 ----------

    def add_guard(self, state: State) -> int:
        """加一个 guard，返回它的 id。"""
        node_id = self._next_id
        self._next_id += 1
        self._nodes[node_id] = Node(id=node_id, state=state, kind=NodeKind.GUARD)
        self._adj[node_id] = set()
        return node_id

    def add_connector(self, state: State, guard_a: int, guard_b: int) -> int:
        """加一个 connector 并自动连上两条边，返回它的 id。

        guard_a / guard_b 的传入顺序无所谓，内部会按时间先后归一化后存进
        Node.guards，并登记到 (min_id, max_id) 的索引里。
        """
        if guard_a == guard_b:
            raise ValueError("connector 的两个 guard 必须不同")
        for g in (guard_a, guard_b):
            if self._nodes[g].kind is not NodeKind.GUARD:
                raise ValueError(f"节点 {g} 不是 guard")

        early, late = (guard_a, guard_b)
        if self._nodes[early].state.t > self._nodes[late].state.t:
            early, late = late, early

        node_id = self._next_id
        self._next_id += 1
        self._nodes[node_id] = Node(id=node_id, state=state,
                                    kind=NodeKind.CONNECTOR, guards=(early, late))
        self._adj[node_id] = {early, late}
        self._adj[early].add(node_id)
        self._adj[late].add(node_id)
        self._by_guard_pair.setdefault(self.pair_key(early, late), []).append(node_id)
        return node_id

    def remove_connector(self, node_id: int) -> None:
        """删掉一个 connector 及其两条边。替换旧 connector 时用。"""
        node = self._nodes[node_id]
        if node.kind is not NodeKind.CONNECTOR or node.guards is None:
            raise ValueError(f"节点 {node_id} 不是 connector")
        for g in node.guards:
            self._adj[g].discard(node_id)
        self._by_guard_pair[self.pair_key(*node.guards)].remove(node_id)
        del self._adj[node_id]
        del self._nodes[node_id]

    # ---------- 读 ----------

    @staticmethod
    def pair_key(guard_a: int, guard_b: int) -> tuple[int, int]:
        return (guard_a, guard_b) if guard_a <= guard_b else (guard_b, guard_a)

    @property
    def nodes(self) -> dict[int, Node]:
        return self._nodes

    def state(self, node_id: int) -> State:
        return self._nodes[node_id].state

    def kind(self, node_id: int) -> NodeKind:
        return self._nodes[node_id].kind

    def guard_ids(self) -> list[int]:
        return [n.id for n in self._nodes.values() if n.kind is NodeKind.GUARD]

    def connector_ids(self) -> list[int]:
        return [n.id for n in self._nodes.values() if n.kind is NodeKind.CONNECTOR]

    def neighbors(self, node_id: int) -> list[int]:
        """无向邻居。要做 DFS 的话，你需要的是其中**时间在后**的那些——
        自己比一下 state(nid).t，别在这里图省事。"""
        return sorted(self._adj[node_id])

    def connectors_between(self, guard_a: int, guard_b: int) -> list[int]:
        """这一对 guard 之间已有的全部 connector。

        去重只在**共享同一对 guard** 的 connector 之间进行，因为
        uvd_equivalent 要求两条轨迹的起点终点都相同，而
        g0 -> x -> g1 和 g0 -> xj -> g1 正好满足这个前提。
        所以别去遍历全图，用这个索引。
        """
        return list(self._by_guard_pair.get(self.pair_key(guard_a, guard_b), ()))

    def edges(self) -> list[tuple[int, int]]:
        """全部无向边，(较早, 较晚) 的形式。画图和性质检查用。"""
        out = []
        for node_id, adj in self._adj.items():
            for other in adj:
                if node_id < other:
                    a, b = node_id, other
                    if self.state(a).t > self.state(b).t:
                        a, b = b, a
                    out.append((a, b))
        return out

    def path_trajectory(self, node_ids: list[int], name: str = "") -> Trajectory:
        """把一串节点 id 变成 Trajectory（按 world.time_scale 参数化）。"""
        return Trajectory(states=[self.state(i) for i in node_ids],
                          time_scale=self.world.time_scale, name=name)

    def stats(self) -> dict[str, int]:
        """统计量。阶段 2 的 bug 大多不报错，先看这几个数再看图。"""
        pairs = {k: len(v) for k, v in self._by_guard_pair.items() if v}
        return {
            "guards": len(self.guard_ids()),
            "connectors": len(self.connector_ids()),
            "edges": len(self.edges()),
            "guard_pairs": len(pairs),
            "max_connectors_per_pair": max(pairs.values(), default=0),
        }

    def __len__(self) -> int:
        return len(self._nodes)

    def __repr__(self) -> str:
        s = self.stats()
        return (f"Graph(guards={s['guards']}, connectors={s['connectors']}, "
                f"edges={s['edges']})")


@dataclass
class FixedSampler:
    """按顺序吐出预先给定的样本，用来写**确定性**的测试。

    阶段 2 最大的测试困难是随机性。解决办法就是把采样器做成可注入的：
    真实运行时用你的 sample_state，测试时换成这个。
    """

    states: list[State]
    _index: int = field(default=0, repr=False)

    def __call__(self, world: World, start: State, goal: State, rng) -> State:
        if self._index >= len(self.states):
            raise IndexError("FixedSampler 的样本用完了")
        state = self.states[self._index]
        self._index += 1
        return state
