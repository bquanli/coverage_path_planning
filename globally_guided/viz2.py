#!/usr/bin/env python3
"""阶段 2 的三维可视化。不需要修改。

阶段 1 调试靠打印中间量（c0 / c1 / A / B / u），那套在这里会失效：
一堆节点坐标打印出来你什么也发现不了。阶段 2 的 bug 几乎都是「图长得不对」，
看图比读 log 快一个数量级。

    python globally_guided/viz2.py --list
    python globally_guided/viz2.py --scenario single_static
    python globally_guided/viz2.py --scenario crossing_pedestrian --seed 3
    python globally_guided/viz2.py --scenario big_blocker --save /tmp/prm.rrd   # 无头

配色：红点 = guard，蓝点 = connector，灰线 = 图的边，粗彩线 = 最终的每一个拓扑类。
没画出彩线就说明 DFS 或去重那一步挂了。
"""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

import numpy as np

import cases2 as case_data
import exercise2
from graph import NodeKind
from world import State, Trajectory, World

ORANGE = [255, 140, 0]
BLACK = [20, 20, 20]
GREY = [160, 160, 160]
RED = [220, 50, 50]
BLUE = [60, 110, 230]
PALETTE = [[0, 160, 90], [230, 80, 180], [250, 190, 0], [0, 180, 200],
           [150, 90, 220], [200, 60, 60], [90, 140, 40], [240, 130, 60]]


def _circle(centre, radius: float, z: float, segments: int = 36):
    return [[float(centre[0] + radius * math.cos(2 * math.pi * i / segments)),
             float(centre[1] + radius * math.sin(2 * math.pi * i / segments)),
             float(z)] for i in range(segments + 1)]


def _log_obstacles(rr, world: World, horizon: float, layers: int = 25) -> None:
    for index, obs in enumerate(world.obstacles):
        radius = world.inflated_radius(obs)
        rings, centres = [], []
        for step in range(layers + 1):
            t = horizon * step / layers
            centre = obs.at(t)
            z = t * world.time_scale
            rings.append(_circle(centre, radius, z))
            centres.append([float(centre[0]), float(centre[1]), float(z)])
        rr.log(f"world/obstacle_{index}/inflated",
               rr.LineStrips3D(rings, colors=[ORANGE] * len(rings), radii=0.012))
        rr.log(f"world/obstacle_{index}/centre",
               rr.LineStrips3D([centres], colors=[BLACK], radii=0.02))


def _xyz(state: State, world: World):
    return [state.x, state.y, state.t * world.time_scale]


def main() -> int:
    parser = argparse.ArgumentParser(description="Visibility-PRM 的三维可视化")
    parser.add_argument("--scenario", default="single_static")
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--num-samples", type=int, default=None,
                        help="覆盖场景自带的采样数。调小一点能看清楚图是怎么长起来的。")
    parser.add_argument("--save", type=Path, default=None,
                        help="写到 .rrd 文件而不开窗口（无头环境用这个）")
    parser.add_argument("--list", action="store_true")
    args = parser.parse_args()

    if args.list:
        for scenario in case_data.TOPOLOGY_SCENARIOS:
            expected = ("观察用" if scenario.expected_classes is None
                        else f"{scenario.expected_classes} 类")
            print(f"  {scenario.name:<22} expected={expected}")
        return 0

    scenario = case_data.find_topology_scenario(args.scenario)
    world = scenario.world
    num_samples = args.num_samples or scenario.num_samples

    rng = np.random.default_rng(args.seed)
    try:
        graph = exercise2.build_prm(world, scenario.start, scenario.goal,
                                    num_samples, rng)
    except NotImplementedError as exc:
        print(f"还没实现：{exc}", file=sys.stderr)
        return 1

    paths, taus = [], []
    for step, fn in (("enumerate_paths", lambda: exercise2.enumerate_paths(graph)),
                     ("distinct_trajectories",
                      lambda: exercise2.distinct_trajectories(paths, graph, world))):
        try:
            result = fn()
        except NotImplementedError:
            print(f"{step} 还没实现，只画图不画轨迹。")
            break
        if step == "enumerate_paths":
            paths = result
        else:
            taus = result

    try:
        import rerun as rr
    except ImportError:
        print("没有装 rerun-sdk。uv sync 或 pip install rerun-sdk。", file=sys.stderr)
        return 1

    rr.init(f"prm_{scenario.name}", spawn=args.save is None)
    if args.save is not None:
        rr.save(args.save)
    rr.log("/", rr.ViewCoordinates.RIGHT_HAND_Z_UP, static=True)

    _log_obstacles(rr, world, scenario.goal.t)

    guard_ids = graph.guard_ids()
    connector_ids = graph.connector_ids()
    if guard_ids:
        rr.log("prm/guards", rr.Points3D(
            [_xyz(graph.state(i), world) for i in guard_ids],
            colors=[RED] * len(guard_ids), radii=0.09,
            labels=[f"g{i}" for i in guard_ids]))
    if connector_ids:
        rr.log("prm/connectors", rr.Points3D(
            [_xyz(graph.state(i), world) for i in connector_ids],
            colors=[BLUE] * len(connector_ids), radii=0.06))
    edges = graph.edges()
    if edges:
        rr.log("prm/edges", rr.LineStrips3D(
            [[_xyz(graph.state(a), world), _xyz(graph.state(b), world)]
             for a, b in edges], colors=[GREY] * len(edges), radii=0.008))

    for index, tau in enumerate(taus):
        points = [_xyz(tau.at(k / 120), world) for k in range(121)]
        rr.log(f"guidance/class_{index}", rr.LineStrips3D(
            [points], colors=[PALETTE[index % len(PALETTE)]], radii=0.04))

    stats = graph.stats()
    rr.log("notes", rr.TextDocument(
        f"# {scenario.name}  (seed={args.seed}, samples={num_samples})\n\n"
        f"{scenario.description}\n\n"
        f"- guards: {stats['guards']}\n- connectors: {stats['connectors']}\n"
        f"- 路径: {len(paths)}\n- 拓扑类: {len(taus)}\n\n"
        "z 轴是时间。红点 guard，蓝点 connector，灰线 边，彩线 拓扑类。",
        media_type="text/markdown"), static=True)

    print(f"场景 {scenario.name}（seed={args.seed}, samples={num_samples}）")
    print(f"  {stats}")
    print(f"  DFS 找到 {len(paths)} 条路径 -> 去重后 {len(taus)} 个拓扑类", end="")
    if scenario.expected_classes is not None:
        print(f"（期望 {scenario.expected_classes}）")
    else:
        print("（该场景不判对错）")
    if args.save is not None:
        print(f"  已写入 {args.save}，用 rerun {args.save} 打开。")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
