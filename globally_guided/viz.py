#!/usr/bin/env python3
"""把一个 UVD 用例画在 (x, y, t) 三维空间里。不需要修改。

    python globally_guided/viz.py --list
    python globally_guided/viz.py --case crossing_speed_up_vs_slow_down
    python globally_guided/viz.py --case static_left_vs_right --save /tmp/uvd.rrd

纵轴（z）是时间。障碍物是一根沿时间轴倒下的斜圆柱，两条轨迹是三维折线，
连接它们的「横档」就是 UVD 要检查的那些线段：绿色表示无碰撞，红色表示撞了。
只要有一根红的，两条轨迹就是拓扑不等价的。

横档的颜色用的是**你自己写的** segment_collision_free；还没实现时全部画成灰色。
"""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

import numpy as np
import rerun as rr

sys.path.insert(0, str(Path(__file__).resolve().parent))

import cases as case_data
import exercise
from world import Trajectory, World

GREEN = (80, 200, 120)
RED = (220, 70, 70)
GREY = (150, 150, 150)
BLUE = (70, 130, 230)
ORANGE = (240, 160, 60)
BLACK = (30, 30, 30)


def _circle(centre: np.ndarray, radius: float, z: float, segments: int = 32):
    angles = np.linspace(0.0, 2.0 * math.pi, segments + 1)
    return [
        [float(centre[0] + radius * math.cos(t)),
         float(centre[1] + radius * math.sin(t)),
         float(z)]
        for t in angles
    ]


def _log_obstacles(world: World, slices: int = 40) -> None:
    import rerun as rr

    for index, obs in enumerate(world.obstacles):
        radius = world.inflated_radius(obs)
        rings, centres = [], []
        for k in range(slices + 1):
            t = world.horizon * k / slices
            centre = obs.at(t)
            z = t * world.time_scale
            rings.append(_circle(centre, radius, z))
            centres.append([float(centre[0]), float(centre[1]), float(z)])
        # 膨胀后的圆盘堆叠成的「管」，以及圆心轨迹。
        rr.log(f"world/obstacle_{index}/inflated",
               rr.LineStrips3D(rings, colors=[ORANGE] * len(rings), radii=0.012))
        rr.log(f"world/obstacle_{index}/centre",
               rr.LineStrips3D([centres], colors=[BLACK], radii=0.02))


def _log_trajectory(path: str, tau: Trajectory, colour, world: World,
                    resolution: int = 120) -> None:

    points = []
    for index in range(resolution + 1):
        state = tau.at(index / resolution)
        points.append([state.x, state.y, state.t * world.time_scale])
    rr.log(path, rr.LineStrips3D([points], colors=[colour], radii=0.03))


def _log_rungs(tau_1: Trajectory, tau_2: Trajectory, world: World,
               num_samples: int) -> tuple[int, int]:

    strips, colours = [], []
    hits = unknown = 0
    for index in range(num_samples + 1):
        s = index / num_samples
        p, q = tau_1.at(s), tau_2.at(s)
        try:
            free = exercise.segment_collision_free(p, q, world)
            colour = GREEN if free else RED
            hits += 0 if free else 1
        except Exception:  # noqa: BLE001 - 还没实现或者报错都当未知
            colour = GREY
            unknown += 1
        strips.append([[p.x, p.y, p.t * world.time_scale],
                       [q.x, q.y, q.t * world.time_scale]])
        colours.append(colour)
    rr.log("guidance/rungs", rr.LineStrips3D(strips, colors=colours, radii=0.012))
    return hits, unknown


def main() -> int:
    parser = argparse.ArgumentParser(description="UVD 用例的三维可视化")
    parser.add_argument("--case", default="crossing_speed_up_vs_slow_down")
    parser.add_argument("--num-samples", type=int, default=20)
    parser.add_argument("--save", type=Path, default=None,
                        help="写到 .rrd 文件而不开窗口（无头环境用这个）")
    parser.add_argument("--list", action="store_true", help="列出所有用例名")
    args = parser.parse_args()

    if args.list:
        for case in case_data.UVD_CASES:
            print(f"  {case.name:<34} expected={'等价' if case.expected else '不等价'}")
        return 0

    try:
        import rerun as rr
    except ImportError:
        print("没有装 rerun-sdk。uv sync 或 pip install rerun-sdk。", file=sys.stderr)
        return 1

    case = case_data.find_uvd_case(args.case)
    rr.init(f"uvd_{case.name}", spawn=args.save is None)
    if args.save is not None:
        rr.save(args.save)

    rr.log("/", rr.ViewCoordinates.RIGHT_HAND_Z_UP, static=True)
    rr.log("notes", rr.TextDocument(
        f"# {case.name}\n\n{case.description}\n\n"
        f"期望结果：{'UVD 等价' if case.expected else 'UVD 不等价'}\n\n"
        "z 轴是时间。橙色管是膨胀后的障碍，红色横档表示该处连线撞了。",
        media_type="text/markdown"), static=True)

    _log_obstacles(case.world)
    _log_trajectory("guidance/tau_1", case.tau_1, BLUE, case.world)
    _log_trajectory("guidance/tau_2", case.tau_2, ORANGE, case.world)
    hits, unknown = _log_rungs(case.tau_1, case.tau_2, case.world, args.num_samples)

    endpoints = [[case.tau_1.start.x, case.tau_1.start.y,
                  case.tau_1.start.t * case.world.time_scale],
                 [case.tau_1.goal.x, case.tau_1.goal.y,
                  case.tau_1.goal.t * case.world.time_scale]]
    rr.log("guidance/endpoints",
           rr.Points3D(endpoints, colors=[BLACK, BLACK], radii=0.08,
                       labels=["start", "goal"]))

    print(f"用例 {case.name}：{case.description}")
    if unknown:
        print(f"  segment_collision_free 还没实现，{unknown} 根横档画成了灰色。")
    else:
        verdict = "不等价" if hits else "等价"
        print(f"  {hits} 根横档碰撞 -> 你的实现判为{verdict}，"
              f"期望{'等价' if case.expected else '不等价'}。")
    if args.save is not None:
        print(f"  已写入 {args.save}，用 rerun {args.save} 打开。")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
