#!/usr/bin/env python3
"""把 check2.py 的验收用例画成图。脚手架，不需要修改。

viz2.py 用 rerun，需要开窗口；这个脚本用 matplotlib 直接出 PNG，
适合「只看到 PASS/FAIL、不知道用例到底长什么样」的时候。

    python globally_guided/viz_cases.py                 # 全画
    python globally_guided/viz_cases.py --stage edge    # 只画一组
    python globally_guided/viz_cases.py --stage add --out /tmp

每张图的坐标系都是 (x, y, t)：**竖直方向就是时间**，
障碍物因此是一根管子——静态障碍是直立的圆柱，匀速行人是斜着的圆柱。
管子画的是膨胀半径 R = robot_radius + obstacle_radius，所以只要
线不进管子，机器人就不碰障碍。

图例统一：
    红点   guard          蓝点   connector      灰线   图的边
    绿线   可行 / 可见     红虚线 不可行 / 不可见
"""

from __future__ import annotations

import argparse
import os
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
os.environ.setdefault("MPLCONFIGDIR", "/tmp/mplcfg")

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

import cases2 as case_data
import exercise2
from graph import Graph, NodeKind
from world import State, World

GUARD_C = "#dc3232"
CONN_C = "#3c6ee6"
EDGE_C = "#a0a0a0"
OK_C = "#17a257"
BAD_C = "#dc3232"
OBS_C = "#ff8c00"
PALETTE = ["#00a05a", "#e650b4", "#fabe00", "#00b4c8", "#965adc", "#c83c3c"]


# ------------------------------------------------------------------ 画布工具


def _ax3d(fig, nrows, ncols, index, title, compact: bool = True):
    """compact=False 给格子大的布局用：标签不再往里挤，刻度也放大一点。"""
    ax = fig.add_subplot(nrows, ncols, index, projection="3d")
    label_pad = -8 if compact else 4
    label_size = 7 if compact else 9
    tick_size = 6 if compact else 8
    tick_pad = -2 if compact else 1
    ax.set_title(title, fontsize=9, pad=2)
    ax.set_xlabel("x", fontsize=label_size, labelpad=label_pad)
    ax.set_ylabel("y", fontsize=label_size, labelpad=label_pad)
    ax.set_zlabel("t", fontsize=label_size, labelpad=label_pad)
    ax.tick_params(labelsize=tick_size, pad=tick_pad)
    ax.view_init(elev=22, azim=-62)
    return ax


def _draw_obstacles(ax, world: World, t_max: float, layers: int = 24) -> None:
    """把每个障碍物画成时间方向上的管子。"""
    for obs in world.obstacles:
        radius = world.inflated_radius(obs)
        ts = np.linspace(0.0, max(t_max, 1e-6), layers)
        th = np.linspace(0.0, 2 * np.pi, 40)
        centres = np.array([obs.at(float(t)) for t in ts])
        x = centres[:, 0][None, :] + radius * np.cos(th)[:, None]
        y = centres[:, 1][None, :] + radius * np.sin(th)[:, None]
        z = np.broadcast_to(ts * world.time_scale, x.shape)
        ax.plot_surface(
            x, y, z, color=OBS_C, alpha=0.18, linewidth=0, shade=False, zorder=0
        )
        ax.plot(
            centres[:, 0],
            centres[:, 1],
            ts * world.time_scale,
            color=OBS_C,
            lw=0.8,
            alpha=0.7,
        )


def _xyz(state: State, world: World):
    return state.x, state.y, state.t * world.time_scale


def _seg(ax, p: State, q: State, world: World, **kw):
    ax.plot(*zip(_xyz(p, world), _xyz(q, world)), **kw)


def _autoscale(ax, world: World, states, t_max: float, margin: float = 0.8):
    xs = [s.x for s in states]
    ys = [s.y for s in states]
    for obs in world.obstacles:
        r = world.inflated_radius(obs)
        for t in (0.0, t_max):
            cx, cy = obs.at(t)
            xs += [cx - r, cx + r]
            ys += [cy - r, cy + r]
    ax.set_xlim(min(xs) - margin, max(xs) + margin)
    ax.set_ylim(min(ys) - margin, max(ys) + margin)
    ax.set_zlim(0.0, max(t_max, 1e-6) * world.time_scale)


def _mark(ok: bool) -> str:
    return "PASS" if ok else "FAIL"


def _save(fig, out_dir: Path, name: str) -> Path:
    out_dir.mkdir(parents=True, exist_ok=True)
    path = out_dir / name
    fig.savefig(path, dpi=130, bbox_inches="tight")
    plt.close(fig)
    print(f"  -> {path}")
    return path


def _draw_graph(ax, graph: Graph, world: World, labels: bool = True) -> None:
    for a, b in graph.edges():
        _seg(ax, graph.state(a), graph.state(b), world, color=EDGE_C, lw=1.0, zorder=2)
    for kind, colour, size in (
        (NodeKind.GUARD, GUARD_C, 42),
        (NodeKind.CONNECTOR, CONN_C, 26),
    ):
        ids = [i for i in graph.nodes if graph.kind(i) is kind]
        if not ids:
            continue
        pts = np.array([_xyz(graph.state(i), world) for i in ids])
        ax.scatter(
            pts[:, 0],
            pts[:, 1],
            pts[:, 2],
            c=colour,
            s=size,
            depthshade=False,
            zorder=5,
        )
        if labels:
            for i, p in zip(ids, pts):
                ax.text(p[0], p[1], p[2], f" {i}", fontsize=6, color=colour)


# ------------------------------------------------------------------ edge


def plot_edge(out_dir: Path) -> None:
    cases = case_data.EDGE_CASES
    fig = plt.figure(figsize=(16.0, 7.2))
    for n, case in enumerate(cases, start=1):
        try:
            got = exercise2.edge_feasible(case.p, case.q, case.world)
        except NotImplementedError:
            got = None
        ok = got == case.expected
        t_max = max(case.p.t, case.q.t, 0.5)
        ax = _ax3d(
            fig,
            2,
            4,
            n,
            f"{n}. {case.name}\nexpect={case.expected} got={got} [{_mark(ok)}]",
        )
        _draw_obstacles(ax, case.world, t_max)
        colour = OK_C if case.expected else BAD_C
        style = "-" if case.expected else "--"
        _seg(ax, case.p, case.q, case.world, color=colour, lw=2.2, ls=style, zorder=6)
        for s, lab in ((case.p, "p"), (case.q, "q")):
            x, y, z = _xyz(s, case.world)
            ax.scatter([x], [y], [z], c="black", s=24, depthshade=False, zorder=7)  # type: ignore
            ax.text(x, y, z, f" {lab}", fontsize=7)
        _autoscale(ax, case.world, [case.p, case.q], t_max)
    fig.suptitle(
        "stage edge - edge_feasible: green solid = should be feasible, "
        "red dashed = should be rejected",
        fontsize=11,
    )
    fig.tight_layout(rect=(0, 0, 1, 0.95))
    _save(fig, out_dir, "cases_edge.png")


# ------------------------------------------------------------------ sample


def plot_sample(out_dir: Path, draws: int = 900, seed: int = 0) -> None:
    scenarios = case_data.SAMPLE_SCENARIOS
    fig = plt.figure(figsize=(17.0, 4.4))
    for n, sc in enumerate(scenarios, start=1):
        rng = np.random.default_rng(seed)
        pts, misses = [], 0
        for _ in range(draws):
            try:
                s = exercise2.sample_state(sc.world, sc.start, sc.goal, rng)
            except NotImplementedError:
                s = None
            if s is None:
                misses += 1
            else:
                pts.append(_xyz(s, sc.world))
        ax = _ax3d(
            fig,
            1,
            4,
            n,
            f"{n}. {sc.name}\n{len(pts)} samples, {misses} None "
            f"(v_max={sc.world.max_velocity})",
        )
        _draw_obstacles(ax, sc.world, sc.goal.t)
        if pts:
            arr = np.array(pts)
            ax.scatter(
                arr[:, 0],
                arr[:, 1],
                arr[:, 2],  # type: ignore
                c=arr[:, 2],
                cmap="viridis",
                s=4,
                alpha=0.55,
                depthshade=False,
            )
        for s, lab in ((sc.start, "start"), (sc.goal, "goal")):
            x, y, z = _xyz(s, sc.world)
            ax.scatter([x], [y], [z], c=GUARD_C, s=55, depthshade=False, zorder=7)  # type: ignore
            ax.text(x, y, z, f" {lab}", fontsize=7)
        _autoscale(ax, sc.world, [sc.start, sc.goal], sc.goal.t)
    fig.suptitle(
        "stage sample - sample_state: the cloud must form a LENS "
        "(two cones meeting in the middle), not a box",
        fontsize=11,
    )
    fig.tight_layout(rect=(0, 0, 1, 0.88))
    _save(fig, out_dir, "cases_sample.png")


# ------------------------------------------------------------------ visible


def plot_visible(out_dir: Path) -> None:
    cases = case_data.VISIBLE_CASES
    fig = plt.figure(figsize=(16.0, 7.2))
    for n, case in enumerate(cases, start=1):
        graph = Graph(case.world)
        for state in case.guards:
            graph.add_guard(state)
        try:
            got = tuple(sorted(exercise2.visible_guards(case.x, graph, case.world)))
        except NotImplementedError:
            got = None
        ok = got == tuple(sorted(case.expected_indices))
        states = [*case.guards, case.x]
        t_max = max(s.t for s in states)
        ax = _ax3d(
            fig,
            2,
            4,
            n,
            f"{n}. {case.name}\nexpect={case.expected_indices} got={got} [{_mark(ok)}]",
        )
        _draw_obstacles(ax, case.world, t_max)
        for gid, g in enumerate(case.guards):
            visible = gid in case.expected_indices
            _seg(
                ax,
                case.x,
                g,
                case.world,
                color=OK_C if visible else BAD_C,
                lw=1.8,
                ls="-" if visible else "--",
                zorder=6,
            )
            x, y, z = _xyz(g, case.world)
            ax.scatter([x], [y], [z], c=GUARD_C, s=42, depthshade=False, zorder=7)
            ax.text(x, y, z, f" g{gid}", fontsize=7, color=GUARD_C)
        x, y, z = _xyz(case.x, case.world)
        ax.scatter(
            [x], [y], [z], c=CONN_C, s=46, marker="D", depthshade=False, zorder=8
        )
        ax.text(x, y, z, " x", fontsize=7, color=CONN_C)
        _autoscale(ax, case.world, states, t_max)
    fig.suptitle(
        "stage visible - visible_guards: green = guard visible from x, "
        "red dashed = not visible",
        fontsize=11,
    )
    fig.tight_layout(rect=(0, 0, 1, 0.95))
    _save(fig, out_dir, "cases_visible.png")


# ------------------------------------------------------------------ add

_FATE_STYLE = {
    "new guard": (GUARD_C, "^"),
    "connector": (OK_C, "o"),
    "replaced": ("#1f78b4", "P"),
    "discarded": ("#808080", "x"),
}


def _inject(case) -> tuple[Graph, list[tuple[State, str]]]:
    """按顺序注入样本，记下每一个的命运（升 guard / 成 connector / 替换 / 丢弃）。"""
    graph = Graph(case.world)
    for state in (case_data.START, case_data.GOAL, *case.extra_guards):
        graph.add_guard(state)
    fates = []
    for sample in case.samples:
        before_g = set(graph.guard_ids())
        before_c = set(graph.connector_ids())
        try:
            exercise2.try_add_sample(sample, graph, case.world)
        except NotImplementedError:
            fates.append((sample, "discarded"))
            continue
        after_g = set(graph.guard_ids())
        after_c = set(graph.connector_ids())
        if after_g - before_g:
            fate = "new guard"
        elif after_c - before_c:
            fate = "replaced" if before_c - after_c else "connector"
        else:
            fate = "discarded"
        fates.append((sample, fate))
    return graph, fates


def plot_add(out_dir: Path) -> None:
    cases = case_data.ADD_CASES
    fig = plt.figure(figsize=(13.5, 12.0))
    for n, case in enumerate(cases, start=1):
        graph, fates = _inject(case)
        stats = graph.stats()
        ok = (
            stats["guards"] == case.expected_guards
            and stats["connectors"] == case.expected_connectors
        )
        states = [*case.samples, case_data.START, case_data.GOAL, *case.extra_guards]
        t_max = max(s.t for s in states)
        ax = _ax3d(
            fig,
            3,
            3,
            n,
            f"{n}. {case.name}\nguards {stats['guards']}/{case.expected_guards}  "
            f"conn {stats['connectors']}/{case.expected_connectors} [{_mark(ok)}]",
        )
        _draw_obstacles(ax, case.world, t_max)
        _draw_graph(ax, graph, case.world)
        for order, (sample, fate) in enumerate(fates, start=1):
            colour, marker = _FATE_STYLE[fate]
            x, y, z = _xyz(sample, case.world)
            ax.scatter(
                [x],
                [y],
                [z],
                c=colour,
                s=46,
                marker=marker,
                depthshade=False,
                zorder=9,
                linewidths=1.4,
            )
            ax.text(x, y, z, f" #{order} {fate}", fontsize=6, color=colour)
        _autoscale(ax, case.world, states, t_max)
    fig.suptitle(
        "stage add - try_add_sample: fate of every injected sample "
        "(#k = injection order)",
        fontsize=11,
    )
    fig.tight_layout(rect=(0, 0, 1, 0.96))
    _save(fig, out_dir, "cases_add.png")


# ------------------------------------------------------------------ dfs


def plot_dfs(out_dir: Path) -> None:
    cases = case_data.DFS_CASES
    world = case_data.EMPTY
    fig = plt.figure(figsize=(13.5, 8.0))
    for n, case in enumerate(cases, start=1):
        graph = Graph(world)
        for state in case.guards:
            graph.add_guard(state)
        for state, g_a, g_b in case.connectors:
            graph.add_connector(state, g_a, g_b)
        try:
            paths = exercise2.enumerate_paths(graph)
        except NotImplementedError:
            paths = []
        got = sorted(tuple(p) for p in paths)
        ok = got == sorted(case.expected_paths)
        states = [graph.state(i) for i in graph.nodes]
        t_max = max([s.t for s in states] + [0.5])
        ax = _ax3d(
            fig,
            2,
            3,
            n,
            f"{n}. {case.name}\nexpect {len(case.expected_paths)} path(s), "
            f"got {len(got)} [{_mark(ok)}]",
        )
        _draw_graph(ax, graph, world)
        for k, path in enumerate(got):
            pts = np.array([_xyz(graph.state(i), world) for i in path])
            ax.plot(
                pts[:, 0],
                pts[:, 1],
                pts[:, 2],
                color=PALETTE[k % len(PALETTE)],
                lw=2.6,
                alpha=0.85,
                zorder=6,
                label=str(list(path)),
            )
        if got:
            ax.legend(fontsize=6, loc="upper left")
        _autoscale(ax, world, states, t_max)
    fig.suptitle(
        "stage dfs - enumerate_paths: coloured lines are the enumerated "
        "start->goal paths (time must strictly increase)",
        fontsize=11,
    )
    fig.tight_layout(rect=(0, 0, 1, 0.94))
    _save(fig, out_dir, "cases_dfs.png")


# ------------------------------------------------------------------ topology


def plot_topology(out_dir: Path, seed: int = 0) -> None:
    scenarios = case_data.TOPOLOGY_SCENARIOS
    # 5 个 3D 子图挤在一行里会被拉得极扁，轨迹线根本分不开，
    # 所以改成 2x3 网格（最后一格空着），每格都是近乎正方形的。
    fig = plt.figure(figsize=(15.0, 10.0))
    for n, sc in enumerate(scenarios, start=1):
        rng = np.random.default_rng(seed)
        try:
            graph = exercise2.build_prm(
                sc.world, sc.start, sc.goal, sc.num_samples, rng
            )
            paths = exercise2.enumerate_paths(graph)
            taus = exercise2.distinct_trajectories(paths, graph, sc.world)
        except NotImplementedError:
            graph, paths, taus = Graph(sc.world), [], []
        expect = "n/a" if sc.expected_classes is None else str(sc.expected_classes)
        ok = sc.expected_classes is None or len(taus) == sc.expected_classes
        ax = _ax3d(
            fig,
            2,
            3,
            n,
            f"{n}. {sc.name}\n{len(paths)} paths -> {len(taus)} classes "
            f"(expect {expect}) [{_mark(ok)}]",
            compact=False,
        )
        _draw_obstacles(ax, sc.world, sc.goal.t)
        _draw_graph(ax, graph, sc.world, labels=False)
        for k, tau in enumerate(taus):
            pts = np.array([_xyz(tau.at(i / 120), sc.world) for i in range(121)])
            ax.plot(
                pts[:, 0],
                pts[:, 1],
                pts[:, 2],
                color=PALETTE[k % len(PALETTE)],
                lw=3.0,
                zorder=8,
            )
        _autoscale(ax, sc.world, [sc.start, sc.goal], sc.goal.t)
    fig.suptitle(
        f"stage topology - end to end (seed={seed}): thick coloured lines "
        "are the distinct topology classes",
        fontsize=11,
    )
    fig.tight_layout(rect=(0, 0, 1, 0.95))
    _save(fig, out_dir, "cases_topology.png")


STAGES = {
    "edge": plot_edge,
    "sample": plot_sample,
    "visible": plot_visible,
    "add": plot_add,
    "dfs": plot_dfs,
    "topology": plot_topology,
}


def main() -> int:
    parser = argparse.ArgumentParser(description="把 check2.py 的用例画出来")
    parser.add_argument("--stage", choices=[*STAGES, "all"], default="all")
    parser.add_argument(
        "--out", type=Path, default=Path(__file__).resolve().parent / "figures"
    )
    parser.add_argument("--seed", type=int, default=0)
    args = parser.parse_args()

    names = list(STAGES) if args.stage == "all" else [args.stage]
    for name in names:
        print(f"[{name}]")
        fn = STAGES[name]
        if name in ("sample", "topology"):
            fn(args.out, seed=args.seed)
        else:
            fn(args.out)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
