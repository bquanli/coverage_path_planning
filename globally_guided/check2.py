#!/usr/bin/env python3
"""阶段 2 的判卷脚本。不需要修改。

python globally_guided/check2.py --stage edge
python globally_guided/check2.py --stage sample
python globally_guided/check2.py --stage visible
python globally_guided/check2.py --stage add
python globally_guided/check2.py --stage dfs
python globally_guided/check2.py --stage invariant
python globally_guided/check2.py --stage topology
python globally_guided/check2.py                    # 全跑
python globally_guided/check2.py -v                 # 详细诊断
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

import cases2 as case_data
import exercise
import exercise2
import numpy as np
from graph import FixedSampler, Graph, NodeKind
from world import State, Trajectory, World

GREEN, RED, YELLOW, DIM, RESET = (
    "\033[32m",
    "\033[31m",
    "\033[33m",
    "\033[2m",
    "\033[0m",
)
TOL = 1e-9


class Skipped(Exception):
    pass


def _mark(ok: bool) -> str:
    return f"{GREEN}PASS{RESET}" if ok else f"{RED}FAIL{RESET}"


def _report(name: str, ok: bool, detail: str, description: str, verbose: bool) -> bool:
    print(f"  {_mark(ok)} {name}")
    if (not ok or verbose) and description:
        print(f"       {DIM}{description}{RESET}")
    if (not ok or verbose) and detail:
        for line in detail.splitlines():
            print(f"       {line}")
    return ok


def _guarded(fn, name: str):
    """把 NotImplementedError 转成 SKIP，其他异常转成 FAIL。"""
    try:
        return True, fn(), ""
    except NotImplementedError as exc:
        print(f"  {YELLOW}SKIP{RESET} {name}: {exc}")
        raise Skipped from None
    except Exception as exc:  # noqa: BLE001
        return False, None, f"抛出了 {type(exc).__name__}: {exc}"


def _fresh_graph(world: World, guards: tuple[State, ...]) -> Graph:
    graph = Graph(world)
    for state in guards:
        graph.add_guard(state)
    return graph


# ---------------------------------------------------------------- edge


def check_edge(verbose: bool) -> tuple[int, int]:
    print("\n[stage edge] edge_feasible")
    passed = 0
    for case in case_data.EDGE_CASES:
        try:
            ok, actual, detail = _guarded(
                lambda c=case: exercise2.edge_feasible(c.p, c.q, c.world), case.name
            )
        except Skipped:
            continue
        if ok:
            if not isinstance(actual, bool):
                ok, detail = False, f"应该返回 bool，实际返回了 {type(actual).__name__}"
            else:
                ok = actual == case.expected
                detail = f"期望 {case.expected}，实际 {actual}"
        passed += _report(case.name, ok, detail, case.description, verbose)
    return passed, len(case_data.EDGE_CASES)


# -------------------------------------------------------------- sample


def check_sample(verbose: bool, draws: int = 300) -> tuple[int, int]:
    print("\n[stage sample] sample_state 的性质测试")
    passed = 0
    for scenario in case_data.SAMPLE_SCENARIOS:
        world, start, goal = scenario.world, scenario.start, scenario.goal
        rng = np.random.default_rng(0)
        try:
            ok, _, detail = _guarded(
                lambda: exercise2.sample_state(world, start, goal, rng), scenario.name
            )
        except Skipped:
            continue
        problems: list[str] = []
        produced = 0
        if ok:
            rng = np.random.default_rng(0)
            for _ in range(draws):
                try:
                    x = exercise2.sample_state(world, start, goal, rng)
                except Exception as exc:  # noqa: BLE001
                    problems.append(f"抛出了 {type(exc).__name__}: {exc}")
                    break
                if x is None:
                    continue
                produced += 1
                if not isinstance(x, State):
                    problems.append(f"返回了 {type(x).__name__}，应该是 State 或 None")
                    break
                if not (start.t - TOL <= x.t <= goal.t + TOL):
                    problems.append(f"t={x.t:.3f} 跑出了 [{start.t}, {goal.t}]")
                reach = world.max_velocity * (x.t - start.t) + TOL
                if float(np.hypot(x.x - start.x, x.y - start.y)) > reach:
                    problems.append(f"{x} 从起点根本赶不到")
                back = world.max_velocity * (goal.t - x.t) + TOL
                if float(np.hypot(x.x - goal.x, x.y - goal.y)) > back:
                    problems.append(f"{x} 到不了终点")
                if not exercise2.state_collision_free(x, world):
                    problems.append(f"{x} 自身就在障碍里")
                if problems:
                    break
            if produced == 0 and not problems:
                problems.append(f"{draws} 次采样全返回了 None，采样区域恐怕弄错了")
            ok = not problems
            detail = "\n".join(problems[:4]) or f"{produced}/{draws} 个样本全部合法"
        passed += _report(scenario.name, ok, detail, scenario.description, verbose)
    return passed, len(case_data.SAMPLE_SCENARIOS)


# ------------------------------------------------------------- visible


def check_visible(verbose: bool) -> tuple[int, int]:
    print("\n[stage visible] visible_guards")
    passed = 0
    for case in case_data.VISIBLE_CASES:
        graph = _fresh_graph(case.world, case.guards)
        try:
            ok, actual, detail = _guarded(
                lambda: exercise2.visible_guards(case.x, graph, case.world), case.name
            )
        except Skipped:
            continue
        if ok:
            try:
                got = tuple(sorted(actual))  # type: ignore
            except TypeError:
                ok, detail = False, f"应该返回可迭代的 id，实际是 {actual!r}"
            else:
                ok = got == case.expected_indices
                detail = (
                    f"期望看见 guard {list(case.expected_indices)}，实际 {list(got)}"
                )
        passed += _report(case.name, ok, detail, case.description, verbose)
    return passed, len(case_data.VISIBLE_CASES)


# ----------------------------------------------------------------- add


def _state_multiset(states) -> list[tuple[float, float, float]]:
    return sorted((round(s.x, 6), round(s.y, 6), round(s.t, 6)) for s in states)


def check_add(verbose: bool) -> tuple[int, int]:
    print("\n[stage add] try_add_sample")
    passed = 0
    for case in case_data.ADD_CASES:
        graph = _fresh_graph(
            case.world, (case_data.START, case_data.GOAL, *case.extra_guards)
        )

        def run(c=case, g=graph):
            for sample in c.samples:
                exercise2.try_add_sample(sample, g, c.world)
            return g.stats()

        try:
            ok, stats, detail = _guarded(run, case.name)
        except Skipped:
            continue
        if ok:
            problems = []
            if stats["guards"] != case.expected_guards:
                problems.append(
                    f"guard 数：期望 {case.expected_guards}，实际 {stats['guards']}"
                )
            if stats["connectors"] != case.expected_connectors:
                problems.append(
                    f"connector 数：期望 {case.expected_connectors}，实际 {stats['connectors']}"
                )
            if case.expected_connector_states is not None:
                want = _state_multiset(case.expected_connector_states)
                got = _state_multiset(graph.state(i) for i in graph.connector_ids())
                if want != got:
                    problems.append(f"connector 坐标：期望 {want}\n实际 {got}")
            ok = not problems
            detail = "\n".join(problems) or f"{stats}"
        passed += _report(case.name, ok, detail, case.description, verbose)
    return passed, len(case_data.ADD_CASES)


# ----------------------------------------------------------------- dfs


def check_dfs(verbose: bool) -> tuple[int, int]:
    print("\n[stage dfs] enumerate_paths")
    passed = 0
    for case in case_data.DFS_CASES:
        graph = _fresh_graph(case_data.EMPTY, case.guards)
        for state, g_a, g_b in case.connectors:
            graph.add_connector(state, g_a, g_b)
        try:
            ok, actual, detail = _guarded(
                lambda: exercise2.enumerate_paths(graph), case.name
            )
        except Skipped:
            continue
        if ok:
            try:
                got = sorted(tuple(p) for p in actual)
            except TypeError:
                ok, detail = False, f"应该返回 list[list[int]]，实际是 {actual!r}"
            else:
                want = sorted(case.expected_paths)
                ok = got == want
                detail = f"期望 {want}\n实际 {got}"
        passed += _report(case.name, ok, detail, case.description, verbose)
    return passed, len(case_data.DFS_CASES)


# ----------------------------------------------------------- invariant


def _check_invariants(
    graph: Graph, world: World, seed_pair: tuple[int, int]
) -> list[str]:
    bad: list[str] = []
    for node_id, node in graph.nodes.items():
        neighbours = graph.neighbors(node_id)
        if node.kind is NodeKind.CONNECTOR:
            if len(neighbours) != 2:
                bad.append(
                    f"不变量 2 破了：connector {node_id} 的度是 {len(neighbours)}"
                )
            if any(graph.kind(n) is not NodeKind.GUARD for n in neighbours):
                bad.append(f"不变量 1 破了：connector {node_id} 接到了非 guard 节点")
        else:
            if any(graph.kind(n) is not NodeKind.CONNECTOR for n in neighbours):
                bad.append(f"不变量 1 破了：guard {node_id} 接到了非 connector 节点")
    for a, b in graph.edges():
        if not (graph.state(a).t < graph.state(b).t):
            bad.append(f"不变量 4 破了：边 ({a}, {b}) 在时间上不严格向前")
        elif not exercise2.edge_feasible(graph.state(a), graph.state(b), world):
            bad.append(f"边 ({a}, {b}) 并不可行，不该存在于图中")
    guard_ids = graph.guard_ids()
    for i, g_a in enumerate(guard_ids):
        for g_b in guard_ids[i + 1 :]:
            if (g_a, g_b) == seed_pair:
                continue
            if exercise2.edge_feasible(graph.state(g_a), graph.state(g_b), world):
                bad.append(f"不变量 3 破了：guard {g_a} 和 {g_b} 互相可见")
    return bad


def _check_output(
    taus: list[Trajectory], world: World, start: State, goal: State
) -> list[str]:
    bad: list[str] = []
    for tau in taus:
        for p, q in zip(tau.states, tau.states[1:]):
            if not exercise.connection_valid(p, q, world):
                bad.append(f"输出轨迹里有不可行的一段：{p} -> {q}")
            if not exercise.segment_collision_free(p, q, world):
                bad.append(f"输出轨迹里有撞上的一段：{p} -> {q}")
        if (tau.start.x, tau.start.y, tau.start.t) != (start.x, start.y, start.t):
            bad.append(f"轨迹起点不对：{tau.start}")
        if (tau.goal.x, tau.goal.y, tau.goal.t) != (goal.x, goal.y, goal.t):
            bad.append(f"轨迹终点不对：{tau.goal}")
    for i, tau_i in enumerate(taus):
        for tau_j in taus[i + 1 :]:
            if exercise.uvd_equivalent(tau_i, tau_j, world):
                bad.append("输出里有两条 UVD 等价的轨迹，去重没做干净")
    return bad


def _run_once(scenario, seed: int):
    rng = np.random.default_rng(seed)
    graph = exercise2.build_prm(
        scenario.world, scenario.start, scenario.goal, scenario.num_samples, rng
    )
    paths = exercise2.enumerate_paths(graph)
    taus = exercise2.distinct_trajectories(paths, graph, scenario.world)
    return graph, paths, taus


def check_invariant(verbose: bool, seeds: int = 5) -> tuple[int, int]:
    print(f"\n[stage invariant] build_prm 的图不变量 + 输出合法性（{seeds} 个种子）")
    passed = 0
    for scenario in case_data.TOPOLOGY_SCENARIOS:
        try:
            ok, _, detail = _guarded(lambda s=scenario: _run_once(s, 0), scenario.name)
        except Skipped:
            continue
        if ok:
            problems: list[str] = []
            for seed in range(seeds):
                graph, _, taus = _run_once(scenario, seed)
                found = _check_invariants(graph, scenario.world, (0, 1))
                found += _check_output(
                    taus, scenario.world, scenario.start, scenario.goal
                )
                problems += [f"seed={seed}: {m}" for m in found]
                if problems:
                    break
            ok = not problems
            graph, paths, taus = _run_once(scenario, 0)
            detail = "\n".join(dict.fromkeys(problems[:5])) or (
                f"seed=0: {graph.stats()}, 路径 {len(paths)} 条 -> {len(taus)} 类"
            )
        passed += _report(scenario.name, ok, detail, "", verbose)
    return passed, len(case_data.TOPOLOGY_SCENARIOS)


# ------------------------------------------------------------ topology


def check_topology(verbose: bool, seeds: int = 20) -> tuple[int, int]:
    print(f"\n[stage topology] 拓扑类数（{seeds} 个种子）")
    scored = [s for s in case_data.TOPOLOGY_SCENARIOS if s.expected_classes is not None]
    passed = 0
    for scenario in case_data.TOPOLOGY_SCENARIOS:
        try:
            ok, _, detail = _guarded(lambda s=scenario: _run_once(s, 0), scenario.name)
        except Skipped:
            continue
        counts: list[int] = []
        if ok:
            for seed in range(seeds):
                counts.append(len(_run_once(scenario, seed)[2]))
            histogram = {c: counts.count(c) for c in sorted(set(counts))}
            if scenario.expected_classes is None:
                print(f"  {YELLOW}OBS {RESET} {scenario.name}: 类数分布 {histogram}")
                if verbose:
                    print(f"       {DIM}{scenario.description}{RESET}")
                continue
            rate = counts.count(scenario.expected_classes) / len(counts)
            ok = rate >= scenario.min_rate - 1e-9
            detail = (
                f"期望 {scenario.expected_classes} 类，命中率 {rate:.0%}"
                f"（要求 >= {scenario.min_rate:.0%}），分布 {histogram}"
            )
        elif scenario.expected_classes is None:
            print(f"  {RED}OBS {RESET} {scenario.name}: {detail}")
            continue
        passed += _report(scenario.name, ok, detail, scenario.description, verbose)
    return passed, len(scored)


# ----------------------------------------------------------------- cli

STAGES = {
    "edge": check_edge,
    "sample": check_sample,
    "visible": check_visible,
    "add": check_add,
    "dfs": check_dfs,
    "invariant": check_invariant,
    "topology": check_topology,
}


def main() -> int:
    parser = argparse.ArgumentParser(description="阶段 2 判卷")
    parser.add_argument("--stage", choices=[*STAGES, "all"], default="all")
    parser.add_argument("-v", "--verbose", action="store_true")
    parser.add_argument(
        "--seeds", type=int, default=20, help="topology 阶段用多少个种子"
    )
    args = parser.parse_args()

    wanted = list(STAGES) if args.stage == "all" else [args.stage]
    results = {}
    for name in wanted:
        if name == "topology":
            results[name] = STAGES[name](args.verbose, args.seeds)
        else:
            results[name] = STAGES[name](args.verbose)

    print("\n" + "-" * 46)
    total_pass = total = 0
    for name, (good, count) in results.items():
        colour = GREEN if good == count else RED
        print(f"  {name:<12} {colour}{good}/{count}{RESET}")
        total_pass += good
        total += count
    print("-" * 46)
    if total_pass == total:
        print(f"{GREEN}全部通过。阶段 2 完成，全局引导层的骨架已经立起来了。{RESET}")
        return 0
    print(f"{RED}还有 {total - total_pass} 个用例没过。{RESET}")
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
