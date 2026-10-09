#!/usr/bin/env python3
"""阶段 3 的判卷脚本。不需要修改。

python globally_guided/check3.py --stage advance
python globally_guided/check3.py --stage shift
python globally_guided/check3.py --stage carry
python globally_guided/check3.py --stage reintro
python globally_guided/check3.py --stage warm
python globally_guided/check3.py --stage guardmap
python globally_guided/check3.py --stage segid
python globally_guided/check3.py --stage trajid
python globally_guided/check3.py --stage select
python globally_guided/check3.py --stage closed
python globally_guided/check3.py                    # 全跑
python globally_guided/check3.py -v                 # 详细诊断
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

import itertools

import cases3 as case_data
import exercise3
import numpy as np
import tracking
from graph import FixedSampler, Graph, NodeKind
from tracking import IdAllocator
from world import State, Trajectory, World

GREEN, RED, YELLOW, DIM, RESET = (
    "\033[32m",
    "\033[31m",
    "\033[33m",
    "\033[2m",
    "\033[0m",
)
TOL = 1e-9
LOOSE = 1e-4


class Skipped(Exception):
    pass


def _mark(ok: bool) -> str:
    return f"{GREEN}PASS{RESET}" if ok else f"{RED}FAIL{RESET}"


def _report(name, ok, detail, description, verbose) -> bool:
    print(f"  {_mark(ok)} {name}")
    if (not ok or verbose) and description:
        print(f"       {DIM}{description}{RESET}")
    if (not ok or verbose) and detail:
        for line in str(detail).splitlines():
            print(f"       {line}")
    return ok


def _guarded(fn, name: str):
    try:
        return True, fn(), ""
    except NotImplementedError as exc:
        print(f"  {YELLOW}SKIP{RESET} {name}: {exc}")
        raise Skipped from None
    except Exception as exc:  # noqa: BLE001
        return False, None, f"抛出了 {type(exc).__name__}: {exc}"


def _same(a: State | None, b: State | None, tol: float = LOOSE) -> bool:
    if a is None or b is None:
        return a is None and b is None
    return tracking.states_close(a, b, tol)


def _invariants(graph: Graph) -> list[str]:
    """阶段 2 的四条图不变量。热启动不应该破坏它们中的任何一条。"""
    problems = []
    for node_id, node in graph.nodes.items():
        kinds = {graph.kind(n) for n in graph.neighbors(node_id)}
        if node.kind in kinds:
            problems.append(f"节点 {node_id} 和同类型的节点相连，图不再是二分的")
        if node.kind is NodeKind.CONNECTOR:
            nb = graph.neighbors(node_id)
            if len(nb) != 2 or len(set(nb)) != 2:
                problems.append(f"connector {node_id} 的度不是 2：{nb}")
    for a, b in graph.edges():
        if not graph.state(a).t < graph.state(b).t:
            problems.append(f"边 {a}-{b} 在时间上不是严格向前的")
    return problems


# ------------------------------------------------------------- advance


def check_advance(verbose: bool) -> tuple[int, int]:
    print("\n[stage advance] advance_world")
    passed = 0
    for case in case_data.ADVANCE_CASES:
        before = [tuple(float(v) for v in o.at(0.0)) for o in case.world.obstacles]
        try:
            ok, out, detail = _guarded(
                lambda c=case: exercise3.advance_world(c.world, c.h), case.name
            )
        except Skipped:
            continue
        if ok:
            problems = []
            if not isinstance(out, World):
                problems.append(f"返回的不是 World，而是 {type(out).__name__}")
            else:
                if not isinstance(out.obstacles, tuple):
                    problems.append("World.obstacles 必须是 tuple")
                if len(out.obstacles) != len(case.expected_positions):
                    problems.append(
                        f"障碍物个数变了：{len(case.expected_positions)} -> "
                        f"{len(out.obstacles)}"
                    )
                else:
                    for i, (obs, exp) in enumerate(
                        zip(out.obstacles, case.expected_positions)
                    ):
                        got = tuple(float(v) for v in obs.at(0.0))
                        if abs(got[0] - exp[0]) > TOL or abs(got[1] - exp[1]) > TOL:
                            problems.append(
                                f"障碍 {i}：期望圆心 ({exp[0]:.4f}, {exp[1]:.4f})，"
                                f"得到 ({got[0]:.4f}, {got[1]:.4f})"
                            )
                        src = case.world.obstacles[i]
                        if abs(float(obs.radius) - float(src.radius)) > TOL:
                            problems.append(f"障碍 {i}：radius 不该变")
                        if (
                            float(
                                np.max(
                                    np.abs(
                                        np.asarray(obs.velocity, dtype=float)
                                        - np.asarray(src.velocity, dtype=float)
                                    )
                                )
                            )
                            > TOL
                        ):
                            problems.append(f"障碍 {i}：velocity 不该变（匀速假设）")
                for field in ("robot_radius", "horizon", "max_velocity", "time_scale"):
                    if getattr(out, field) != getattr(case.world, field):
                        problems.append(f"{field} 被改了，机器人参数和规划口径不该动")
            after = [tuple(float(v) for v in o.at(0.0)) for o in case.world.obstacles]
            if after != before:
                problems.append("原来那个 World 被改了！要造新的，不要原地改")
            ok = not problems
            detail = "\n".join(problems)
        passed += _report(case.name, ok, detail, case.description, verbose)
    return passed, len(case_data.ADVANCE_CASES)


# --------------------------------------------------------------- shift


def check_shift(verbose: bool) -> tuple[int, int]:
    print("\n[stage shift] shift_state")
    passed = 0
    for case in case_data.SHIFT_CASES:
        try:
            ok, out, detail = _guarded(
                lambda c=case: exercise3.shift_state(c.state, c.h), case.name
            )
        except Skipped:
            continue
        if ok:
            ok = _same(out, case.expected, TOL)
            detail = (
                f"输入 {case.state}, h={case.h} -> 期望 {case.expected}，得到 {out}"
            )
        passed += _report(case.name, ok, detail, case.description, verbose)
    return passed, len(case_data.SHIFT_CASES)


# --------------------------------------------------------------- carry


def check_carry(verbose: bool) -> tuple[int, int]:
    print("\n[stage carry] carry_connector")
    passed = 0
    for case in case_data.CARRY_CASES:
        graph = case_data.build_graph(case.world, case.guards, case.connectors)
        try:
            ok, out, detail = _guarded(
                lambda c=case, g=graph: exercise3.carry_connector(
                    g, c.connector_id, c.h
                ),
                case.name,
            )
        except Skipped:
            continue
        if ok:
            ok = _same(out, case.expected)
            detail = f"期望 {case.expected}，得到 {out}"
        passed += _report(case.name, ok, detail, case.description, verbose)
    return passed, len(case_data.CARRY_CASES)


# ------------------------------------------------------------- reintro


def check_reintro(verbose: bool) -> tuple[int, int]:
    print("\n[stage reintro] reintroduce_states")
    passed = 0
    for case in case_data.REINTRO_CASES:
        prev = case_data.build_graph(case.world, case.guards, case.connectors)
        try:
            ok, out, detail = _guarded(
                lambda c=case, p=prev: exercise3.reintroduce_states(p, c.world, c.h),
                case.name,
            )
        except Skipped:
            continue
        if ok:
            problems = []
            out = list(out or [])
            remaining = list(case.expected)
            for state in out:
                hit = next((e for e in remaining if _same(state, e)), None)
                if hit is None:
                    problems.append(f"多了一个不该出现的状态：{state}")
                else:
                    remaining.remove(hit)
            for miss in remaining:
                problems.append(f"漏了：{miss}")
            # 顺序：guard 全部在 connector 前面
            guard_states = [prev.state(g) for g in prev.guard_ids() if g not in (0, 1)]
            flags = []
            for state in out:
                origin_guard = any(
                    abs(state.x - g.x) <= LOOSE and abs(state.y - g.y) <= LOOSE
                    for g in guard_states
                )
                flags.append(origin_guard)
            if True in flags and False in flags:  # noqa: SIM102
                if flags.index(False) < len(flags) - 1 - flags[::-1].index(True):
                    problems.append(
                        "顺序错了：所有 guard 必须排在所有 connector 前面。"
                        "connector 先进图的话，它的两个 guard 还不在，"
                        "|L| 会是 0 或 1，于是它被立成 guard 或者被丢掉。"
                    )
            ok = not problems
            detail = "\n".join(problems) or f"得到 {out}"
        passed += _report(case.name, ok, detail, case.description, verbose)
    return passed, len(case_data.REINTRO_CASES)


# ---------------------------------------------------------------- warm


def check_warm(verbose: bool) -> tuple[int, int]:
    print("\n[stage warm] build_prm_warm")
    passed = 0
    for case in case_data.WARM_CASES:
        sampler = FixedSampler(list(case.samples))  # type: ignore
        try:
            ok, graph, detail = _guarded(
                lambda c=case, s=sampler: exercise3.build_prm_warm(
                    c.world,
                    c.start,
                    c.goal,
                    c.num_samples,
                    np.random.default_rng(0),
                    list(c.reintroduced),
                    s,
                ),
                case.name,
            )
        except Skipped:
            continue
        if ok:
            problems = []
            assert graph is not None
            stats = graph.stats()
            if stats["guards"] != case.expected_guards:
                problems.append(
                    f"guard 数：期望 {case.expected_guards}，得到 {stats['guards']}"
                )
            if stats["connectors"] != case.expected_connectors:
                problems.append(
                    f"connector 数：期望 {case.expected_connectors}，"
                    f"得到 {stats['connectors']}"
                )
            if 0 not in graph.nodes or not _same(graph.state(0), case.start, TOL):
                problems.append("节点 0 必须是 start（先加 start 再加 goal）")
            if 1 not in graph.nodes or not _same(graph.state(1), case.goal, TOL):
                problems.append("节点 1 必须是 goal")
            problems += _invariants(graph)
            ok = not problems
            detail = "\n".join(problems) or f"{stats}"
        passed += _report(case.name, ok, detail, case.description, verbose)
    return passed, len(case_data.WARM_CASES)


# ------------------------------------------------------------ guardmap


def check_guardmap(verbose: bool) -> tuple[int, int]:
    print("\n[stage guardmap] match_guards")
    passed = 0
    for case in case_data.GUARDMAP_CASES:
        prev = case_data.build_graph(case.world, case.prev_guards)
        new = case_data.build_graph(case.world, case.new_guards)
        try:
            ok, out, detail = _guarded(
                lambda c=case, p=prev, n=new: exercise3.match_guards(p, n, c.h),
                case.name,
            )
        except Skipped:
            continue
        if ok:
            ok = dict(out or {}) == case.expected
            detail = f"期望 {case.expected}，得到 {dict(out or {})}"
        passed += _report(case.name, ok, detail, case.description, verbose)
    return passed, len(case_data.GUARDMAP_CASES)


# --------------------------------------------------------------- segid


def check_segid(verbose: bool) -> tuple[int, int]:
    print("\n[stage segid] assign_segment_ids")
    passed = 0
    for case in case_data.SEGID_CASES:
        prev = (
            None
            if case.prev_guards is None
            else case_data.build_graph(
                case.world, case.prev_guards, case.prev_connectors
            )
        )
        new = case_data.build_graph(case.world, case.new_guards, case.new_connectors)
        allocator = IdAllocator(10)
        try:
            ok, out, detail = _guarded(
                lambda c=case, p=prev, n=new, a=allocator: exercise3.assign_segment_ids(
                    n,
                    p,
                    {} if p is None else exercise3.match_guards(p, n, c.h),
                    c.prev_segment_ids,
                    a,
                    c.world,
                    c.h,
                ),
                case.name,
            )
        except Skipped:
            continue
        if ok:
            out = dict(out or {})
            problems = []
            if out != case.expected:
                problems.append(f"期望 {case.expected}，得到 {out}")
            if len(set(out.values())) != len(out):
                problems.append("同一帧里两个 connector 拿到了相同的 alpha")
            ok = not problems
            detail = "\n".join(problems)
        passed += _report(case.name, ok, detail, case.description, verbose)
    return passed, len(case_data.SEGID_CASES)


# -------------------------------------------------------------- trajid


def check_trajid(verbose: bool) -> tuple[int, int]:
    print("\n[stage trajid] assign_trajectory_ids")
    passed = 0
    for case in case_data.TRAJID_CASES:
        allocator = IdAllocator(10)
        try:
            ok, out, detail = _guarded(
                lambda c=case, a=allocator: exercise3.assign_trajectory_ids(
                    [list(p) for p in c.paths], c.segment_ids, c.prev_map, a
                ),
                case.name,
            )
        except Skipped:
            continue
        if ok:
            out = list(out or [])
            ok = out == list(case.expected)
            detail = f"期望 {list(case.expected)}，得到 {out}"
        passed += _report(case.name, ok, detail, case.description, verbose)
    return passed, len(case_data.TRAJID_CASES)


# -------------------------------------------------------------- select


def _straight(length: float) -> Trajectory:
    return Trajectory(states=[State(0.0, 0.0, 0.0), State(length, 0.0, 0.0)])


def check_select(verbose: bool) -> tuple[int, int]:
    print("\n[stage select] select_guidance")
    passed = 0
    for case in case_data.SELECT_CASES:
        trajs = [_straight(v) for v in case.lengths]
        try:
            ok, out, detail = _guarded(
                lambda c=case, t=trajs: exercise3.select_guidance(
                    t, list(c.trajectory_ids), c.previous_id, c.penalty
                ),
                case.name,
            )
        except Skipped:
            continue
        if ok:
            ok = out == case.expected
            detail = (
                f"长度 {list(case.lengths)}, beta {list(case.trajectory_ids)}, "
                f"上帧 {case.previous_id}, 惩罚 {case.penalty}\n"
                f"期望下标 {case.expected}，得到 {out}"
            )
        passed += _report(case.name, ok, detail, case.description, verbose)
    return passed, len(case_data.SELECT_CASES)


# -------------------------------------------------------------- closed


def _run_scenario(scenario, seed: int):
    return tracking.run_frames(
        scenario.world,
        case_data.START,
        case_data.GOAL_XY,
        scenario.frames,
        case_data.H,
        scenario.num_samples,
        np.random.default_rng(seed),
        consistency_penalty=scenario.penalty,
        warm=scenario.warm,
    )


def _frame_problems(frames) -> list[str]:
    problems = []
    for i, frame in enumerate(frames):
        betas = frame.trajectory_ids
        if len(set(betas)) != len(betas):
            problems.append(f"帧 {i}：同一帧里两条轨迹拿到了相同的 beta {betas}")
        alphas = list(frame.segment_ids.values())
        if len(set(alphas)) != len(alphas):
            problems.append(f"帧 {i}：同一帧里两座桥拿到了相同的 alpha {alphas}")
        if frame.chosen is not None and frame.chosen not in betas:
            problems.append(f"帧 {i}：chosen={frame.chosen} 不在 {betas} 里")
        problems += [f"帧 {i}：{m}" for m in _invariants(frame.graph)]
    return problems


def check_closed(verbose: bool) -> tuple[int, int]:
    print("\n[stage closed] 多帧闭环的时序断言")
    scored = [s for s in case_data.CLOSED_SCENARIOS if s.max_switches is not None]
    passed = 0
    for scenario in case_data.CLOSED_SCENARIOS:
        try:
            ok, _, detail = _guarded(
                lambda s=scenario: _run_scenario(s, 0), scenario.name
            )
        except Skipped:
            continue
        switches: list[int] = []
        problems: list[str] = []
        if ok:
            for seed in range(scenario.seeds):
                frames = _run_scenario(scenario, seed)
                chosen = [f.chosen for f in frames]
                switches.append(sum(1 for a, b in itertools.pairwise(chosen) if a != b))
                problems += [f"seed={seed} {m}" for m in _frame_problems(frames)]
            if scenario.max_switches is None:
                print(
                    f"  {YELLOW}OBS {RESET} {scenario.name}: 每个种子的切换次数 {switches}"
                )
                if verbose:
                    print(f"       {DIM}{scenario.description}{RESET}")
                continue
            worst = max(switches)
            if worst > scenario.max_switches:
                problems.insert(
                    0,
                    f"chosen 最多允许变 {scenario.max_switches} 次，"
                    f"实测最差的种子变了 {worst} 次；各种子 {switches}",
                )
            ok = not problems
            detail = "\n".join(dict.fromkeys(problems[:6])) or (
                f"{scenario.frames} 帧 x {scenario.seeds} 种子，切换次数 {switches}"
            )
        elif scenario.max_switches is None:
            print(f"  {RED}OBS {RESET} {scenario.name}: {detail}")
            continue
        passed += _report(scenario.name, ok, detail, scenario.description, verbose)
    return passed, len(scored)


# ----------------------------------------------------------------- cli

STAGES = {
    "advance": check_advance,
    "shift": check_shift,
    "carry": check_carry,
    "reintro": check_reintro,
    "warm": check_warm,
    "guardmap": check_guardmap,
    "segid": check_segid,
    "trajid": check_trajid,
    "select": check_select,
    "closed": check_closed,
}


def main() -> int:
    parser = argparse.ArgumentParser(description="阶段 3 判卷")
    parser.add_argument("--stage", choices=[*STAGES, "all"], default="all")
    parser.add_argument("-v", "--verbose", action="store_true")
    args = parser.parse_args()

    wanted = list(STAGES) if args.stage == "all" else [args.stage]
    results = {name: STAGES[name](args.verbose) for name in wanted}

    print("\n" + "-" * 46)
    total_pass = total = 0
    for name, (good, count) in results.items():
        colour = GREEN if good == count else RED
        print(f"  {name:<12} {colour}{good}/{count}{RESET}")
        total_pass += good
        total += count
    print("-" * 46)
    if total_pass == total:
        print(f"{GREEN}全部通过。阶段 3 完成，引导层现在有记忆了。{RESET}")
        return 0
    print(f"{RED}还有 {total - total_pass} 个用例没过。{RESET}")
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
