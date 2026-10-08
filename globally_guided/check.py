#!/usr/bin/env python3
"""判卷脚本。不需要修改。

    python globally_guided/check.py --stage segment
    python globally_guided/check.py --stage connection
    python globally_guided/check.py --stage uvd
    python globally_guided/check.py                     # 全跑
    python globally_guided/check.py --stage uvd -v      # 失败时输出每根横档的诊断
"""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

import cases as case_data
import exercise
from world import State, Trajectory, World

GREEN = "\033[32m"
RED = "\033[31m"
YELLOW = "\033[33m"
DIM = "\033[2m"
RESET = "\033[0m"


def _mark(ok: bool) -> str:
    return f"{GREEN}PASS{RESET}" if ok else f"{RED}FAIL{RESET}"


def _label(value: bool, true_text: str, false_text: str) -> str:
    return true_text if value else false_text


def _run_case(name: str, call, expected: bool, true_text: str, false_text: str,
              description: str, verbose: bool) -> bool:
    try:
        actual = call()
    except NotImplementedError as exc:
        print(f"  {YELLOW}SKIP{RESET} {name}: {exc}")
        return False
    except Exception as exc:  # noqa: BLE001 - 练习代码报错要看得见
        print(f"  {RED}FAIL{RESET} {name}: 抛出了 {type(exc).__name__}: {exc}")
        return False

    if not isinstance(actual, bool):
        print(f"  {RED}FAIL{RESET} {name}: 应该返回 bool，实际返回了 {type(actual).__name__}")
        return False

    ok = actual == expected
    print(f"  {_mark(ok)} {name}")
    if not ok or verbose:
        print(f"       {DIM}{description}{RESET}")
        print(f"       期望 {_label(expected, true_text, false_text)}，"
              f"实际 {_label(actual, true_text, false_text)}")
    return ok


def check_segment(verbose: bool) -> tuple[int, int]:
    print("\n[stage segment] segment_collision_free")
    passed = 0
    for case in case_data.SEGMENT_CASES:
        ok = _run_case(
            case.name,
            lambda c=case: exercise.segment_collision_free(c.a, c.b, c.world),
            case.expected, "无碰撞", "碰撞", case.description, verbose,
        )
        passed += ok
    return passed, len(case_data.SEGMENT_CASES)


def check_connection(verbose: bool) -> tuple[int, int]:
    print("\n[stage connection] connection_valid")
    passed = 0
    for case in case_data.CONNECTION_CASES:
        ok = _run_case(
            case.name,
            lambda c=case: exercise.connection_valid(c.a, c.b, c.world),
            case.expected, "可行", "不可行", case.description, verbose,
        )
        passed += ok
    return passed, len(case_data.CONNECTION_CASES)


def _rung_report(tau_1: Trajectory, tau_2: Trajectory, world: World,
                 num_samples: int) -> None:
    """用**你自己的** segment_collision_free 把每根横档列出来，方便定位问题。"""
    print(f"       {DIM}s      tau_1(s)                   tau_2(s)                   横档{RESET}")
    for index in range(num_samples + 1):
        s = index / num_samples
        p = tau_1.at(s)
        q = tau_2.at(s)
        try:
            free = exercise.segment_collision_free(p, q, world)
            verdict = "free" if free else f"{RED}HIT{RESET}"
        except Exception as exc:  # noqa: BLE001
            verdict = f"{RED}{type(exc).__name__}{RESET}"
        print(f"       {s:<6.3f} ({p.x:6.3f},{p.y:7.3f},t={p.t:5.3f})  "
              f"({q.x:6.3f},{q.y:7.3f},t={q.t:5.3f})  {verdict}")


def check_uvd(verbose: bool, num_samples: int) -> tuple[int, int]:
    print(f"\n[stage uvd] uvd_equivalent (num_samples={num_samples})")
    if num_samples < 8:
        print(f"  {YELLOW}注意{RESET}: num_samples 太小时，横档会跨过障碍而漏判。"
              "这不是你的 bug，是 UVD 离散化的固有性质，详见 README。")
    passed = 0
    for case in case_data.UVD_CASES:
        # 前提条件：起点和终点必须重合，否则 UVD 没有定义。
        for attr in ("start", "goal"):
            p = getattr(case.tau_1, attr)
            q = getattr(case.tau_2, attr)
            assert math.isclose(p.x, q.x) and math.isclose(p.y, q.y) \
                and math.isclose(p.t, q.t), f"{case.name}: {attr} 不重合"

        ok = _run_case(
            case.name,
            lambda c=case: exercise.uvd_equivalent(
                c.tau_1, c.tau_2, c.world, num_samples),
            case.expected, "等价", "不等价", case.description, verbose,
        )
        if not ok and verbose:
            _rung_report(case.tau_1, case.tau_2, case.world, num_samples)
        passed += ok
    return passed, len(case_data.UVD_CASES)


def main() -> int:
    parser = argparse.ArgumentParser(description="阶段 1 验收")
    parser.add_argument("--stage", choices=["segment", "connection", "uvd", "all"],
                        default="all")
    parser.add_argument("-v", "--verbose", action="store_true",
                        help="打印详细诊断（UVD 失败时会列出每根横档）")
    parser.add_argument("--num-samples", type=int, default=20,
                        help="UVD 沿 s 的区间数，默认 20")
    args = parser.parse_args()

    results = []
    if args.stage in ("segment", "all"):
        results.append(("segment", check_segment(args.verbose)))
    if args.stage in ("connection", "all"):
        results.append(("connection", check_connection(args.verbose)))
    if args.stage in ("uvd", "all"):
        results.append(("uvd", check_uvd(args.verbose, args.num_samples)))

    print("\n" + "-" * 46)
    total_passed = total = 0
    for stage, (passed, count) in results:
        total_passed += passed
        total += count
        color = GREEN if passed == count else RED
        print(f"  {stage:<12} {color}{passed}/{count}{RESET}")
    print("-" * 46)

    if total_passed == total:
        print(f"{GREEN}全部通过。阶段 1 完成，可以进入 Visibility-PRM 了。{RESET}")
        return 0
    print(f"{RED}还有 {total - total_passed} 个用例没过。{RESET}")
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
