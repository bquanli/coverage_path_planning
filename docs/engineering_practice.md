# 工程实践指南

以 `path_smoother`（路径平滑）为完整样例，记录这个项目应该怎么写、怎么测、出问题怎么查。

## 1. 分层：算法、示例、测试各归各位

最开始平滑算法写在 `examples/rerun/smooth_line.cc` 里，这带来三个连带后果：没法被
其他模块复用、没法单独测试（项目里已经出现了 `#include "../examples/rerun/reference_line.cc"`
加宏开关这种写法）、改动时只能启动 rerun 用肆眼验收。

现在的分层：

| 路径 | 职责 |
| --- | --- |
| `include/coverage_path_planning/planning/path_smoother.hh` | 公开 API、数学定义、异常契约 |
| `src/planning/path_smoother.cc` | 算法实现，不包含任何 I/O 与可视化 |
| `tests/path_smoother_test.cc` | 用不变量锁住行为，回归的唯一保障 |
| `examples/rerun/smooth_line.cc` | 瞄客户端：准备数据、调库、打印诊断、画图 |

两条经验规则：

- **库函数不打印。** 调试信息要通过返回值交出去（`SmoothResult` 里的
  `iterations` / `max_step` / `converged`），而不是 `std::cout`。谁调用谁决定要不要显示。
- **返回值不允许假数据。** 把 `iterations` 写成常量、`max_step` 永远为 0，等于主动
  没收了自己排查问题的眼睛。字段宁可不要，也不要给错的。

## 2. 公开可观测量，而不是只公开结果

`path_smoother.hh` 除了 `smooth_path`，还公开了两个函数：

- `smoothing_cost`：目标函数 J
- `smoothing_gradient`：解析梯度

这不是泄露实现细节，而是**为可测试性做的有意设计**。一个迭代优化器如果只能看到
最终结果，就只能靠肆眼看图调试；把 J 和梯度暴露出来，就能写出下面这两个
几乎能捕获所有实现 bug 的测试。

## 3. 三个探针：迭代优化类代码的排查顺序

“看到结果不对但不知道从哪里下手”是这类代码的典型困境。按这个顺序做，从便宜的到
贵的，每一步都能砍掉一大类可能性。

### 探针一：分量级别的位移统计

分开打印 `max|dx|` 和 `max|dy|`，不要合起来打 norm。合着打只能看到“有点动”，
分开打才能看到“某一个分量根本没动”。

**精确的 `0.000000` 永远值得追查**——它意味着某个赋值路径根本没走到，而不是数值
算得不好。看到它就不要再去怀疑步长、权重、迭代次数这些数值问题。

### 探针二：逐轮打印目标函数 J

步长取 `1/L`（L 是梯度的 Lipschitz 常数上界）时，梯度下降在数学上**保证** J 单调不增。
所以 J 不降就是实现错了，这是个不需要动脑的判据。不同失败模式有不同指纹：

| J 的行为 | 含义 |
| --- | --- |
| 单调下降但太慢 | 迭代次数不够，或 tolerance 没接上 |
| 发散 | 步长太大，或 L 估小了 |
| 在两个值之间奇偶振荡 | 双缓冲区 `swap` 时有字段未写全，旧值被换回来了 |

### 探针三：梯度数值校验

写优化代码最值钱的一个工具，已经固定成 `PathSmoother.GradientMatchesCentralDifference`。
用中心差分逼近每个分量的偏导，和解析梯度逐项比。两个必须注意的坑：

- **测试点必须偏离原始位置。** 在 `p == origin` 处贴近项梯度恰好为零，怎么写都能蒙对。
- **`eps` 取 `1e-6` 量级。** 太小会被浮点噪声吃掉，太大截断误差超标。

整数倍的误差（比如算出 1.6 而真值 3.2）是强烈的结构信号：某项被重复累加或漏加，
而不是公式推导错了。

## 4. 崩溃的排查顺序

1. **做最小可复现程序。** 剥掉 rerun / 网络 / GUI，只留算法和固定输入。现在这一步已经
   不需要手工做了，因为算法本来就是独立库，直接写个测试用例就是最小复现。
2. **用 sanitizer 构建，不要用 release 构建。** 见下一节命令。数值循环里的下标 bug，
   ASan 几乎都是一击命中，比 gdb 翻变量快得多。
3. **读报告，而不只看行号。** 要提取三件信息：是读还是写；偏移多少（`16 bytes before`
   这种“恰好负一个元素”的模式，99% 是 `i - 1` 配无符号下标）；那块内存是谁分配的。
4. **区分两种崩溃形态。** SIGSEGV（内存访问违规）和 abort（未捕获异常导致
   `std::terminate`）在终端上都是“崩了”，排查方向完全不同。先找有没有
   `terminating due to uncaught exception` 字样。
5. **最后才人工复核。** 对嵌套索引循环固定查四件事：循环变量和循环体里用的下标
   是不是同一个；所有 `i - 1`、`n - 1` 在无符号下的下界安全；x/y 这种对称分量有没有
   写漏或写重；进函数前有没有校验输入规模。

把循环条件写成 `i + 1 < n` 而不是 `i < n - 1`，对无符号类型天然安全，这个习惯
能直接消灭一类 bug。

## 5. 常用命令

```bash
# 常规构建 + 跑全部测试
cmake -S . -B build/Debug
cmake --build build/Debug -j
ctest --test-dir build/Debug --output-on-failure

# 只重跑某个用例
ctest --test-dir build/Debug -R GradientMatchesCentralDifference

# 开 ASan + UBSan（排查崩溃、越界、未定义行为用这个）
cmake -S . -B build/Debug -DCOVERAGE_ENABLE_SANITIZERS=ON
cmake --build build/Debug -j
ctest --test-dir build/Debug --output-on-failure

# 关掉（sanitizer 构建慢很多，不要常开）
cmake -S . -B build/Debug -DCOVERAGE_ENABLE_SANITIZERS=OFF
```

权限受限的容器里 LeakSanitizer 用不了 ptrace，需要额外加：

```bash
cmake -S . -B build/Debug -DCOVERAGE_ENABLE_SANITIZERS=ON \
  -DCOVERAGE_ASAN_TEST_OPTIONS='detect_container_overflow=0:detect_leaks=0'
```

## 6. 构建系统的几个决定及其理由

- **告警和 sanitizer 走接口目标，不走全局 `CMAKE_CXX_FLAGS`。**
  `coverage_path_planning::warnings` 用 `PRIVATE` 链接（只管自己的编译），
  `coverage_path_planning::sanitizers` 用 `PUBLIC` 链接（编译和链接都要生效，且必须传递
  给可执行文件）。全局 flags 会把告警注入第三方代码，告警列表就永远不可能清零，
  然后整个团队就开始忽视告警。
- **GTest 是可选依赖。** 找不到就跳过测试目标并打 `message(STATUS ...)`，不阻塞主构建。
- **`gtest_discover_tests` 把每个 `TEST` 单独注册成 CTest 用例。** 这样
  `ctest -R <名字>` 能精准重跑单个用例。但 sanitizer 构建下它无法给“测试发现”
  步骤设置环境变量，所以那种模式下退回成单个 `add_test`，细粒度筛选用
  `--gtest_filter`。
- **`detect_container_overflow=0` 是必需的，不是偷懒。** 我们的代码被插桩而系统的
  libstdc++ 与 GTest 没有，混用时该检查会报假阳性。

## 7. 测试是按不变量组织的，不是按函数组织的

`tests/path_smoother_test.cc` 里每个用例对应一条“无论怎么改都必须成立”的性质：

| 用例 | 锁住的不变量 |
| --- | --- |
| `GradientMatchesCentralDifference` | 解析梯度 == 目标函数的数值微分 |
| `CostDecreasesMonotonically` | 步长 1/L 下 J 单调不增 |
| `ConvergesBeforeIterationLimit` | 正常输入下靠 tolerance 停，而不是耗尽上限 |
| `ReportsFailureWhenIterationLimitIsHit` | 没收敛时诚实返回 `converged == false` |
| `OffsetStaysInside*Region` | 约束从不被突破 |
| `ZeroRadiusLeavesPathUnchanged` | 半径为 0 是恒等变换 |
| `FixedEndsStayExactlyInPlace` | 首尾点精确不动 |
| `StraightLineIsFixedPoint` | 已是最优解时原地不动 |
| `Rejects*` | 非法输入抛 `std::invalid_argument`，而不是 UB |

不动点测试（`StraightLineIsFixedPoint`）特别值得学：直线已经是最优解，任何漏写分量、
符号弄反都会把点推走，所以一条断言能拦住很多种 bug。

## 8. 还没做完的部分

- `examples/rerun/reference_line.cc` 里的几何计算仍然在示例文件里，
  `tests/reference_line_geometry_test.cc` 靠 `#include` 那个 `.cc` 加
  `REFERENCE_LINE_GEOMETRY_ONLY` 宏来偷代码，且未接入构建。
  按本文第 1 节的模式搬成 `planning/reference_line.{hh,cc}` + GTest 用例。
- `tests/pdlog_test.cc`、`tests/show_water_leak_test.cc` 同样没有注册到 CTest。
- `thrid_party/` 是 `third_party` 的拼写错误，涉及多处引用，建议单独一个 commit 改名。
