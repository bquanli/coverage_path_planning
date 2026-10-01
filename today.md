轨迹是一系列 Pose，每个 Pose 都可以用一个刚体变换 Transform 来表示。


    if(i == 0)
    {
      dx = path[1].x() - path[0].x();
      dy = path[1].y() - path[0].y();
      theta[i] = std::atan2(dy, dx);
      tangent_x[i] = dx / accumulated_s_[i + 1] - accumulated_s_[i];
      tangent_y[i] = dy / accumulated_s_[i + 1] - accumulated_s_[i];
      continue;
    }

    if(i + 1 == path.size())
    {
      dx = path[i].x() - path[i - 1].x();
      dy = path[i].y() - path[i - 1].y();
      theta[i] = std::atan2(dy, dx);
      tangent_x[i] = dx / accumulated_s_[i] - accumulated_s_[i - 1];
      tangent_y[i] = dy / accumulated_s_[i] - accumulated_s_[i - 1];
      continue;
    }



        if(i == 0)
    {
      dx = path[1].x() - path[0].x();
      dy = path[1].y() - path[0].y();
      theta[i] = std::atan2(dy, dx);
      // 起点为 0，可以省略
      double ds = accumulated_s_[i + 1];
      tangent_x[i] = dx / ds;
      tangent_y[i] = dy / ds;
      continue;
    }

    if(i + 1 == path.size())
    {
      dx = path[i].x() - path[i - 1].x();
      dy = path[i].y() - path[i - 1].y();
      theta[i] = std::atan2(dy, dx);
      double ds = accumulated_s_[i] - accumulated_s_[i - 1];
      tangent_x[i] = dx / ds;
      tangent_y[i] = dy / ds;
      continue;
    }

    # C++ 大型项目中局部代码的独立验证

对于大型 C++ 项目中的一个计算函数，可以尝试把它放进一个小型验证程序，直接执行真实源码，输入容易判断结果的数据。只要能补齐这部分代码需要的编译和链接依赖，就不必为了验证一个公式而构建整个项目。

本文以 `ReferenceLine` 的边界生成与坐标投影检查为例，说明这种方式的原理、依赖处理思路和验证范围。

## 编译与链接如何决定依赖

C++ 构建通常分为预处理、编译和链接。预处理展开 `#include`；编译器根据声明检查类型并生成目标代码；链接器把目标文件和库组合起来，解析外部符号。

一个 `.cpp` 加上它包含的头文件，经过预处理后构成一个翻译单元。包含某个头文件，并不意味着必须构建该头文件涉及的所有模块。

例如，头文件声明：

```cpp
void writeLog();
double calculateDistance(double x, double y);
```

声明告诉编译器函数的参数与返回类型，但并没有提供实现。如果生成的目标代码引用了这些函数，链接时就需要从其他目标文件或库中找到对应实现；找不到时通常报 `undefined reference`。只声明而没有被引用的普通函数，不会因为声明本身就要求链接它的实现。

需要特别注意：普通链接方式下，一个被直接链接的目标文件里，即使某个函数没有被 `main()` 调用，该函数内部的外部引用仍可能需要解析。不能只检查测试入口调用的函数。链接器的无用节移除或静态库成员选择可能改变这一点，但本次验证没有依靠这些机制消除依赖。

## 如何处理链式依赖

从目标源文件开始，区分每个依赖提供的是声明、头文件实现，还是需要单独链接的实现。

| 依赖形式 | 处理方式 |
|---|---|
| 头文件中的内联函数或可见的模板定义 | 添加正确的头文件搜索路径，由编译器生成所需代码 |
| 实现在其他 `.cpp` 中的必要函数 | 把对应源文件加入编译，或链接已有库 |
| 第三方库中的必要函数 | 使用匹配的头文件、库文件和编译配置 |
| 日志等不影响本次计算的外围函数 | 可以在验证程序中提供同签名的替代实现 |
| 仅声明且没有被引用的接口 | 通常不需要补充实现 |

如果新增的源文件又引入外部引用，就沿这条链继续补齐。依赖范围由实际代码决定，不能保证始终只需一个文件。

替代实现通常称为桩函数。它的目的，是让与验证目标无关的外围行为不再拉入整个运行环境。例如验证几何计算时，可以将日志输出设为空操作；但曲线求解、地图查询、坐标变换等参与结果计算的功能，应使用真实实现，或明确说明替代行为及其对结论的影响。

## 本次 ReferenceLine 验证的依赖处理

验证程序直接包含了被检查的 `reference_line.cpp`，因此执行的是当时磁盘上的计算实现。相关依赖处理如下：

| 依赖 | 本次处理 |
|---|---|
| `ReferenceLine` 的成员函数 | 通过包含目标 `.cpp` 引入实现 |
| `ReferencePoint` 的构造函数与访问函数 | 使用头文件中已有的实现 |
| Eigen 向量及相关运算 | 使用系统 Eigen 头文件中的模板实现 |
| C++ 标准库及数学函数 | 由正常的 `g++` 构建命令处理 |
| 项目日志函数 | 在临时程序中提供空实现，保留相同命名空间和签名 |

修改后的版本增加了路径日志接口，链接时曾报告缺少 `log_global_path` 等符号。验证程序补上了这些日志接口的空实现，没有替换几何算法。

为了查看私有的参考点、边界数组和采样函数，临时程序还在包含目标类头文件时使用了 `#define private public`，随后立即 `#undef private`。标准库与 Eigen 头文件预先包含，避免宏影响它们。这是临时诊断技巧，没有修改项目头文件，也没有改变计算表达式；正式测试更适合使用公共接口或明确的测试访问方式。

## 独立验证程序的大致结构

下面是结构示意，具体日志接口和构建入口需要按被检查版本适配：

```cpp
#include <Eigen/Eigen>
#include <string>
#include "local_planner/planner/lattice/reference_line.h"

// 直接引入目标实现。
#include "local_planner/planner/lattice/reference_line.cpp"

namespace local_planner {
void debug_log(std::string const&) {}
void error_log(std::string const&) {}
// 按链接错误补齐其他与验证计算无关的日志接口。
}

int main() {
    // 构造简单路径，调用实际接口，比较结果与预期值。
}
```

从仓库根目录编译时，命令形式可以是：

```bash
g++ -std=c++17 -O0 \
    -I/usr/include/eigen3 \
    -Ilocal_planner/include -I. \
    /tmp/validation.cpp -o /tmp/validation
```

这里的路径是示例，需要先准备验证程序。直接包含 `.cpp` 后，不要再把同一个 `.cpp` 单独加入编译命令，否则可能出现重复定义。另一种常见方式是让验证程序只包含头文件，再把目标 `.cpp` 作为单独的编译输入；两种方式择一即可。

## 从输入到结论的验证流程

1. 确认正在检查的源码版本与目录，避免测试另一份工作副本。
2. 阅读目标函数、数据结构和调用链，确定需要保留的真实计算依赖。
3. 在临时目录建立最小验证程序，先编译，再按错误补齐头文件路径和外部符号。
4. 选择能够独立判断结果的输入，比较位置、方向、曲率或投影参数，而不只是确认程序没有崩溃。
5. 记录预期值、实际值以及被替代的依赖，明确结论能覆盖哪些行为。
6. 修改源码后重新编译验证程序，再运行用例，避免执行旧二进制。

本次使用的输入包括水平直线、竖直直线、两个点的路径、较短路径，以及弯曲段上的已知采样点。例如对 `(0,0) → (1,0) → (2,0)`，可以直接判断朝向应为零、切向量应为 `(1,0)`、左右各偏移 `0.5 m` 后应位于 `y=±0.5`。

另外，取 Hermite 曲线在已知参数位置的采样点，再投影回去，可以检查采样与投影是否一致。不过两者可能共享错误，因此这种一致性检查应与直线等独立可计算的预期结果结合使用。

## 适用范围与限制

这种方式适合数学计算、几何转换、插值和其他输入输出明确的局部逻辑。它能快速复现具体错误，并验证修改是否解决了这些用例中的问题。

它不能代替完整项目的构建与集成验证。独立程序没有覆盖真实日志系统、业务数据链路、线程交互，也不能证明所有路径上的投影都正确。若正式项目使用不同的宏、编译选项、平台或库版本，还需要评估这些差异是否影响计算结果。

如果目标函数深度依赖地图、服务、设备或大量核心库，应扩大到必要模块的真实构建，或使用项目已有测试环境。独立验证的价值在于控制依赖范围，同时保留要验证的真实计算行为。
---
#include <Eigen/Eigen>
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <iterator>
#include <limits>
#include <sstream>
#include <string>
#include <vector>
#include "local_planner/planner/lattice/reference_point.h"
#include "local_planner/planner/lattice/debug.h"
#define private public
#include "local_planner/planner/lattice/reference_line.h"
#undef private
#ifdef REVIEW_SIM
#include "/workspace/nvq_sim/nvq_navigation/local_planner/planner/lattice/reference_line.cpp"
#define REVIEW_BUILD create_new_line
#else
#include "/workspace/nvq/nvq_navigation/local_planner/planner/lattice/reference_line.cpp"
#define REVIEW_BUILD build
#endif
namespace local_planner {
void debug_log(std::string const&) {}
void error_log(std::string const&) {}
#ifdef REVIEW_SIM
void log_global_path(std::string const&, std::vector<Eigen::Vector3d> const&) {}
void log_lane_boundary(std::string const&, std::vector<Eigen::Vector3d> const&) {}
void log_center_path(std::string const&, std::vector<Eigen::Vector3d> const&) {}
#endif
}

void inspect(char const* name, std::vector<Eigen::Vector3d> const& path) {
  local_planner::ReferenceLine line;
  if (!line.REVIEW_BUILD(path)) { std::cout << name << " build failed\n"; return; }
  std::cout << name << '\n';
  for (auto const& p : line.reference_points_) {
    local_planner::Vec2d left, right;
    line.slToXy({p.sl().s, .5}, &left);
    line.slToXy({p.sl().s, -.5}, &right);
    std::cout << "  s=" << p.sl().s << " theta=" << p.theta()
              << " kappa=" << p.kappa() << " tangent=(" << p.tangent_x()
              << ',' << p.tangent_y() << ") left=(" << left.x << ',' << left.y
              << ") right=(" << right.x << ',' << right.y << ")\n";
  }
  if (!line.left_boundary_sl_.empty()) {
    std::cout << "  debug half-width="
              << (line.left_boundary_sl_.front().head<2>() -
                  line.center_line_.front().head<2>()).norm() << '\n';
  }
}

int main() {
  std::cout << std::fixed << std::setprecision(6);
  inspect("horizontal 3 points", {{0,0,0},{1,0,0},{2,0,0}});
  inspect("horizontal 4 points at 0.2m", {{0,0,0},{.2,0,0},{.4,0,0},{.6,0,0}});
  inspect("vertical 3 points", {{0,0,0},{0,1,0},{0,2,0}});
  inspect("horizontal 2 points", {{0,0,0},{1,0,0}});
  local_planner::ReferencePoint p({0,0},0,{0,0},0,0,1,0);
  std::cout << "constructor tangent input=(1,0) stored=("
            << p.tangent_x() << ',' << p.tangent_y() << ")\n";
  local_planner::ReferenceLine line;
  line.REVIEW_BUILD({{0,0,0},{.2,0,0},{.4,0,0}});
  auto a = line.sampleReference(0,0);
  auto b = line.sampleReference(0,1);
  std::cout << "Hermite segment [0,.2] sample ratio=0 -> (" << a.x << ',' << a.y
            << "), ratio=1 -> (" << b.x << ',' << b.y << ")\n";
  line.REVIEW_BUILD({{0,0,0},{1,0,0},{2,0,0}});
  auto c = line.sampleReference(1,.5);
  local_planner::SLPoint projected;
  bool ok = line.xyToSl(local_planner::Vec2d{1.5,0}, &projected);
  std::cout << "last horizontal segment midpoint=(" << c.x << ',' << c.y << ")\n";
  std::cout << "xyToSl(1.5,0): success=" << ok;
  if(ok) std::cout << " s=" << projected.s << " l=" << projected.l;
  std::cout << '\n';
  line.REVIEW_BUILD({{0,0,0},{.0001,0,0}});
  projected = local_planner::SLPoint{-123,-456};
  ok = line.xyToSl(local_planner::Vec2d{.00005,0}, &projected);
  std::cout << "short valid path xyToSl: success=" << ok
            << " s=" << projected.s << " l=" << projected.l << '\n';
  line.REVIEW_BUILD({{0,0,0},{1,0,0},{1,1,0}});
  auto curve = line.sampleReference(1,.5);
  projected = local_planner::SLPoint{-123,-456};
  ok = line.xyToSl(local_planner::Vec2d{curve.x,curve.y}, &projected);
  std::cout << "exact Hermite midpoint (" << curve.x << ',' << curve.y
            << ") expected s=1.5 l=0: success=" << ok
            << " s=" << projected.s << " l=" << projected.l << '\n';
}



---

# 算法的设计与实现

对于算法的设计与实现,还是有很大的区别的!!!
在设计算法时：
$$\text{定义 }J\;\longrightarrow\;\text{推导 }\nabla J\;\longrightarrow\;\text{写出更新公式}$$
在这份代码运行时：
$$\text{当前轨迹}\;\longrightarrow\;\text{计算梯度}\;\longrightarrow\;\text{更新并投影}$$




应该转换思路,对于中心差分,不应该考虑每个点的差分怎样计算,也就是 g[i] = w1 + w2 + w3,因为在计算时,只能计算每个 w,这时,我们是将每个 w 在赋值给三个 gi-1, gi, gi+1


一个自己没有注意到的细节:max|dx| 和 max|dy| 都是 0.012641，完全对称——L 形路径本来就该对称!!!!!


最值得学的一个设计决定：公开可观测量


循环条件写成 i + 1 < n 而不是 i < n - 1，对无符号类型天然安全，直接消灭了你第一轮遇到的那类下溢崩溃。整条语句用 Eigen::Vector2d 而不是分别操作 .x() 和 .y()，x/y 写漏或写重这种 bug 在语法层面就不可能发生了——这比"小心一点"可靠得多。

就像 Points3D 能被 3D 视图显示，Scalars 能被 Time Series 视图显示。不同的 archetype 为相应的视图提供它能够理解的数据。
不同的类型,对于不同的试图来说,可显示性是不同的!


从变量名开始往外读,右结合性.


所以你的理解方向是对的，但建议把概念区分开：
- 表达式里的 a = b = c：涉及运算符的右结合性。
- T const* const p：主要涉及 C++ 声明符的语法和优先级。
- “从变量名向外读”：是解析复杂声明的一种实用方法。
如果只看 const，还有一个更简单的口诀：
const 默认修饰它左边的东西；左边没有东西时修饰右边。