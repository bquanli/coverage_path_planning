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


今日目标：
1. reference_line 中增加对障碍物的权重
2. 增加历史轨迹的权重
3. 整理一下当前的一些问题，以及解决思路


之前有遇到这样一个问题，需要让搜索的参考线包含有道路中心线，也就是l=0，自己的想法是额外的去找到这个。。。但是更好的理解应该是让最小的哪个直接为0！

查看cmake中target的名字：
/workspace/nvq_sim ❯ find /opt/sdk/Fields2Cover -name 'Fields2CoverConfig.cmake' -o -name 'Fields2CoverTargets.cmake'
/opt/sdk/Fields2Cover/lib/cmake/Fields2Cover/Fields2CoverTargets.cmake
/opt/sdk/Fields2Cover/lib/cmake/Fields2Cover/Fields2CoverConfig.cmake
/workspace/nvq_sim ❯ grep -R "add_library" /opt/sdk/Fields2Cover/lib/cmake/Fields2Cover
/opt/sdk/Fields2Cover/lib/cmake/Fields2Cover/Fields2CoverTargets.cmake:add_library(Fields2Cover::Fields2Cover SHARED IMPORTED)
/opt/sdk/Fields2Cover/lib/cmake/Fields2Cover/Fields2CoverTargets.cmake:add_library(Fields2Cover::steering_functions SHARED IMPORTED)
/opt/sdk/Fields2Cover/lib/cmake/Fields2Cover/Fields2CoverTargets.cmake:add_library(Fields2Cover::matplot SHARED IMPORTED)
/workspace/nvq_sim ❯ find /opt/sdk/Fields2Cover /opt/sdk/Fields2CoverSources -name 'libortools.so*' 2>/dev/null
/opt/sdk/Fields2Cover/lib/libortools.so.9
/opt/sdk/Fields2Cover/lib/libortools.so.9.9.3963
/opt/sdk/Fields2Cover/lib/libortools.so
/opt/sdk/Fields2CoverSources/build/_deps/ortools-src/lib/libortools.so.9
/opt/sdk/Fields2CoverSources/build/_deps/ortools-src/lib/libortools.so.9.9.3963
/opt/sdk/Fields2CoverSources/build/_deps/ortools-src/lib/libortools.so
/workspace/nvq_sim ❯ ldd /workspace/coverage_path_planning/build/Debug/examples/fields2cover/fm | grep -E 'ortools|not found'
        libortools.so.9 => not found
        libsteering_functions.so => not found
        libmatplot.so.1 => not found
/workspace/nvq_sim ❯


---
核心思路一句话：**不要把它当成"求一个函数的最小值"，而要当成"求一个点到一个集合的最近点"**。一旦切换成后一种视角，你要做的事情就变成三步——写成标准形式、判断集合 \(S\) 的结构、再从结构挑工具。下面是我思考这类问题的顺序。

## 第一步：分离"目标"和"可行集"

把问题写成

\[
\min_{x \in \mathbb{R}^2} \ \|x\|^2 \quad \text{s.t.} \quad x \in S
\]

这里目标函数 \(\|x\|^2\) 是固定的、光滑的、凸的，所有的难度**全部**集中在 \(S\) 上。这个分离很重要：以后你换成机器人可行域、换成障碍物约束，目标函数那一半完全不变，你只需要重新回答"\(S\) 长什么样"。所以后面的思考几乎都是在研究 \(S\)。

顺带一句，\(\|x\|^2\) 和 \(\|x\|\) 的最小点是同一个（平方是单调变换），所以你可以自由地在"距离"和"距离平方"之间切换——取平方只是为了可微、好算。

## 第二步：把 \(S\) 问清楚，尤其是"圆"的歧义

你说"\(x_1, x_2\) 是一个圆 \(S\) 中的坐标"，这句话有两种完全不同的含义，而它们的解不一样：

- **圆盘**（实心）：\(S = \{x : \|x - c\| \le r\}\)，这是一个**凸集**；
- **圆周**（只有那条线）：\(S = \{x : \|x - c\| = r\}\)，这是**非凸集**。

为什么必须分清？因为如果圆心 \(c\) 离原点很近（\(\|c\| \le r\)，即原点落在圆内），圆盘情形的答案直接就是 \(x^* = 0\)、最小值 \(0\)；而圆周情形原点根本不在 \(S\) 里，答案是圆周上某一点。更极端地，若 \(c = 0\)，圆周情形下整条圆周都是最优解——解不唯一。**凸集的投影永远唯一，非凸集的投影可能有多个**，这是你判断"要不要担心局部最优"的第一个分水岭。

## 第三步：先用几何和对称性手算，别急着上求解器

这道题有闭式解，而找到它的思路是**对称性**：目标 \(\|x\|^2\) 关于原点旋转对称，约束关于 \(c\) 旋转对称，整个问题只在"原点—圆心"这条直线方向上有结构。所以最优点必然落在原点与圆心的连线上。

同样的结论可以用拉格朗日函数机械地推出来：对 \(L = \|x\|^2 + \lambda(\|x-c\|^2 - r^2)\) 求梯度置零，得 \(x(1+\lambda) = \lambda c\)，也就是 \(x\) 与 \(c\) 共线。两条路通向同一个洞察，而几何那条路更快。

于是对圆盘：原点在圆内时 \(x^* = 0\)；否则沿着 \(c\) 的方向退回到边界，\(x^* = \left(1 - \frac{r}{\|c\|}\right)c\)，最小值 \((\|c\| - r)^2\)。这个"沿着连线走到边界"的图像，就是所有球/球壳投影的通用答案，值得记住。

我建议你在上代码之前一定走一遍这步，不是因为你以后都能手算，而是因为**你需要一个已知正确的答案来校验数值方法**。

## 第四步：用最优性条件来"验货"

怎么知道一个候选点真的是最优？对凸集 \(S\)，\(x^*\) 是原点的投影当且仅当

\[
\langle 0 - x^*,\ y - x^* \rangle \le 0 \quad \forall y \in S
\]

直观说法是：从 \(x^*\) 指向原点的方向，必须是 \(S\) 在 \(x^*\) 处的**外法方向**；换个说法，负梯度 \(-\nabla f(x^*) = -2x^*\) 要落在 \(S\) 在该点的法锥里。几何上就是"最近点处，连线垂直于边界切线"。

这条几何判据比 KKT 的代数形式更耐用：当 \(S\) 是多边形、箱体、或者多个约束的交集时，"垂直"会退化成"落在某个锥里"，而 KKT 乘子的符号条件讲的正是同一件事。

## 第五步：当 \(S\) 复杂起来，方法按阶梯往上爬

这是整个框架里最可复用的部分。遇到一个新的 \(S\)，我按这个顺序问自己：

**(1) 它是不是一个有已知闭式投影的"原子"集合？** 球、箱/区间、半空间、仿射子空间、概率单纯形、二阶锥、低秩矩阵集合，这些都有现成公式。能查表就不要迭代。

**(2) 它凸吗？** 凸的话局部最优即全局最优，而且 \(\min \|x\|^2\) 加上凸约束通常可以整理成一个二次规划或二阶锥规划，丢给求解器一定收敛到全局解，你不需要调初值。非凸的话（圆周、"在障碍物外面"这类 \(\|x - c\| \ge r\) 的约束），同一个求解器只能给你局部解，初值就变成关键变量。

**(3) 如果凸但没有闭式投影，用投影梯度或者通用凸求解器。** 投影梯度的逻辑很简单：沿负梯度走一步，再投回 \(S\)，重复。它把"难的优化"化简成"反复做简单投影"，前提是单个投影要便宜——这就回到了问题 (1)。

**(4) 如果 \(S\) 是多个集合的交集，而每个分量都好投影。** 这时不要去硬求交集的投影，而是用交替投影（Dykstra）或者 ADMM，让每个约束各自投影、再协调。机器人里关节限位、速度限位、平面约束叠在一起，典型就是这种形态。

**(5) 如果非凸。** 三种常规手段：多起点局部求解取最好；把非凸可行域切成若干个凸块、每块解一次凸问题再比较（对多边形障碍物很有效）；或者序列凸化——在当前点把障碍约束线性化成一个半平面，解一个凸子问题，迭代推进。这正是机器人轨迹优化里 sequential convex programming 的做法。

## 第六步：确认这个框架为什么能推广

把原点换成任意参考点 \(x_{\text{ref}}\)，\(\min_{x \in S} \|x - x_{\text{ref}}\|^2\) 就是"离我当前状态最近的可行状态"——这是机器人里的可行性修复、逆运动学的最近解、安全滤波器的共同骨架。如果 \(S\) 换成另一个凸物体，这个问题就变成"两个物体之间的最小距离"，也就是碰撞检测的核心（GJK 算法本质上在做这件事）。所以你现在琢磨的这个二维小问题，结构上和那些工程问题是同一个。

## 一份自检清单

落到代码之后，我会用这几件事确认自己没搞错：先验证边界情况（原点在 \(S\) 内部、\(c = 0\)、\(r = 0\)），再拿数值解和前面手算的闭式解对比，最后在 \(S\) 里随机采一大堆点，看有没有哪个点比你的"最优解"更小——如果有，说明约束写错了或者陷在局部解里。二维问题还有个奢侈的福利：直接画出来，等高线加可行域，对不对一眼就能看见。

---

如果你想，我可以照这个阶梯给你写出可运行的实现：闭式解、拉格朗日/KKT 验证、投影梯度、以及一个把圆换成任意可行域都能跑的通用版本，顺便画图对照。