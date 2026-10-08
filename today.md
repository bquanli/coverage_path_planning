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
今日计划:
1. reference_line 的长度和宽度限制
2. 粗略平滑
3. 每个点的宽度的计算，这个要平滑


---

问题：
  if(s <= boundary_.front().s)
  {
    return boundary_.front();
  }

  if(s >= boundary_.back().s)
  {
    return boundary_.back();
  }



https://github.com/Hypha-ROS/hypharos_minicar
https://github.com/Geonhee-LEE/mpc_ros?utm_source=chatgpt.com
https://arxiv.org/pdf/2303.07751
https://github.com/tud-amr/guidance_planner
https://autonomousrobots.nl/
https://github.com/orgs/tud-amr/repositories?page=2


"""你的练习文件：只实现下面两个函数，不需要修改检查器。

推荐先完成 row_intervals，再完成 decompose。
运行 python check.py --stage rows 或 python check.py --stage decompose。
代码中保留类型标注，避免空列表、NumPy dtype 的类型推断不明确。
"""

from __future__ import annotations

import numpy as np
from numpy.typing import NDArray

BoolArray = NDArray[np.bool_]
IntArray = NDArray[np.int64]
Interval = tuple[int, int]  # (left, right)，两端包含
Row = tuple[int, int, int]  # (y, left, right)


# 返回的是，当前行有效的区间
def row_intervals(row: BoolArray) -> list[Interval]:
    # new_row = np.array([0, row.astype(int), 0])
    # concatenate, 保持原来的数组结构，沿某个已有轴把数组接起来(默认是 axis=0)。 注意这里传入的是 [0]
    # np.concatenate 要求的是：除了你指定的拼接轴 axis 之外，其他维度的 shape 必须相同，并且所有输入数组必须有相同的维度数。。
    # 这里需要传入的多个数组的shape相同才可以,自己想的那种情况并不存在
    new_row = np.concatenate(([0], row.astype(int), [0]))

    result = new_row[1:] - new_row[:-1]
    starts = np.where(result == 1)[0]
    ends = np.where(result == -1)[0]
    # 1 是 1 开头的下标的位置, -1 是 0 开头的下标的位置,因此 end 需要减1才能得到正确的 1 的间隔
    intervals = [(int(s), int(e) - 1) for s, e in zip(starts, ends)]

    return intervals


# 对于图问题,自己始终有些疑惑的是不知道该如何表示这些区域或者关系, 就比如下面的这个一个区域,它可能跨越多行, 那它本质上就是一个数组啊,这个数组的每一项,就是它每一行的元素
# 对于上面的连通区域也是,其实就是一个区间!  其实是自己思考方式还是以人的视角来思考，而非是计算机的角度
# 应该总结一下...
# 自己现在还有一个问题是,很难静下心来阅读...对于这个函数,传入什么样的参数,返回什么样的类型,其实都很明确了...没有多大的耐心阅读完...
# 将属于哪个区间和记录结果分开！


ActiveInterval = tuple[int, int, int]


def decompose(free: BoolArray) -> tuple[IntArray, list[list[Row]]]:
    labels = np.full_like(free, -1, dtype=np.int64)
    cells: list[list[Row]] = []
    # enumerate:在遍历一个可迭代对象时，同时给你“索引”和“当前元素”
    # 存放的是上一行的子区域，（cell_id, start, end)
    # 合并：一个子区间对应于多个父区间（父区间仍然只有一个子区间），分裂：一个父区间对应与多个子区间（子区间仍然只有一个父区间）
    # 只有当前区间恰好连接一个父区间，
    # 且该父区间恰好连接一个当前区间时，才延续原区域。
    # 其他情况为当前区间创建新区域。
    previous_intervals: list[ActiveInterval] = []
    for y, row in enumerate(free):
        # cur_inter 存放的是当前行的子区域 (id,start,end)
        current_intervals = row_intervals(row)
        # 存放当前每个区间，有哪些父区间, 存放的是在 previous_intervals 中的下标(index)
        parent_indices: list[list[int]] = [[] for _ in current_intervals]
        # 这个和 previous_intervals 的index 是相同的
        child_counts = [0] * len(previous_intervals)
        # 判断是否相同子区域，不必那么麻烦，只需要逐一判断！
        for current_index, (left, right) in enumerate(current_intervals):
            for previous_index, (_, prev_left, prev_right) in enumerate(
                previous_intervals
            ):
                if max(left, prev_left) <= min(right, prev_right):
                    parent_indices[current_index].append(previous_index)
                    child_counts[previous_index] += 1
        # 更新下一行的 previous
        next_previous: list[ActiveInterval] = []
        for current_index, (left, right) in enumerate(current_intervals):
            parents = parent_indices[current_index]
            cell_id = len(cells)

            if len(parents) == 1:
                parent_index = parents[0]
                if child_counts[parent_index] == 1:
                    cell_id = previous_intervals[parent_index][0]
            if cell_id == len(cells):
                cells.append([])
            cells[cell_id].append((y, left, right))
            next_previous.append((cell_id, left, right))
            labels[y, left : right + 1] = cell_id

        previous_intervals = next_previous

    return labels, cells
