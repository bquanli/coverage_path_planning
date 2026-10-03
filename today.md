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