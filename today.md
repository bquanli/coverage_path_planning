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