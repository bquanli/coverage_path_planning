# Show MVP：现有功能的简单封装

本文替代此前的完整工具方案，以当前实现为准。目标是拆开原 `tools/show.cc` 的职责，方便阅读和局部修改。仅封装已有能力，继续使用公共 `planning_viz`，采用普通结构体、函数和一个有状态的日志解析类。

## 目录与职责

| 文件 | 职责 |
|---|---|
| `main.cc` | 加载配置和机器人模型，调用回放入口，处理启动异常 |
| `options.hh / options.cc` | `Options`、YAML 配置加载和参数校验 |
| `types.hh` | 共享的 `RobotState`、`Frame` 数据结构 |
| `log_parser.hh / log_parser.cc` | `LogParser`：逐行解析日志，生成位姿帧 |
| `trajectory.hh / trajectory.cc` | 机器人轮廓变换及完整轨迹采样 |
| `replay.hh / replay.cc` | 可视化初始化、回放调度、数据发布、信号停止和关闭 |
| `CMakeLists.txt` | 构建可执行目标 `show` |

调用关系：

```text
main
 ├─ load_options()                  options
 ├─ RobotModelHelper               现有机器人模型
 └─ run(options, footprint)         replay
     ├─ planning_viz::init()
     ├─ LogParser::next_frame()     log_parser → types
     ├─ build_trajectory()          trajectory → types
     ├─ 按时间/帧率循环发布
     │   ├─ odometry()
     │   ├─ make_footprint() → footprint()
     │   └─ trajectory_footprints()
     └─ planning_viz::shutdown()
```

所有工具代码位于 `namespace show`；信号标记和回放辅助函数保留在 `replay.cc` 的匿名命名空间中。解析和几何模块不依赖可视化会话。

## 数据与接口

`RobotState` 保存 xyz、yaw 及可选线速度/角速度。

`Frame` 保存机器人状态、日志时间和这一帧合并的速度记录数量。日志记录是否为位姿的标记属于解析器内部 `Event`，不暴露给轨迹和回放模块。

```cpp
Options load_options();

class LogParser {
public:
  explicit LogParser(std::filesystem::path const& path);
  bool next_frame(Frame& frame);
};

using Polygon = std::vector<Eigen::Vector3d>;

Polygon make_footprint(RobotState const& state,
                       coverage_path_planning::Footprint const& footprint);

std::vector<Polygon> build_trajectory(
    std::span<Frame const> frames,
    coverage_path_planning::Footprint const& footprint,
    double distance_step_m,
    double yaw_step_deg);

int run(Options const& options,
        coverage_path_planning::Footprint const& footprint);
```

以上为接口摘要；实际声明见对应头文件。`build_trajectory()` 返回拥有顶点数据的 polygon 集合。回放中用于发布的 span 只引用这份集合，集合在整个回放期间保持有效。

## 现有行为

- 配置读取 `configs/show.yaml`，相对路径仍按项目根目录解析。
- 模型仍由现有 `RobotModelHelper` 从 `data/robot.yaml` 加载。
- 日志格式仍识别 `current pos ... yaw ...` 和 `final linearV ... angularV ...`。
- 一个位姿之后、下一个位姿之前的速度记录合并到当前帧，保留最后一条速度及记录数。
- 跨午夜处理、时间回退夹取、无效记录跳过均沿用原逻辑。
- 完整读取日志后构建轨迹；按距离和累计 yaw 变化采样，保留首尾，原地旋转也可采样。
- 每个采样点根据实际位姿绘制机器人 polygon。
- `speed=0` 按固定帧率回放；`speed>0` 使用日志时间及倍率，并受帧率限制。
- 保留启动延迟、循环、当前机器人、odometry/TF、WebSocket 和可选 MCAP 输出。
- 保留每秒重发完整轨迹以供晚连接客户端查看的机制。此前讨论的闪烁/订阅缓存优化不包含在这次结构重构中。
- Ctrl-C/SIGTERM 停止，显式关闭可视化服务并检查关闭错误。

## 阅读和修改入口

修改配置字段：看 `options`。修改日志字段或解析规则：看 `log_parser`。修改轮廓或轨迹密度：看 `trajectory`。修改播放节奏、发布频率或启动/退出：看 `replay`。

后续出现实际扩展需求时，再针对上述边界增加实现。目前不需要注册表、抽象图层、输出插件或新的日志格式。

## 构建与运行

在项目根目录执行：

```bash
cmake --build build --target show -j 4
./build/tools/show
```

目标名和可执行文件路径沿用现有方式。`tools/CMakeLists.txt` 使用 `add_subdirectory(show show_build)`，将 CMake 的中间目录与 `build/tools/show` 可执行文件分开，避免目录重名。

## 验证范围

重构验证关注行为一致性：对同一份日志逐帧比较旧、新解析结果，对比采样轮廓的数量和每个顶点；覆盖跨午夜、坏记录、角度跨 ±π 及原地旋转。随后构建 `show`，运行文件输出回放并检查 MCAP 中的轨迹、当前轮廓和位姿数量。
