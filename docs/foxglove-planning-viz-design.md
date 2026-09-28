# Foxglove C++ 规划可视化薄封装

定位：本项目的架构与实现约定。原方案参考 Foxglove C++ SDK 0.27.0 文档，本次审查同时对照本机 `/opt/sdk/foxglove` 的头文件、C++ 包装源码及官方 schema；不据此宣称已确认安装版本或完成编译、联调。下文代码片段是骨架，完整实现以仓库源码为准。

## 设计结论

对外提供 planning_viz 命名空间中的业务函数；内部只保留一个 Backend 类，直接持有 SDK 原生通道、独立 Context、WebSocketServer 和可选 McapWriter。几何转换使用无状态函数。不建立通用 ChannelManager、不包装每一种 SDK channel、不引入动态 topic 注册表。

固定业务入口包括 globalPath、localPath、footprint 和 clearLocalPath。业务模块 include 一个头文件即可调用，无需持有或传递可视化对象。

## 文件组织

| 文件 | 职责 |
|---|---|
| include/foxglove_viz/viz.hh | 对外配置、绘图上下文和业务函数声明 |
| src/viz.cc | 唯一 Backend 的访问函数；全部对外函数转发 |
| src/backend.hh | 私有 Backend 类与资源成员 |
| src/backend.cc | 初始化、关闭、选择固定通道、发布、错误处理 |
| src/converters.hh | 私有消息构造函数声明 |
| src/converters.cc | 路径/轮廓到 SceneUpdate 的无状态转换 |
| src/main.cc | 使用封装的初始化、周期发布、正常关闭示例 |
| CMakeLists.txt、src/CMakeLists.txt | 构建 planning_viz 共享库及示例；链接 SDK、Eigen 和线程库；传播 C++20 要求 |

所有 src 头文件均为内部文件，公共 include 目录不暴露 Backend 或 SDK 类型。多动态库调用时，共同链接同一份 planning_viz 共享库，避免每个动态库各自拥有一个全局后端。Eigen::Eigen 是公共依赖；SDK 和线程库是私有依赖。SDK 查找允许使用 foxglove-sdk_DIR/CMAKE_PREFIX_PATH，`/opt/sdk/foxglove` 仅作本机后备路径。

## 对外接口：viz.hh

```cpp
#pragma once
#include <Eigen/Core>
#include <cstdint>
#include <optional>
#include <span>
#include <string>
#include <string_view>

namespace planning_viz {

// 明确表示 (x, y, z)。若业务轨迹是 (x, y, yaw)，另写适配函数。
using Points3 = std::span<const Eigen::Vector3d>;

struct Config {
    bool enabled = true;
    bool websocket_enabled = true;
    std::string host = "127.0.0.1";
    std::uint16_t port = 8765;
    std::optional<std::string> mcap_path;
    std::string default_frame = "map";
};

struct DrawContext {
    // nullopt 表示在本次调用时取 system_clock。
    // 显式 0 合法，方便使用仿真时间。
    std::optional<std::uint64_t> stamp_ns;
    std::string_view frame_id; // 空则使用 default_frame
};

// init/shutdown 必须与彼此及所有发布调用互斥：启动线程前 init，join 后 shutdown。
// error 输出失败原因；成功时清空。重复 init 返回 false。
bool init(const Config& config, std::string& error);
bool shutdown(std::string& error);

void globalPath(Points3 points, DrawContext ctx = {});
void localPath(Points3 points, DrawContext ctx = {});
void footprint(Points3 vertices, DrawContext ctx = {});
void clearLocalPath(DrawContext ctx = {});

} // namespace planning_viz
```

普通调试可以省略 DrawContext；同一规划周期的不同 topic 可以显式传入相同时间戳。同一 topic 的更新和清除必须严格递增，详见下方时间规则。frame_id 只声明坐标含义，不执行坐标变换；所有图形使用 frame_locked=false。若使用车体坐标，调用方需提供对应时刻的变换并周期重发轮廓，或先将点转换到 map。span 和 string_view 只在同步调用期间使用，不保存到后台。

未初始化、禁用或 shutdown 后的发布均无操作。enabled=false 仍占用一次初始化状态；再次 init 前必须 shutdown。正常退出前必须显式 shutdown，以获取关闭和落盘错误；不依赖全局对象析构的时机。

## 内部类：backend.hh

以下展示主要成员和接口，省略头文件和共用发布函数声明。

```cpp
namespace planning_viz::detail {

class Backend {
public:
    bool init(const Config&, std::string& error);
    bool shutdown(std::string& error);

    void globalPath(Points3, DrawContext);
    void localPath(Points3, DrawContext);
    void footprint(Points3, DrawContext);
    void clearLocalPath(DrawContext);

private:
    using SceneChannel = foxglove::messages::SceneUpdateChannel;

    // 固定实体的顺序控制，不包装 channel、不缓存几何数据。
    struct PublicationState {
        std::mutex mutex;
        std::optional<std::uint64_t> last_stamp;
    };

    // 仅组织所有权，不提供通道管理接口。
    struct Resources {
        Config config;
        // 独立 Context；默认构造 Context 会引用 SDK 的默认全局 Context。
        foxglove::Context context = foxglove::Context::create();
        std::optional<SceneChannel> global_path;
        std::optional<SceneChannel> local_path;
        std::optional<SceneChannel> footprint;
        std::optional<foxglove::McapWriter> writer;
        std::optional<foxglove::WebSocketServer> server;
        PublicationState global_path_state;
        PublicationState local_path_state;
        PublicationState footprint_state;
    };

    bool initialized_ = false;
    std::unique_ptr<Resources> resources_;
};

} // namespace planning_viz::detail
```

optional 用于表达初始化前/未开启时没有资源，也适合承载 SDK 的可移动、不可复制对象。Resources 中 Context 位于依赖它的对象之前，按 C++ 逆序析构规则在这些对象之后释放。运行阶段 Resources 不替换、不移动，其配置也不修改；每个 PublicationState 在自己的互斥锁下更新。

## 全局入口：viz.cc

```cpp
#include "foxglove_viz/viz.hh"
#include "backend.hh"

namespace planning_viz {
namespace {
detail::Backend& backend() {
    static detail::Backend instance;
    return instance;
}
}

bool init(const Config& config, std::string& error) {
    return backend().init(config, error);
}
bool shutdown(std::string& error) {
    return backend().shutdown(error);
}
void globalPath(Points3 p, DrawContext ctx) {
    backend().globalPath(p, ctx);
}
void localPath(Points3 p, DrawContext ctx) {
    backend().localPath(p, ctx);
}
void footprint(Points3 p, DrawContext ctx) {
    backend().footprint(p, ctx);
}
void clearLocalPath(DrawContext ctx) {
    backend().clearLocalPath(ctx);
}
}
```

函数局部 static 只保证 instance 的首次构造安全，不会自动保护后续操作。Backend 默认构造保持轻量，不打开端口或文件。

## 初始化与失败处理：backend.cc

init 的步骤：

1. 如果 initialized_ 为 true，返回“已初始化”，不覆盖原资源。
2. enabled 为 false 时记录 initialized_ = true，保持 resources_ 为空，后续发布无操作。
3. 校验非空 default_frame；enabled 为 true 时至少开启一个输出端；配置了 mcap_path 时路径不得为空。
4. 创建局部 unique_ptr<Resources> candidate，复制 Config。
5. 用 candidate->context 创建固定通道，逐个检查返回结果。
6. 按配置创建 MCAP 和 WebSocket，并绑定同一个 Context。
7. 全部成功后，将 candidate 移入 resources_，设置 initialized_。
8. 任一步失败都清理已创建资源，返回包含操作名的错误。已创建的 MCAP 文件可能保留为可检查的部分文件，不自动删除。

代表性 SDK 调用：

```cpp
auto result = SceneChannel::create(
    "/planning/global_path", candidate->context);
if (!result.has_value()) {
    error = std::string("create global_path: ")
          + foxglove::strerror(result.error());
    return false; // 此处尚未创建输出端
}
candidate->global_path.emplace(std::move(result.value()));

// local_path 和 footprint 同样创建；均必须检查 result。

// 仅 mcap_path 有值时执行，检查结果后持有 writer。
foxglove::McapWriterOptions mcap;
mcap.context = candidate->context;
mcap.path = *candidate->config.mcap_path;
mcap.truncate = false;
auto writer_result = foxglove::McapWriter::create(mcap);

// 仅 websocket_enabled 时执行。
foxglove::WebSocketServerOptions ws;
ws.context = candidate->context;
ws.host = candidate->config.host;
ws.port = candidate->config.port;
auto server_result = foxglove::WebSocketServer::create(std::move(ws));
// 检查 server_result，再 emplace 到 candidate->server。
```

上面是调用片段，实际顺序为通道 → MCAP → WebSocket，端口启动成功后立即提交 candidate。统一清理逻辑显式 stop 已启动的 server、close 已创建的 writer，并保留错误信息。本机 SDK 的析构器也会释放底层句柄，但显式关闭才能读取关闭错误；还需捕获资源创建中的异常并进入相同清理路径。MCAP 使用 truncate=false，不覆盖已有文件。

shutdown 在业务线程退出后调用：先移动 resources_ 到局部变量，使后续顺序调用成为无操作；分别 stop server、close writer，记录全部关闭错误；释放资源并重置 initialized_。某一项关闭失败也应继续清理其他项。无资源时重复 shutdown 成功。

## 发布一条路径

以下为关键流程（省略无效几何输入转删除的分支）；实际 globalPath/localPath/footprint 共用 publishLine。resolveStamp、makeLineUpdate 与 reportError 为本封装内部函数。

```cpp
void Backend::globalPath(Points3 points, DrawContext ctx) {
    if (!resources_) return;
    auto& r = *resources_;

    try {
        std::lock_guard lock(r.global_path_state.mutex);
        const auto stamp_ns = resolveStamp(ctx.stamp_ns);
        if (r.global_path_state.last_stamp &&
            stamp_ns <= *r.global_path_state.last_stamp) {
            reportError("globalPath", "timestamp must strictly increase");
            return;
        }
        const std::string_view frame = ctx.frame_id.empty()
            ? std::string_view(r.config.default_frame) : ctx.frame_id;

        auto message = makeLineUpdate(
            points, "path", frame, stamp_ns,
            foxglove::messages::Color{0.2, 0.8, 0.3, 1.0},
            0.04, false);

        const auto error = r.global_path->log(message, stamp_ns);
        if (error != foxglove::FoxgloveError::Ok) {
            reportError("globalPath", foxglove::strerror(error));
        } else {
            r.global_path_state.last_stamp = stamp_ns;
        }
    } catch (const std::exception& error) {
        reportError("globalPath", error.what());
    }
}
```

resolveStamp 只在 optional 无值时读取 system_clock；检查负时间和消息 Timestamp 秒字段范围，防止无符号转换或截断。reportError 使用线程安全的 stderr 兜底日志，所有发布错误合计每秒至多输出一次，不向调用方抛出异常。业务有日志框架时可替换。内存分配、锁等待、序列化和输出都可能有开销，不承诺硬实时；init/shutdown 的错误字符串分配也不承诺在内存耗尽时仍可返回错误。

localPath 使用 local_path 通道及另一组样式；footprint 使用 LINE_LOOP；clearLocalPath 发送 MATCHING_ID 的删除消息。业务调用忽略内部绘图错误，初始化/关闭仍显式返回结果。

## 转换函数：converters.cc

函数声明：

```cpp
foxglove::messages::SceneUpdate makeLineUpdate(
    Points3 points, std::string_view entity_id,
    std::string_view frame_id, std::uint64_t stamp_ns,
    foxglove::messages::Color color, double width_m, bool closed);
```

转换核心，省略外层函数声明及输入校验：

```cpp
namespace msg = foxglove::messages;
msg::LinePrimitive line;
line.type = closed ? msg::LinePrimitive::LineType::LINE_LOOP
                   : msg::LinePrimitive::LineType::LINE_STRIP;
line.thickness = width_m;
line.scale_invariant = false;
line.color = color;

// 显式单位位姿，避免四元数默认为全零。
msg::Pose identity;
identity.position = msg::Vector3{0.0, 0.0, 0.0};
identity.orientation = msg::Quaternion{0.0, 0.0, 0.0, 1.0};
line.pose = identity;

line.points.reserve(points.size());
for (const auto& p : points) {
    line.points.push_back(msg::Point3{p.x(), p.y(), p.z()});
}

msg::SceneEntity entity;
entity.id = std::string(entity_id);
entity.frame_id = std::string(frame_id);
entity.timestamp = toMessageTimestamp(stamp_ns);
entity.lifetime = msg::Duration{0, 0};
entity.frame_locked = false;
entity.lines.push_back(std::move(line));

msg::SceneUpdate update;
update.entities.push_back(std::move(entity));
return update;
```

转换及发布的明确约定：

- 空点集转换为删除同 ID 实体的 SceneUpdate，不只是 return。
- 非空开放折线少于 2 点、闭合轮廓少于 3 点、存在非有限坐标或线宽非有限/不为正时，转换函数抛出 invalid_argument。发布层捕获后报告无效输入并发送删除消息，避免保留旧图形；第一版不把单点视为可见线段。空输入直接删除，不报告几何错误。
- 轮廓顶点按边界顺序提供，LINE_LOOP 自动连接最后一点与第一点，无需重复首点。本封装不判断多边形自交、面积或路径碰撞。
- toMessageTimestamp 使用 sec = stamp_ns / 1e9、nsec = stamp_ns % 1e9，检查 sec 能放入 uint32_t。
- 删除消息使用 SceneEntityDeletionType::MATCHING_ID，附同一数据时间戳及 entity ID。按官方 schema，删除只匹配严格早于删除时间的实体；相同时间戳的删除不能作为可靠的清理方式。
- 非空实体显式 lifetime 为零（不限期）。重用同 topic、同 ID 更新实体。

### 时间与清除规则

每个固定 topic 独立维护一个互斥锁和最近一次 log 返回 Ok 的时间戳。锁覆盖取时、比较、转换、log 和状态提交；不同 topic 可以并发。所有发布（包括空输入、无效几何转删除、clearLocalPath）必须晚于该 topic 的上一次成功提交，否则限频报告错误并跳过，保留旧状态；不偷偷修改调用方的时间戳。log 失败时不推进状态，可以用相同时间戳重试；Ok 只表示 SDK 接受调用，不等于客户端已收到数据或文件已持久化。

例如 localPath(points, {.stamp_ns=100}) 后，clearLocalPath({.stamp_ns=100}) 会被拒绝；应使用 101 或下一周期时间戳清除。不要混用系统时间和仿真时间，需要对齐显示或录制的各 topic 应采用同一时间基准。系统时钟回拨时较旧的数据也会跳过；仿真重置需停止发布、shutdown/init，并让可视化客户端开启新会话，录制时使用新的 MCAP 路径。Timestamp 最大值处无法再发送更晚的删除，应开启新会话。上述规则以可预测行为为先，不提供时间倒退回放或自动重排。

## topic 和实体 ID

| 业务入口 | 原生 channel 的 topic | ID |
|---|---|---|
| globalPath | /planning/global_path | path |
| localPath | /planning/local_path | path |
| footprint | /planning/footprint | footprint |

独立 topic 便于独立订阅和控制显示。若后续希望同组管理，可把全局/局部路径改到同一 SceneUpdateChannel 并用 global/local ID 区分；对外函数无需改变。业务函数与 channel 不必一一对应。

## 调用示例

```cpp
planning_viz::Config config;
config.host = "0.0.0.0"; // 需要其他设备连接时设置
std::string error;
if (!planning_viz::init(config, error)) {
    // 记录 error；由应用决定是否继续运行。
}

// 任意业务 .cpp，点集的内存在函数返回前有效：
planning_viz::DrawContext ctx{
    .stamp_ns = sensor_stamp_ns,
    .frame_id = "map"
};
planning_viz::globalPath(global_path, ctx);
planning_viz::localPath(local_path, ctx);
planning_viz::footprint(footprint_in_map, ctx);

// 停止并 join 所有发布线程后：
if (!planning_viz::shutdown(error)) {
    // 记录关闭或录制落盘错误。
}
```

`src/main.cc` 周期重发全局路径、局部路径及 map 坐标下的轮廓，并周期演示清除局部路径。示例约 30 秒后结束，或通过 Ctrl-C/SIGTERM 提前结束，然后调用 shutdown。可选的唯一命令行参数为新的 MCAP 输出路径。Foxglove 连接 `ws://127.0.0.1:8765` 后，在 3D 面板使用 map 坐标系并启用 `/planning/*` 三个 topic。

## 并发、性能与验证边界

- init/shutdown 与彼此及发布互斥是第一版的调用契约，不是代码提供了热重配置保护。不能并发读写 resources_。
- 配置和 SDK 资源的所有权在发布期间不变；各 topic 在自己的锁内构造局部消息并更新顺序状态。调用方不得同时修改传入点集或 frame_id 对应字符串。
- 同一个实体建议只有一个发布者。多个发布者允许并发调用，但锁不按时间排序；晚到的旧时间戳将被拒绝。
- 不使用 hasSinks/clientCount 跳过发布作为第一版优化，避免混淆实时订阅和 MCAP 录制。
- 同步发布不保存 span、不增加异步队列；消息构造和序列化仍有开销，不承诺硬实时。
- 不自动缓存所有绘图结果。晚连接客户端应由业务周期重发当前数据；若要求连接后立即看到一次性全局路径，后续单独增加有限的最新状态重发机制。
- 禁用时可跳过转换，但函数参数表达式仍会求值。昂贵的调试数据准备应由调用方的调试开关控制。
- 本轮按要求仅进行文档和源码静态审查，不编译、不运行示例或联调。后续运行验证重点包括：同 ID 更新、空/非法数据清理、相同或倒退时间戳拒绝、初始化失败回滚、MCAP 正常关闭及晚连接周期重发。静态审查不能替代这些验证。

## 官方参考

- [C++ SDK 总览](https://foxglove-sdk-api-docs.pages.dev/cpp/)
- [Context](https://foxglove-sdk-api-docs.pages.dev/cpp/generated/api/classfoxglove_1_1Context)
- [SceneUpdateChannel](https://foxglove-sdk-api-docs.pages.dev/cpp/generated/api/classfoxglove_1_1messages_1_1SceneUpdateChannel)
- [LinePrimitive](https://foxglove-sdk-api-docs.pages.dev/cpp/generated/api/structfoxglove_1_1messages_1_1LinePrimitive)
- [SceneEntity](https://docs.foxglove.dev/docs/sdk/schemas/scene-entity)
- [SceneEntityDeletion schema](https://docs.foxglove.dev/docs/sdk/schemas/scene-entity-deletion)

参考日期：2026-09-27（Asia/Shanghai）。
