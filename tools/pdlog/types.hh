#pragma once

#include <chrono>
#include <cstddef>
#include <filesystem>
#include <string>

namespace pdlog
{
// 文件名中的本地时间，仅用于排序和筛选，不表示已转换为 UTC。
using Timestamp = std::chrono::sys_seconds;

enum class InputFormat : uint8_t
{
  Pdlog,
  Text,
};

struct LogFile
{
  std::filesystem::path path;
  Timestamp time;
  InputFormat format = InputFormat::Pdlog;
};

struct LogLine
{
  std::filesystem::path source_file;
  std::size_t line_number = 0; // 从 1 开始，每个文件重新计数。
  std::string text; // 拥有正文，保留 LF、CRLF、内嵌 NUL 和无换行末行。
};
} // namespace pdlog
