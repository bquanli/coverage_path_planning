#pragma once

#include "types.hh"

#include <iosfwd>
#include <optional>
#include <vector>

namespace pdlog
{
struct ConversionConfig
{
  std::filesystem::path input_dir;
  std::optional<Timestamp> start;
  std::optional<Timestamp> end;
  std::vector<std::string> keywords;
  InputFormat input_format = InputFormat::Pdlog;
};

ConversionConfig
load_config(std::filesystem::path const& config_path);

// 扫描完成后才开始读取；诊断流用于报告并跳过无效文件名。
std::vector<LogFile>
collect_files(ConversionConfig const& config, std::ostream& diagnostics);
} // namespace pdlog
