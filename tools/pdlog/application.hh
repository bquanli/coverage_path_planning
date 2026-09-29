#pragma once

#include "config.hh"

#include <cstdio>
#include <iosfwd>

namespace pdlog
{
struct ConversionReport
{
  std::size_t files_processed = 0;
  std::size_t lines_read = 0;
  std::size_t lines_written = 0;
};

class ConversionApplication
{
public:
  // 借用输出和诊断流；run() 刷新输出，但不关闭它们。
  ConversionApplication(ConversionConfig config,
                        FILE* output,
                        std::ostream& diagnostics);
  // 当前完成日志读取、关键词过滤及文本输出；失败抛异常。
  [[nodiscard]] ConversionReport
  run() const;

private:
  ConversionConfig config_;
  FILE* output_;
  std::ostream& diagnostics_;
};
} // namespace pdlog
