#pragma once

#include "ppmd_decoder.hh"
#include "types.hh"

#include <cstdio>
#include <functional>
#include <memory>
#include <optional>
#include <vector>

namespace pdlog
{
namespace detail
{
struct FileCloser
{
  void
  operator()(FILE* file) const noexcept
  {
    std::fclose(file);
  }
};
using InputFile = std::unique_ptr<FILE, FileCloser>;
} // namespace detail

class ILogLineReader
{
public:
  virtual ~ILogLineReader() = default;
  // nullopt 表示正常读完并完成检查；错误抛异常，之后应丢弃读取器。
  // 已返回的行拥有数据，不会被后续读取或文件切换覆盖。
  virtual std::optional<LogLine>
  next_line() = 0;
};

class PdlogReader final : public ILogLineReader
{
public:
  explicit PdlogReader(std::filesystem::path path);
  std::optional<LogLine>
  next_line() override;
  PdlogReader(PdlogReader const&) = delete;
  PdlogReader&
  operator=(PdlogReader const&) = delete;

private:
  std::filesystem::path path_;
  detail::InputFile input_;
  // 逆序析构：先释放模型及其输入适配器，再关闭 input_。
  std::unique_ptr<PpmdDecoder> decoder_;
  std::size_t line_number_ = 0;
  bool reached_end_ = false;
};

class TextLogReader final : public ILogLineReader
{
public:
  explicit TextLogReader(std::filesystem::path path);
  std::optional<LogLine>
  next_line() override;
  TextLogReader(TextLogReader const&) = delete;
  TextLogReader&
  operator=(TextLogReader const&) = delete;

private:
  std::filesystem::path path_;
  detail::InputFile input_;
  std::size_t line_number_ = 0;
};

class LogSequenceReader
{
public:
  using FileStarted = std::function<void(LogFile const&)>;

  // 按传入顺序读取。目录扫描入口负责排序；也可显式传入任意路径。
  explicit LogSequenceReader(std::vector<LogFile> files,
                             FileStarted on_file_started = {});
  std::optional<LogLine>
  next_line();
  [[nodiscard]] std::size_t
  filesProcessed() const noexcept
  {
    return files_processed_;
  }

private:
  std::vector<LogFile> files_;
  FileStarted on_file_started_;
  std::unique_ptr<ILogLineReader> current_reader_;
  std::size_t files_processed_ = 0;
};
} // namespace pdlog
