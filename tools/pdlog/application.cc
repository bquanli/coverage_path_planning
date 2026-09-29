#include "application.hh"
#include "log_reader.hh"

#include <algorithm>
#include <ostream>
#include <stdexcept>
#include <utility>

namespace pdlog
{
ConversionApplication::ConversionApplication(ConversionConfig config,
                                             FILE* output,
                                             std::ostream& diagnostics)
  : config_(std::move(config))
  , output_(output)
  , diagnostics_(diagnostics)
{
  if(output_ == nullptr)
  {
    throw std::invalid_argument("output must not be null");
  }
}

ConversionReport
ConversionApplication::run() const
{
  std::vector<LogFile> files;
  try
  {
    files = collect_files(config_, diagnostics_);
  }
  catch(std::filesystem::filesystem_error const& error)
  {
    throw std::runtime_error("cannot scan " + config_.input_dir.string() +
                             ": " + error.what());
  }
  if(files.empty())
  {
    throw std::runtime_error(
        "no log files match the configured directory and time range: " +
        config_.input_dir.string());
  }

  LogSequenceReader reader{std::move(files),
                           [this](LogFile const& file)
                           {
                             diagnostics_ << "pdlog_decompress: processing "
                                          << file.path << '\n';
                           }};
  ConversionReport report;
  bool needs_separator = false;
  while(auto line = reader.next_line())
  {
    ++report.lines_read;
    bool const matches =
        config_.keywords.empty() ||
        std::ranges::any_of(
            config_.keywords,
            [&](std::string const& keyword)
            { return line->text.find(keyword) != std::string::npos; });
    if(!matches)
    {
      continue;
    }

    // 只在下一条实际输出前补换行，空文件和无匹配文件不改变状态。
    if((needs_separator && std::fputc('\n', output_) == EOF) ||
       std::fwrite(line->text.data(), 1, line->text.size(), output_) !=
           line->text.size())
    {
      throw std::runtime_error("stdout write failed");
    }
    needs_separator = line->text.back() != '\n';
    ++report.lines_written;
  }
  if(std::fflush(output_) != 0)
  {
    throw std::runtime_error("stdout flush failed");
  }
  report.files_processed = reader.filesProcessed();
  return report;
}
} // namespace pdlog
