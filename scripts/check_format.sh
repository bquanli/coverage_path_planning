#!/usr/bin/env bash
#
# 检查（或修正）本项目源码的排版是否符合 .clang-format。
#
# 两个边界是有意为之的：
#   1. 只看 git 跟踪的文件——本地的临时草稿不应该卡住 CI。
#   2. 排除 third_party/——那是别人的代码，不按我们的风格排版。
#
# 用法：
#   scripts/check_format.sh          只检查，不改文件；有差异则退出码非 0
#   scripts/check_format.sh --fix    就地格式化
#   CLANG_FORMAT=clang-format-23 scripts/check_format.sh    指定可执行文件

set -euo pipefail

cd "$(git rev-parse --show-toplevel)"

# 带版本号后缀是 Debian/Ubuntu 的打包习惯，必须一并探测。
clang_format="${CLANG_FORMAT:-}"
if [[ -z "${clang_format}" ]]; then
  for candidate in clang-format clang-format-23 clang-format-21 \
                   clang-format-20 clang-format-19 clang-format-18; do
    if command -v "${candidate}" > /dev/null 2>&1; then
      clang_format="${candidate}"
      break
    fi
  done
fi

if [[ -z "${clang_format}" ]]; then
  echo "找不到 clang-format（Ubuntu: apt install clang-format）" >&2
  exit 127
fi

mapfile -t files < <(
  git ls-files -- '*.c' '*.cc' '*.cpp' '*.h' '*.hh' '*.hpp' \
      ':(exclude)third_party/'
)

if [[ ${#files[@]} -eq 0 ]]; then
  echo "没有需要检查的文件"
  exit 0
fi

echo "使用 ${clang_format}，共 ${#files[@]} 个文件"

if [[ "${1:-}" == "--fix" ]]; then
  "${clang_format}" -i "${files[@]}"
  echo "已就地格式化"
  exit 0
fi

# --dry-run 不改文件，--Werror 把差异变成非零退出码，
# 并按 file:line:col 打印具体位置，而不是只告诉你“有问题”。
"${clang_format}" --dry-run --Werror "${files[@]}"
echo "格式检查通过"
