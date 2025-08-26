#!/bin/bash
clang-format --version

# 如果输入了目录路径，则使用输入路径；否则使用脚本所在的目录
TARGET_DIR="${1:-$(dirname "$0")}"

# 查找所有的 .c 和 .cpp 文件并进行格式化
find "$TARGET_DIR" -type f \( -name "*.cc" -o -name "*.cpp" -o -name "*.hpp" -o -name "*.h" \) -exec clang-format -style=file -i {} +

echo "格式化完成: $TARGET_DIR 中的所有 .c 和 .cpp 文件"


# cd $(dirname $0)

# clang-format --version
# cmake-format --version
# autopep8 --version

# # clang-format, version v15 is required
# find ./src -regex '.*\.cc\|.*\.cpp\|.*\.h\|.*\.proto' -and -not -regex '.*\.pb\.cc\|.*\.pb\.h' | xargs clang-format -i --style=file

# # cmake-format, apt install cmake-format
# find ./ -regex '.*\.cmake\|.*CMakeLists\.txt$' -and -not -regex '\./.*build.*/.*\|\./document/.*\|\./cmake/kit/.*' | xargs cmake-format -c ./.cmake-format.py -i

# # autopep8, apt install python3-autopep8
# find ./ -regex '.*\.py' -and -not -regex '\./.*build.*/.*\|\./document/.*' | xargs autopep8 -i --global-config ./.pycodestyle