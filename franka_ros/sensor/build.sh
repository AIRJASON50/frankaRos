#!/bin/bash

# 六维力传感器C++程序构建脚本

set -e  # 出错时退出

echo "=== 六维力传感器C++程序构建脚本 ==="

# 检查依赖
echo "检查依赖..."

# 检查OpenCV
if ! pkg-config --exists opencv4; then
    echo "错误：未找到OpenCV 4.x，请安装："
    echo "sudo apt update"
    echo "sudo apt install libopencv-dev"
    exit 1
fi

echo "OpenCV版本: $(pkg-config --modversion opencv4)"

# 创建构建目录
BUILD_DIR="build"
if [ -d "$BUILD_DIR" ]; then
    echo "清理已存在的构建目录..."
    rm -rf "$BUILD_DIR"
fi

mkdir -p "$BUILD_DIR"
cd "$BUILD_DIR"

echo "配置CMake..."
cmake .. -DCMAKE_BUILD_TYPE=Release

echo "编译程序..."
make -j$(nproc)

echo "构建完成！"
echo ""
echo "可执行文件位置: $PWD/force_sensor_reader"
echo ""
echo "运行示例："
echo "  ./force_sensor_reader                    # 使用默认参数"
echo "  ./force_sensor_reader --port /dev/ttyUSB1 --freq 500  # 自定义参数"
echo "  ./force_sensor_reader --help             # 查看帮助"
echo ""
echo "安装到系统（可选）："
echo "  sudo make install" 