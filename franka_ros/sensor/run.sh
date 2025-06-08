#!/bin/bash

# 六维力传感器C++程序快速启动脚本

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
EXECUTABLE="$SCRIPT_DIR/build/force_sensor_reader"

echo "=== 六维力传感器C++数据采集系统 ==="
echo "程序位置: $EXECUTABLE"

# 检查程序是否存在
if [ ! -f "$EXECUTABLE" ]; then
    echo "错误：可执行文件不存在，请先运行构建："
    echo "  ./build.sh"
    exit 1
fi

# 检查串口设备
DEFAULT_PORT="/dev/ttyUSB0"
if [ ! -e "$DEFAULT_PORT" ]; then
    echo "警告：默认串口 $DEFAULT_PORT 不存在"
    echo "可用的串口设备："
    ls /dev/ttyUSB* 2>/dev/null || echo "  未找到 /dev/ttyUSB* 设备"
    ls /dev/ttyACM* 2>/dev/null || echo "  未找到 /dev/ttyACM* 设备"
    echo ""
    echo "使用 --port 参数指定正确的设备路径"
    echo "示例: $0 --port /dev/ttyUSB1"
    echo ""
fi

# 检查串口权限
if [ -e "$DEFAULT_PORT" ]; then
    if [ ! -w "$DEFAULT_PORT" ]; then
        echo "警告：没有串口写权限，可能需要："
        echo "  sudo usermod -a -G dialout \$USER"
        echo "  然后重新登录"
        echo ""
    fi
fi

echo "启动程序..."
echo "按ESC键退出，按R键重置数据"
echo "----------------------------------------"

# 运行程序，传递所有命令行参数
exec "$EXECUTABLE" "$@" 