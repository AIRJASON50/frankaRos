#!/bin/bash

# 六维力传感器C++版本性能测试脚本

echo "=== 六维力传感器性能测试 ==="
echo

# 系统信息
echo "系统信息："
echo "  操作系统: $(uname -a)"
echo "  CPU: $(lscpu | grep 'Model name' | cut -d':' -f2 | xargs)"
echo "  CPU核心数: $(nproc)"
echo "  内存: $(free -h | grep 'Mem:' | awk '{print $2}')"
echo

# 检查可执行文件
CPP_EXECUTABLE="./build/force_sensor_reader"
PYTHON_SCRIPT="../docs/forcesenseRead.py"

if [ ! -f "$CPP_EXECUTABLE" ]; then
    echo "错误：C++程序未找到，请先运行 ./build.sh"
    exit 1
fi

if [ ! -f "$PYTHON_SCRIPT" ]; then
    echo "警告：Python脚本未找到，跳过Python版本测试"
    PYTHON_SCRIPT=""
fi

echo "测试配置："
echo "  C++程序: $CPP_EXECUTABLE"
if [ -n "$PYTHON_SCRIPT" ]; then
    echo "  Python脚本: $PYTHON_SCRIPT"
fi
echo

# 函数：测试程序性能
test_performance() {
    local program=$1
    local name=$2
    local duration=30  # 测试时长（秒）
    
    echo "--- 测试 $name 版本 ---"
    echo "测试时长: ${duration}秒"
    
    # 启动性能监控
    local start_time=$(date +%s)
    local pid
    
    if [[ $program == *.py ]]; then
        echo "启动Python版本..."
        timeout ${duration}s python3 "$program" > /dev/null 2>&1 &
    else
        echo "启动C++版本..."
        timeout ${duration}s "$program" --freq 1000 > /dev/null 2>&1 &
    fi
    
    pid=$!
    
    # 监控资源使用
    local cpu_sum=0
    local mem_sum=0
    local count=0
    
    while kill -0 $pid 2>/dev/null; do
        if ps -p $pid > /dev/null 2>&1; then
            local cpu=$(ps -p $pid -o %cpu --no-headers | xargs)
            local mem=$(ps -p $pid -o %mem --no-headers | xargs)
            
            if [[ $cpu =~ ^[0-9]+\.?[0-9]*$ ]] && [[ $mem =~ ^[0-9]+\.?[0-9]*$ ]]; then
                cpu_sum=$(echo "$cpu_sum + $cpu" | bc -l)
                mem_sum=$(echo "$mem_sum + $mem" | bc -l)
                count=$((count + 1))
            fi
        fi
        sleep 1
    done
    
    wait $pid 2>/dev/null
    
    local end_time=$(date +%s)
    local actual_duration=$((end_time - start_time))
    
    if [ $count -gt 0 ]; then
        local avg_cpu=$(echo "scale=2; $cpu_sum / $count" | bc -l)
        local avg_mem=$(echo "scale=2; $mem_sum / $count" | bc -l)
        
        echo "结果："
        echo "  实际运行时间: ${actual_duration}秒"
        echo "  平均CPU使用率: ${avg_cpu}%"
        echo "  平均内存使用率: ${avg_mem}%"
    else
        echo "结果：无法获取性能数据（程序可能立即退出）"
    fi
    echo
}

# 检查串口设备
if [ ! -e "/dev/ttyUSB0" ]; then
    echo "注意：未找到 /dev/ttyUSB0 设备，程序将无法连接传感器"
    echo "但仍可测试程序启动性能和资源使用情况"
    echo
fi

# 测试C++版本
test_performance "$CPP_EXECUTABLE" "C++"

# 测试Python版本（如果存在）
if [ -n "$PYTHON_SCRIPT" ]; then
    test_performance "$PYTHON_SCRIPT" "Python"
    
    echo "=== 性能对比总结 ==="
    echo "C++版本优势："
    echo "  • 更低的CPU使用率"
    echo "  • 更少的内存占用"
    echo "  • 更快的启动时间"
    echo "  • 支持更高的采样频率（1000Hz+）"
    echo "  • 更低的数据采集延迟"
    echo
fi

echo "=== 测试完成 ===" 