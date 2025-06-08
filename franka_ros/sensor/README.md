# 六维力传感器C++数据采集系统

高性能的六维力传感器数据采集和实时可视化程序，专为M4313MXB六轴力传感器设计。相比Python版本，该C++实现提供了更高的采样频率（可达1000Hz）和更低的延迟。

## 功能特性

- ⚡ **高频采样**：支持高达1000Hz的数据采集频率
- 📊 **实时可视化**：OpenCV驱动的实时图表显示
- 🔧 **自动调零**：启动时自动执行调零校准
- 🧵 **多线程架构**：数据采集和显示分离，确保高性能
- 📈 **性能监控**：实时显示采样频率和统计信息
- 🎛️ **灵活配置**：支持命令行参数配置

## 系统要求

### 硬件
- M4313MXB六轴力传感器
- USB转串口适配器（通常为/dev/ttyUSB0）
- Linux系统（已在Ubuntu 18.04+测试）

### 软件依赖
- C++17编译器（GCC 7.0+或Clang 5.0+）
- CMake 3.16+
- OpenCV 4.x
- POSIX线程库

## 安装依赖

### Ubuntu/Debian
```bash
sudo apt update
sudo apt install build-essential cmake
sudo apt install libopencv-dev
```

### CentOS/RHEL
```bash
sudo yum groupinstall "Development Tools"
sudo yum install cmake opencv-devel
```

## 编译安装

1. **克隆或进入项目目录**
```bash
cd franka_ros/sensor
```

2. **运行构建脚本**
```bash
chmod +x build.sh
./build.sh
```

3. **手动构建（可选）**
```bash
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)
```

## 使用说明

### 基本使用
```bash
# 使用默认参数（/dev/ttyUSB0, 1000Hz）
./build/force_sensor_reader

# 指定串口和采样频率
./build/force_sensor_reader --port /dev/ttyUSB1 --freq 500

# 查看帮助
./build/force_sensor_reader --help
```

### 命令行参数
- `--port <设备>`：指定串口设备路径（默认：/dev/ttyUSB0）
- `--freq <频率>`：设置采样频率Hz（默认：1000）
- `--help`：显示帮助信息

### 交互控制
- **ESC键**：退出程序
- **R键**：重置数据显示
- **Ctrl+C**：安全退出（信号处理）

## 数据格式

程序解析M4313MXB传感器的标准数据包格式：

```
数据包结构:
[0xAA 0x55] [长度] [包编号] [6通道数据] [校验]
```

六个数据通道：
- **Fx, Fy, Fz**：三轴力（单位：N）
- **Mx, My, Mz**：三轴力矩（单位：N·m）

## 性能优化

### 高频采样优化
- 使用原始串口API，避免缓冲延迟
- 多线程架构分离数据采集和显示
- 优化的数据解析算法
- 内存预分配和无锁数据结构

### 实时显示优化
- OpenCV硬件加速绘图
- 动态数据缓冲区管理
- 60FPS显示刷新率限制

## 故障排除

### 串口权限问题
```bash
# 添加用户到dialout组
sudo usermod -a -G dialout $USER
# 重新登录或重启
```

### 设备未找到
```bash
# 检查设备连接
ls /dev/ttyUSB*
dmesg | grep tty

# 检查设备权限
ls -l /dev/ttyUSB0
```

### OpenCV编译错误
```bash
# 确认OpenCV版本
pkg-config --modversion opencv4

# 重新安装OpenCV开发包
sudo apt install --reinstall libopencv-dev
```

## 与Python版本对比

| 功能 | Python版本 | C++版本 |
|------|------------|---------|
| 最大采样频率 | ~300Hz | 1000Hz+ |
| CPU使用率 | 高 | 低 |
| 内存使用 | 高 | 低 |
| 启动时间 | 慢 | 快 |
| 实时性 | 一般 | 优秀 |
| 可视化 | matplotlib | OpenCV |

## 技术架构

```
┌─────────────────┐    ┌──────────────────┐
│   串口数据采集   │    │   实时数据显示    │
│     线程        │    │      线程        │
└─────────────────┘    └──────────────────┘
         │                       │
         ▼                       ▼
┌─────────────────────────────────────────┐
│            数据缓冲区                    │
│         (线程安全队列)                   │
└─────────────────────────────────────────┘
```

### 关键组件
- **SerialPort**：高性能串口通信类
- **DataVisualizer**：OpenCV实时图表类
- **ForceSensorReader**：主控制器类

## 开发计划

- [ ] 数据记录功能（CSV/HDF5格式）
- [ ] 网络数据传输（TCP/UDP）
- [ ] 频谱分析功能
- [ ] 滤波器配置
- [ ] GUI配置界面

## 贡献

欢迎提交Issues和Pull Requests来改进这个项目。

## 许可证

本项目遵循项目仓库的许可证条款。

---

**注意**：使用前请确保传感器正确连接并配置好串口权限。运行时请勿触碰传感器以确保调零准确性。 