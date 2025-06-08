#include <iostream>
#include <thread>
#include <chrono>
#include <atomic>
#include <cstring>
#include <array>
#include <vector>
#include <csignal>
#include <cstdint>
#include <cstdlib>

#include "serial_port.h"
#include "data_visualizer.h"

/**
 * @brief 六维力传感器数据读取器
 */
class ForceSensorReader {
public:
    ForceSensorReader(const std::string& port = "/dev/ttyUSB0", int target_freq = 1000);
    ~ForceSensorReader();
    
    bool initialize();
    void run();
    void stop();

private:
    // 设备参数
    std::string port_;
    int target_frequency_;
    
    // 设备对象
    SerialPort serial_;
    DataVisualizer visualizer_;
    
    // 控制变量
    std::atomic<bool> running_;
    std::atomic<bool> is_zeroing_;
    
    // 数据相关
    std::array<float, 6> zero_offsets_;
    std::vector<std::array<float, 6>> zero_data_;
    
    // 线程
    std::thread data_thread_;
    std::thread display_thread_;
    
    // 统计信息
    std::atomic<uint64_t> packet_count_;
    std::chrono::steady_clock::time_point start_time_;
    
    /**
     * @brief 数据采集线程
     */
    void dataAcquisitionThread();
    
    /**
     * @brief 显示更新线程
     */
    void displayUpdateThread();
    
    /**
     * @brief 解析传感器数据
     * @param data 原始数据包
     * @param package_no 输出包编号
     * @param forces 输出六维力数据
     * @return 解析成功返回true
     */
    bool parseSensorData(const uint8_t* data, uint16_t& package_no, std::array<float, 6>& forces);
    
    /**
     * @brief 反转字节序
     * @param data 原始数据
     * @param size 数据大小
     */
    void reverseBytes(uint8_t* data, size_t size);
    
    /**
     * @brief 校验力值是否在合理范围内
     * @param force 力值
     * @return 合理返回true
     */
    bool isValidForce(float force);
    
    /**
     * @brief 执行调零操作
     */
    void performZeroing();
    
    /**
     * @brief 显示统计信息
     */
    void printStatistics();
};

// 全局变量用于信号处理
std::atomic<bool> g_shutdown(false);
ForceSensorReader* g_reader = nullptr;

void signalHandler(int signal) {
    std::cout << "\n收到信号 " << signal << "，正在关闭程序..." << std::endl;
    g_shutdown = true;
    if (g_reader) {
        g_reader->stop();
    }
    // 强制退出，避免卡在某个地方
    std::exit(0);
}

ForceSensorReader::ForceSensorReader(const std::string& port, int target_freq)
    : port_(port), target_frequency_(target_freq), serial_(port),
      visualizer_("六维力传感器实时数据 - C++版本"), running_(false), is_zeroing_(true),
      packet_count_(0) {
    
    // 初始化零偏值
    zero_offsets_.fill(0.0f);
}

ForceSensorReader::~ForceSensorReader() {
    stop();
}

bool ForceSensorReader::initialize() {
    std::cout << "=== 六维力传感器数据采集系统 (C++版本) ===" << std::endl;
    std::cout << "目标频率: " << target_frequency_ << " Hz" << std::endl;
    std::cout << "串口设备: " << port_ << std::endl;
    
    // 打开串口
    if (!serial_.open()) {
        std::cerr << "串口打开失败" << std::endl;
        return false;
    }
    
    // 发送配置命令
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    
    std::cout << "配置传感器..." << std::endl;
    serial_.sendCommand("AT+SMPF=" + std::to_string(target_frequency_));
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    
    serial_.sendCommand("AT+GSD");
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    
    // 清空缓冲区
    serial_.flushInput();
    
    std::cout << "传感器配置完成" << std::endl;
    std::cout << "开始调零阶段，请勿施加外力..." << std::endl;
    
    return true;
}

void ForceSensorReader::run() {
    if (running_) {
        std::cout << "系统已在运行中" << std::endl;
        return;
    }
    
    running_ = true;
    start_time_ = std::chrono::steady_clock::now();
    
    // 启动线程
    data_thread_ = std::thread(&ForceSensorReader::dataAcquisitionThread, this);
    display_thread_ = std::thread(&ForceSensorReader::displayUpdateThread, this);
    
    std::cout << "数据采集已开始，按ESC键退出程序" << std::endl;
    
    // 等待线程结束
    if (data_thread_.joinable()) {
        data_thread_.join();
    }
    if (display_thread_.joinable()) {
        display_thread_.join();
    }
}

void ForceSensorReader::stop() {
    if (!running_) {
        return;
    }
    
    std::cout << "正在停止数据采集..." << std::endl;
    running_ = false;
    
    // 停止传感器数据传输
    if (serial_.isOpen()) {
        serial_.sendCommand("AT+STOP");
        serial_.close();
    }
    
    printStatistics();
}

void ForceSensorReader::dataAcquisitionThread() {
    uint8_t buffer[31];  // 完整数据包大小
    std::array<float, 6> forces;
    uint16_t package_no;
    
    auto last_stats_time = std::chrono::steady_clock::now();
    const auto stats_interval = std::chrono::seconds(5);
    
    try {
        while (running_ && !g_shutdown) {
            // 同步数据帧头
            if (!serial_.syncData()) {
                std::this_thread::sleep_for(std::chrono::microseconds(100));
                continue;
            }
            
            // 读取剩余29字节
            if (serial_.readData(buffer + 2, 29) < 29) {
                continue;
            }
            
            // 补充帧头
            buffer[0] = 0xAA;
            buffer[1] = 0x55;
            
            // 解析数据
            if (parseSensorData(buffer, package_no, forces)) {
                packet_count_++;
                
                if (is_zeroing_) {
                    // 调零阶段：收集数据
                    zero_data_.push_back(forces);
                    
                    if (zero_data_.size() >= 200) {  // 收集200个数据点用于调零
                        performZeroing();
                        is_zeroing_ = false;
                        std::cout << "调零完成，开始正常数据采集" << std::endl;
                    }
                } else {
                    // 应用零偏校正
                    for (int i = 0; i < 6; ++i) {
                        forces[i] -= zero_offsets_[i];
                        
                        // 数据范围校验
                        if (!isValidForce(forces[i])) {
                            forces[i] = 0.0f;
                        }
                    }
                    
                    // 添加到可视化器
                    visualizer_.addDataPoint(forces);
                }
                
                // 定期打印统计信息
                auto now = std::chrono::steady_clock::now();
                if (now - last_stats_time >= stats_interval) {
                    printStatistics();
                    last_stats_time = now;
                }
            }
        }
    } catch (const std::exception& e) {
        std::cerr << "数据采集线程异常: " << e.what() << std::endl;
    }
}

void ForceSensorReader::displayUpdateThread() {
    try {
        while (running_ && !g_shutdown) {
            // 更新显示，如果用户按ESC则退出
            if (!visualizer_.updateDisplay()) {
                g_shutdown = true;
                break;
            }
            
            // 控制显示刷新频率（约60 FPS）
            std::this_thread::sleep_for(std::chrono::milliseconds(16));
        }
    } catch (const std::exception& e) {
        std::cerr << "显示线程异常: " << e.what() << std::endl;
    }
}

bool ForceSensorReader::parseSensorData(const uint8_t* data, uint16_t& package_no, std::array<float, 6>& forces) {
    // 检查帧头
    if (data[0] != 0xAA || data[1] != 0x55) {
        return false;
    }
    
    // 读取数据包长度（大端序）
    uint16_t package_length = (static_cast<uint16_t>(data[2]) << 8) | data[3];
    
    // 读取包编号（大端序）
    package_no = (static_cast<uint16_t>(data[4]) << 8) | data[5];
    
    // 调试：打印前几个数据包
    static int debug_count = 0;
    if (debug_count < 3) {
        std::cout << "数据包 " << package_no << " (长度: " << package_length << ")" << std::endl;
        debug_count++;
    }
    
    // 解析六个通道的数据（从第6字节开始）
    // 根据调试程序确认，直接按IEEE 754浮点数解析是正确的
    for (int i = 0; i < 6; ++i) {
        uint8_t raw_bytes[4];
        std::memcpy(raw_bytes, &data[6 + i * 4], 4);
        
        // 直接按IEEE 754浮点数解析（小端序）
        float value;
        std::memcpy(&value, raw_bytes, 4);
        forces[i] = value;
        
        // 调试：显示前几个数据包的解析结果
        if (debug_count <= 3) {
            printf("Ch%d: %.3f ", i, value);
        }
    }
    
    if (debug_count <= 3) {
        std::cout << std::endl;
    }
    
    return true;
}

void ForceSensorReader::reverseBytes(uint8_t* data, size_t size) {
    for (size_t i = 0; i < size / 2; ++i) {
        std::swap(data[i], data[size - 1 - i]);
    }
}

bool ForceSensorReader::isValidForce(float force) {
    const float max_force = 1000.0f;  // 最大合理力值
    return (force >= -max_force && force <= max_force) && std::isfinite(force);
}

void ForceSensorReader::performZeroing() {
    // 计算每个通道的平均零偏值
    for (int channel = 0; channel < 6; ++channel) {
        float sum = 0.0f;
        for (const auto& data_point : zero_data_) {
            sum += data_point[channel];
        }
        zero_offsets_[channel] = sum / zero_data_.size();
    }
    
    std::cout << "零偏值: ";
    for (int i = 0; i < 6; ++i) {
        std::cout << std::fixed << std::setprecision(3) << zero_offsets_[i];
        if (i < 5) std::cout << ", ";
    }
    std::cout << std::endl;
    
    zero_data_.clear();
}

void ForceSensorReader::printStatistics() {
    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - start_time_);
    
    double actual_freq = 0.0;
    if (elapsed.count() > 0) {
        actual_freq = (packet_count_.load() * 1000.0) / elapsed.count();
    }
    
    std::cout << "运行时间: " << elapsed.count() / 1000.0 << "s, "
              << "数据包: " << packet_count_.load() << ", "
              << "实际频率: " << std::fixed << std::setprecision(1) << actual_freq << " Hz"
              << std::endl;
}

int main(int argc, char* argv[]) {
    // 设置信号处理
    std::signal(SIGINT, signalHandler);
    std::signal(SIGTERM, signalHandler);
    
    std::string port = "/dev/ttyUSB0";
    int frequency = 1000;
    
    // 解析命令行参数
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--port" && i + 1 < argc) {
            port = argv[++i];
        } else if (arg == "--freq" && i + 1 < argc) {
            frequency = std::stoi(argv[++i]);
        } else if (arg == "--help") {
            std::cout << "用法: " << argv[0] << " [选项]\n"
                      << "选项:\n"
                      << "  --port <设备>    串口设备路径 (默认: /dev/ttyUSB0)\n"
                      << "  --freq <频率>    采样频率 Hz (默认: 1000)\n"
                      << "  --help          显示此帮助信息\n";
            return 0;
        }
    }
    
    try {
        ForceSensorReader reader(port, frequency);
        g_reader = &reader;
        
        if (!reader.initialize()) {
            std::cerr << "初始化失败" << std::endl;
            return 1;
        }
        
        reader.run();
        
    } catch (const std::exception& e) {
        std::cerr << "程序异常: " << e.what() << std::endl;
        return 1;
    }
    
    std::cout << "程序正常退出" << std::endl;
    return 0;
} 