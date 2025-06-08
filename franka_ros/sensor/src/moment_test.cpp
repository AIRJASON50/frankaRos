#include <iostream>
#include <thread>
#include <chrono>
#include <atomic>
#include <cstring>
#include <array>
#include <vector>
#include <csignal>
#include <cstdint>
#include <iomanip>
#include <cmath>
#include <cstdlib>

#include "serial_port.h"

// 全局控制变量
std::atomic<bool> g_running(true);

void signalHandler(int signal) {
    std::cout << "\n收到信号 " << signal << "，正在关闭程序..." << std::endl;
    g_running = false;
    std::exit(0);
}

// 力矩数据统计
struct MomentStats {
    double sum = 0.0;
    double min_val = 1e6;
    double max_val = -1e6;
    int count = 0;
    
    void add(double val) {
        sum += val;
        min_val = std::min(min_val, val);
        max_val = std::max(max_val, val);
        count++;
    }
    
    double average() const {
        return count > 0 ? sum / count : 0.0;
    }
    
    double range() const {
        return max_val - min_val;
    }
};

// 数据解析函数
bool parseSensorData(const uint8_t* data, uint16_t& package_no, std::array<float, 6>& forces) {
    // 检查帧头
    if (data[0] != 0xAA || data[1] != 0x55) {
        return false;
    }
    
    // 读取包编号（大端序）
    package_no = (static_cast<uint16_t>(data[4]) << 8) | data[5];
    
    // 解析六个通道的数据（从第6字节开始）
    for (int i = 0; i < 6; ++i) {
        uint8_t raw_bytes[4];
        std::memcpy(raw_bytes, &data[6 + i * 4], 4);
        
        // 直接按IEEE 754浮点数解析
        float value;
        std::memcpy(&value, raw_bytes, 4);
        forces[i] = value;
    }
    
    return true;
}

int main() {
    // 设置信号处理
    std::signal(SIGINT, signalHandler);
    std::signal(SIGTERM, signalHandler);
    
    std::cout << "=== 六维力传感器力矩数据分析程序 ===" << std::endl;
    std::cout << "测试力矩通道数据接收和统计..." << std::endl;
    
    SerialPort serial("/dev/ttyUSB0");
    
    if (!serial.open()) {
        std::cerr << "串口打开失败" << std::endl;
        return 1;
    }
    
    // 配置传感器
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    serial.sendCommand("AT+SMPF=500");  // 使用500Hz便于观察
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    serial.sendCommand("AT+GSD");
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    serial.flushInput();
    
    std::cout << "开始数据采集，收集1000个数据包进行分析..." << std::endl;
    std::cout << "请尝试轻微转动传感器以产生力矩数据" << std::endl;
    
    uint8_t buffer[31];
    std::array<float, 6> forces;
    uint16_t package_no;
    int packet_count = 0;
    const int target_packets = 1000;
    
    // 统计数据
    std::array<MomentStats, 6> stats;
    std::array<std::string, 6> labels = {"Fx(N)", "Fy(N)", "Fz(N)", "Mx(N·m)", "My(N·m)", "Mz(N·m)"};
    
    auto start_time = std::chrono::steady_clock::now();
    
    while (g_running && packet_count < target_packets) {
        // 同步数据帧头
        if (!serial.syncData()) {
            std::this_thread::sleep_for(std::chrono::microseconds(100));
            continue;
        }
        
        // 读取剩余29字节
        if (serial.readData(buffer + 2, 29) < 29) {
            continue;
        }
        
        // 补充帧头
        buffer[0] = 0xAA;
        buffer[1] = 0x55;
        
        // 解析数据
        if (parseSensorData(buffer, package_no, forces)) {
            packet_count++;
            
            // 统计每个通道的数据
            for (int i = 0; i < 6; ++i) {
                stats[i].add(forces[i]);
            }
            
            // 每100个包显示一次当前数据
            if (packet_count % 100 == 0) {
                std::cout << "\n包 " << packet_count << ": ";
                for (int i = 0; i < 6; ++i) {
                    printf("%s:%.3f ", labels[i].substr(0,2).c_str(), forces[i]);
                }
                
                // 检查力矩数据变化
                std::cout << "\n力矩变化范围: ";
                for (int i = 3; i < 6; ++i) {
                    printf("%s:[%.4f] ", labels[i].substr(0,2).c_str(), stats[i].range());
                }
                std::cout << std::endl;
            }
        }
    }
    
    auto end_time = std::chrono::steady_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
    
    serial.sendCommand("AT+STOP");
    serial.close();
    
    // 显示统计结果
    std::cout << "\n=== 数据统计结果 ===" << std::endl;
    std::cout << "采集时间: " << duration.count() / 1000.0 << "秒" << std::endl;
    std::cout << "数据包数: " << packet_count << std::endl;
    std::cout << "实际频率: " << (packet_count * 1000.0 / duration.count()) << " Hz\n" << std::endl;
    
    std::cout << std::fixed << std::setprecision(6);
    std::cout << "通道     平均值      最小值      最大值      变化范围    标准差估算" << std::endl;
    std::cout << "--------------------------------------------------------------------" << std::endl;
    
    for (int i = 0; i < 6; ++i) {
        double avg = stats[i].average();
        double range = stats[i].range();
        double std_est = range / 4.0;  // 粗略估算标准差
        
        std::cout << labels[i] << "  " 
                  << std::setw(10) << avg << "  "
                  << std::setw(10) << stats[i].min_val << "  "
                  << std::setw(10) << stats[i].max_val << "  "
                  << std::setw(10) << range << "  "
                  << std::setw(10) << std_est << std::endl;
    }
    
    // 力矩数据分析
    std::cout << "\n=== 力矩数据分析 ===" << std::endl;
    bool has_moment = false;
    for (int i = 3; i < 6; ++i) {
        if (stats[i].range() > 0.001) {  // 如果变化范围大于1mN·m
            has_moment = true;
            std::cout << labels[i] << " 检测到明显变化！范围: " << stats[i].range() << std::endl;
        }
    }
    
    if (!has_moment) {
        std::cout << "未检测到明显的力矩变化。可能原因：" << std::endl;
        std::cout << "1. 传感器未受到明显的扭矩作用" << std::endl;
        std::cout << "2. 传感器安装位置稳定，无旋转力矩" << std::endl;
        std::cout << "3. 力矩测量范围较小，需要更精确的操作" << std::endl;
        std::cout << "\n建议：轻微转动或扭动传感器来产生力矩信号" << std::endl;
    }
    
    return 0;
} 