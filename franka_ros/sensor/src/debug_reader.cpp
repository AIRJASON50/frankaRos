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

#include "serial_port.h"

// 全局控制变量
std::atomic<bool> g_running(true);

void signalHandler(int signal) {
    std::cout << "\n收到信号 " << signal << "，正在关闭程序..." << std::endl;
    g_running = false;
}

// 数据解析函数
bool parseSensorData(const uint8_t* data, uint16_t& package_no, std::array<float, 6>& forces) {
    // 检查帧头
    if (data[0] != 0xAA || data[1] != 0x55) {
        return false;
    }
    
    // 读取数据包长度（大端序）
    uint16_t package_length = (static_cast<uint16_t>(data[2]) << 8) | data[3];
    
    // 读取包编号（大端序）
    package_no = (static_cast<uint16_t>(data[4]) << 8) | data[5];
    
    std::cout << "数据包 " << package_no << " (长度: " << package_length << "): ";
    for (int i = 0; i < std::min(31, static_cast<int>(package_length + 6)); ++i) {
        printf("%02X ", data[i]);
        if (i == 5) std::cout << "| ";  // 分隔数据头和数据部分
    }
    std::cout << std::endl;
    
    // 解析六个通道的数据（从第6字节开始）
    for (int i = 0; i < 6; ++i) {
        uint8_t raw_bytes[4];
        std::memcpy(raw_bytes, &data[6 + i * 4], 4);
        
        // 方式1：直接按IEEE 754浮点数解析
        float value_direct;
        std::memcpy(&value_direct, raw_bytes, 4);
        
        // 方式2：字节反转后解析
        uint8_t reversed[4] = {raw_bytes[3], raw_bytes[2], raw_bytes[1], raw_bytes[0]};
        float value_reversed;
        std::memcpy(&value_reversed, reversed, 4);
        
        // 方式3：当作32位整数解析
        uint32_t int_value;
        std::memcpy(&int_value, raw_bytes, 4);
        uint32_t int_big_endian = __builtin_bswap32(int_value);
        
        printf("Ch%d: [%02X %02X %02X %02X] -> 直接:%.3f, 反转:%.3f, 整数:%u/%u\n", 
               i, raw_bytes[0], raw_bytes[1], raw_bytes[2], raw_bytes[3],
               value_direct, value_reversed, int_value, int_big_endian);
        
        // 选择合理的值
        if (std::isfinite(value_direct) && std::abs(value_direct) < 1000.0f) {
            forces[i] = value_direct;
        } else if (std::isfinite(value_reversed) && std::abs(value_reversed) < 1000.0f) {
            forces[i] = value_reversed;
        } else {
            // 可能是整数需要缩放
            forces[i] = static_cast<float>(static_cast<int32_t>(int_big_endian)) / 1000.0f;
        }
    }
    
    std::cout << "解析结果: ";
    for (int i = 0; i < 6; ++i) {
        printf("%.3f ", forces[i]);
    }
    std::cout << std::endl << std::endl;
    
    return true;
}

int main() {
    // 设置信号处理
    std::signal(SIGINT, signalHandler);
    std::signal(SIGTERM, signalHandler);
    
    std::cout << "=== 六维力传感器数据解析调试程序 ===" << std::endl;
    
    SerialPort serial("/dev/ttyUSB0");
    
    if (!serial.open()) {
        std::cerr << "串口打开失败" << std::endl;
        return 1;
    }
    
    // 配置传感器
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    serial.sendCommand("AT+SMPF=100");  // 降低频率便于调试
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    serial.sendCommand("AT+GSD");
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    serial.flushInput();
    
    std::cout << "开始数据采集，观察前10个数据包..." << std::endl;
    
    uint8_t buffer[31];
    std::array<float, 6> forces;
    uint16_t package_no;
    int packet_count = 0;
    
    while (g_running && packet_count < 10) {
        // 同步数据帧头
        if (!serial.syncData()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
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
            std::this_thread::sleep_for(std::chrono::milliseconds(100));  // 暂停便于观察
        }
    }
    
    serial.sendCommand("AT+STOP");
    serial.close();
    
    std::cout << "调试完成" << std::endl;
    return 0;
} 