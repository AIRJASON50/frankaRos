#pragma once

#include <string>
#include <vector>
#include <cstdint>
#include <memory>

/**
 * @brief 串口通信类，用于与六维力传感器通信
 */
class SerialPort {
public:
    /**
     * @brief 构造函数
     * @param port 串口设备路径，如 "/dev/ttyUSB0"
     * @param baudrate 波特率，默认115200
     */
    SerialPort(const std::string& port = "/dev/ttyUSB0", int baudrate = 115200);
    
    /**
     * @brief 析构函数
     */
    ~SerialPort();

    /**
     * @brief 打开串口
     * @return 成功返回true，失败返回false
     */
    bool open();

    /**
     * @brief 关闭串口
     */
    void close();

    /**
     * @brief 检查串口是否已打开
     * @return 已打开返回true，否则返回false
     */
    bool isOpen() const;

    /**
     * @brief 发送命令到传感器
     * @param command 要发送的命令字符串
     * @return 成功发送的字节数
     */
    int sendCommand(const std::string& command);

    /**
     * @brief 读取指定字节数的数据
     * @param buffer 数据缓冲区
     * @param size 要读取的字节数
     * @return 实际读取的字节数
     */
    int readData(uint8_t* buffer, size_t size);

    /**
     * @brief 同步数据，查找帧头 0xAA 0x55
     * @return 找到帧头返回true，否则返回false
     */
    bool syncData();

    /**
     * @brief 清空接收缓冲区
     */
    void flushInput();

private:
    std::string port_;
    int baudrate_;
    int fd_;  // 文件描述符
    
    /**
     * @brief 配置串口参数
     * @return 成功返回true，失败返回false
     */
    bool configure();
}; 