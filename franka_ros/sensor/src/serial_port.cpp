#include "serial_port.h"
#include <iostream>
#include <cstring>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <sys/ioctl.h>

SerialPort::SerialPort(const std::string& port, int baudrate)
    : port_(port), baudrate_(baudrate), fd_(-1) {
}

SerialPort::~SerialPort() {
    close();
}

bool SerialPort::open() {
    if (fd_ >= 0) {
        std::cout << "串口已经打开" << std::endl;
        return true;
    }

    // 打开串口设备
    fd_ = ::open(port_.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd_ < 0) {
        std::cerr << "无法打开串口: " << port_ << " - " << strerror(errno) << std::endl;
        return false;
    }

    // 配置串口
    if (!configure()) {
        ::close(fd_);
        fd_ = -1;
        return false;
    }

    std::cout << "串口 " << port_ << " 已打开，波特率: " << baudrate_ << std::endl;
    return true;
}

void SerialPort::close() {
    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
        std::cout << "串口已关闭" << std::endl;
    }
}

bool SerialPort::isOpen() const {
    return fd_ >= 0;
}

bool SerialPort::configure() {
    struct termios options;
    
    // 获取当前串口配置
    if (tcgetattr(fd_, &options) != 0) {
        std::cerr << "获取串口配置失败" << std::endl;
        return false;
    }

    // 设置波特率
    speed_t baud;
    switch (baudrate_) {
        case 9600:   baud = B9600; break;
        case 19200:  baud = B19200; break;
        case 38400:  baud = B38400; break;
        case 57600:  baud = B57600; break;
        case 115200: baud = B115200; break;
        case 230400: baud = B230400; break;
        case 460800: baud = B460800; break;
        case 921600: baud = B921600; break;
        default:
            std::cerr << "不支持的波特率: " << baudrate_ << std::endl;
            return false;
    }
    
    cfsetispeed(&options, baud);
    cfsetospeed(&options, baud);

    // 设置数据位、停止位、校验位
    options.c_cflag &= ~PARENB;   // 无校验
    options.c_cflag &= ~CSTOPB;   // 1个停止位
    options.c_cflag &= ~CSIZE;    // 清除数据位设置
    options.c_cflag |= CS8;       // 8个数据位
    options.c_cflag |= CREAD | CLOCAL; // 启用接收器，忽略调制解调器状态线

    // 设置输入模式
    options.c_iflag &= ~(IXON | IXOFF | IXANY); // 禁用软件流控制
    options.c_iflag &= ~(INLCR | ICRNL);        // 禁用换行转换

    // 设置输出模式
    options.c_oflag &= ~OPOST; // 禁用输出处理

    // 设置本地模式
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); // 原始输入模式

    // 设置超时
    options.c_cc[VMIN] = 0;   // 非阻塞读取
    options.c_cc[VTIME] = 1;  // 0.1秒超时

    // 应用配置
    if (tcsetattr(fd_, TCSANOW, &options) != 0) {
        std::cerr << "设置串口配置失败" << std::endl;
        return false;
    }

    // 清空缓冲区
    tcflush(fd_, TCIOFLUSH);
    
    return true;
}

int SerialPort::sendCommand(const std::string& command) {
    if (fd_ < 0) {
        std::cerr << "串口未打开" << std::endl;
        return -1;
    }

    std::string cmd = command;
    if (cmd.back() != '\n') {
        if (cmd.back() != '\r') {
            cmd += "\r\n";
        } else {
            cmd += "\n";
        }
    } else if (cmd.length() < 2 || cmd.substr(cmd.length() - 2) != "\r\n") {
        cmd = cmd.substr(0, cmd.length() - 1) + "\r\n";
    }

    ssize_t bytes_written = write(fd_, cmd.c_str(), cmd.length());
    if (bytes_written < 0) {
        std::cerr << "发送命令失败: " << strerror(errno) << std::endl;
        return -1;
    }

    std::cout << "发送命令: " << command << std::endl;
    return bytes_written;
}

int SerialPort::readData(uint8_t* buffer, size_t size) {
    if (fd_ < 0) {
        return -1;
    }

    ssize_t bytes_read = read(fd_, buffer, size);
    return bytes_read;
}

bool SerialPort::syncData() {
    if (fd_ < 0) {
        return false;
    }

    uint8_t byte;
    while (true) {
        if (readData(&byte, 1) <= 0) {
            continue;
        }
        
        if (byte == 0xAA) {
            if (readData(&byte, 1) <= 0) {
                continue;
            }
            if (byte == 0x55) {
                return true;
            }
        }
    }
}

void SerialPort::flushInput() {
    if (fd_ >= 0) {
        tcflush(fd_, TCIFLUSH);
    }
} 