#include <ros/ros.h>
#include <geometry_msgs/WrenchStamped.h>
#include <serial/serial.h>
#include <iostream>
#include <vector>
#include <deque>
#include <thread>
#include <mutex>
#include <chrono>
#include <iomanip>
#include <std_msgs/Float64.h>

class ForceSensorReader {
private:
    serial::Serial ser_;
    ros::Publisher wrench_pub_;
    ros::Publisher raw_wrench_pub_;
    ros::Publisher frequency_pub_;
    std::thread read_thread_;
    mutable std::mutex data_mutex_;
    bool running_;
    bool is_zeroing_;
    
    // 调零相关
    std::vector<std::deque<double>> zero_data_;  // 6个通道的调零数据
    std::vector<double> zero_offsets_;           // 调零偏移值
    static const int ZERO_SAMPLES = 100;        // 调零样本数量（增加到100以提高精度）
    
    // 持续校准
    std::vector<std::deque<double>> drift_monitor_;  // 监测数据漂移
    static const int DRIFT_WINDOW_SIZE = 50;        // 漂移检测窗口大小
    std::chrono::steady_clock::time_point last_drift_check_;
    bool enable_drift_correction_;                   // 是否启用漂移校正
    
    // 数据缓冲
    std::deque<std::vector<double>> data_buffer_;
    static const int BUFFER_SIZE = 100;
    
    // ==== 新增: 低通滤波相关 ====
    double filter_alpha_;                       // EMA平滑系数
    bool filter_initialized_;
    std::vector<double> filtered_forces_;       // 上一次滤波结果
    // ============================
    
    // 频率测量
    std::chrono::steady_clock::time_point last_packet_time_;
    std::deque<double> frequency_buffer_;
    double current_frequency_;
    
    // 通道标签
    std::vector<std::string> labels_ = {"Fx", "Fy", "Fz", "Mx", "My", "Mz"};

public:
    ForceSensorReader(ros::NodeHandle& nh) : 
        running_(false), 
        is_zeroing_(true),
        zero_data_(6),
        zero_offsets_(6, 0.0),
        current_frequency_(0.0),
        drift_monitor_(6),
        enable_drift_correction_(true) {
        
        // ==== 读取滤波参数并初始化 ====
        ros::NodeHandle pnh("~");
        pnh.param("force_sensor/filter_alpha", filter_alpha_, 0.2); // 默认alpha=0.2
        filter_initialized_ = false;
        filtered_forces_.assign(6, 0.0);
        // =================================
        
        // 初始化发布器 - 提高队列大小和发布频率
        wrench_pub_ = nh.advertise<geometry_msgs::WrenchStamped>("force_sensor/wrench", 1000);
        raw_wrench_pub_ = nh.advertise<geometry_msgs::WrenchStamped>("force_sensor/raw_wrench", 1000);
        frequency_pub_ = nh.advertise<std_msgs::Float64>("force_sensor/frequency", 10);
        
        // 自动检测最优的USB端口和波特率
        std::string port;
        int baudrate;
        if (!findOptimalBaudrate(port, baudrate)) {
            ROS_ERROR("Force Sensor: Failed to find compatible USB device with sensor!");
            return;
        }
        
        ROS_INFO("Force Sensor: Auto-configured on port: %s, baudrate: %d", port.c_str(), baudrate);
        
        // 初始化串口
        try {
            ser_.setPort(port);
            ser_.setBaudrate(baudrate);
            ser_.setBytesize(serial::eightbits);
            ser_.setParity(serial::parity_none);
            ser_.setStopbits(serial::stopbits_one);
            serial::Timeout timeout = serial::Timeout::simpleTimeout(1000);
            ser_.setTimeout(timeout);
            
            ser_.open();
            if (!ser_.isOpen()) {
                ROS_ERROR("Force Sensor: Failed to open serial port %s", port.c_str());
                return;
            }
            
            ROS_INFO("Force Sensor: Serial port %s opened successfully", port.c_str());
            
            // 清空串口缓冲区
            ser_.flushInput();
            ser_.flushOutput();
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            
            // 配置传感器到最优参数
            if (!configureSensorOptimal()) {
                ROS_ERROR("Force Sensor: Failed to configure sensor to optimal parameters");
                return;
            }
            
            // 初始化频率测量
            last_packet_time_ = std::chrono::steady_clock::now();
            frequency_buffer_.clear();
            
            // 启动数据读取线程
            running_ = true;
            read_thread_ = std::thread(&ForceSensorReader::readDataThread, this);
            
            ROS_INFO("Force Sensor: High-frequency data acquisition started (target: 1000Hz)");
            
        } catch (const std::exception& e) {
            ROS_ERROR("Force Sensor: Failed to initialize serial communication: %s", e.what());
            return;
        }
    }
    
    ~ForceSensorReader() {
        stop();
    }
    
    void stop() {
        if (running_) {
            running_ = false;
            if (read_thread_.joinable()) {
                read_thread_.join();
            }
            
            if (ser_.isOpen()) {
                sendCommand("AT+STOP");
                ser_.close();
                ROS_INFO("Force sensor serial port closed");
            }
        }
    }
    
    bool isZeroingComplete() const {
        std::lock_guard<std::mutex> lock(data_mutex_);
        return !is_zeroing_;
    }
    
    std::vector<double> getZeroOffsets() const {
        std::lock_guard<std::mutex> lock(data_mutex_);
        return zero_offsets_;
    }

private:
    bool findOptimalBaudrate(std::string& port, int& baudrate) {
        std::vector<std::string> possible_ports = {"/dev/ttyUSB0", "/dev/ttyUSB1", "/dev/ttyUSB2", "/dev/ttyUSB3"};
        std::vector<int> possible_baudrates = {921600, 460800, 230400, 115200, 57600};
        
        for (const std::string& test_port : possible_ports) {
            for (int test_baudrate : possible_baudrates) {
                try {
                    serial::Serial test_serial;
                    test_serial.setPort(test_port);
                    test_serial.setBaudrate(test_baudrate);
                    test_serial.setBytesize(serial::eightbits);
                    test_serial.setParity(serial::parity_none);
                    test_serial.setStopbits(serial::stopbits_one);
                    serial::Timeout timeout = serial::Timeout::simpleTimeout(1000);
                    test_serial.setTimeout(timeout);
                    
                    test_serial.open();
                    if (test_serial.isOpen()) {
                        // 清空缓冲区
                        test_serial.flushInput();
                        test_serial.flushOutput();
                        std::this_thread::sleep_for(std::chrono::milliseconds(100));
                        
                        // 测试基本通信 - 查询软件版本
                        test_serial.write("AT+SFWV=?\r\n");
                        std::this_thread::sleep_for(std::chrono::milliseconds(200));
                        
                        if (test_serial.available() > 0) {
                            std::string response;
                            response = test_serial.read(test_serial.available());
                            if (response.find("ACK+SFWV") != std::string::npos) {
                                ROS_INFO("Force Sensor: Found compatible device at %s with baudrate %d", 
                                         test_port.c_str(), test_baudrate);
                                test_serial.close();
                                port = test_port;
                                baudrate = test_baudrate;
                                return true;
                            }
                        }
                        
                        test_serial.close();
                    }
                } catch (const std::exception& e) {
                    // Port not available, try next
                    continue;
                }
            }
        }
        
        ROS_ERROR("No compatible force sensor found on any port with any baudrate");
        return false;
    }

    bool configureSensorOptimal() {
        try {
            // 1. 查询当前配置
            ROS_INFO("Force Sensor: Querying current configuration...");
            
            // 查询当前采样频率
            sendCommandWithResponse("AT+SMPF=?");
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
            
            // 2. 根据手册配置最优波特率 (针对1000Hz)
            ROS_INFO("Force Sensor: Configuring optimal baudrate for 1000Hz...");
            sendCommandWithResponse("AT+UARTCFG=921600,8,1,0");  // 921600波特率，8数据位，1停止位，无校验
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            
            // 3. 设置1000Hz采样率
            ROS_INFO("Force Sensor: Setting sampling rate to 1000Hz...");
            sendCommandWithResponse("AT+SMPF=1000");
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
            
            // 4. 设置数据校验方式为SUM (更快)
            ROS_INFO("Force Sensor: Setting checksum method to SUM for higher speed...");
            sendCommandWithResponse("AT+DCKMD=SUM");
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
            
            // 5. 查询解耦矩阵配置单位
            sendCommandWithResponse("AT+DCPCU=?");
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
            
            // 6. 开始连续数据传输
            ROS_INFO("Force Sensor: Starting continuous data transmission...");
            sendCommand("AT+GSD");
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            
            ROS_INFO("Force Sensor: Optimal configuration completed - 1000Hz @ 921600 baud");
            return true;
            
        } catch (const std::exception& e) {
            ROS_ERROR("Force Sensor: Configuration failed: %s", e.what());
            return false;
        }
    }

    std::string findUSBPort() {
        std::vector<std::string> possible_ports = {"/dev/ttyUSB0", "/dev/ttyUSB1", "/dev/ttyUSB2", "/dev/ttyUSB3"};
        
        for (const std::string& port : possible_ports) {
            try {
                serial::Serial test_serial;
                test_serial.setPort(port);
                test_serial.setBaudrate(115200);
                test_serial.setBytesize(serial::eightbits);
                test_serial.setParity(serial::parity_none);
                test_serial.setStopbits(serial::stopbits_one);
                serial::Timeout timeout = serial::Timeout::simpleTimeout(100);
                test_serial.setTimeout(timeout);
                
                test_serial.open();
                if (test_serial.isOpen()) {
                    test_serial.close();
                    ROS_INFO("Found USB device at: %s", port.c_str());
                    return port;
                }
            } catch (const std::exception& e) {
                // Port not available or permission denied, try next
                continue;
            }
        }
        
        ROS_WARN("No USB serial devices found in standard ports (/dev/ttyUSB0-3)");
        return "";
    }
    
    void setupSensor() {
        // 设置采样率1000Hz (最大频率)
        sendCommand("AT+SMPF=1000");
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        
        // 开始连续传输
        sendCommand("AT+GSD");
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
    
    void sendCommand(const std::string& command) {
        try {
            std::string cmd = command;
            if (cmd.find("\r\n") == std::string::npos) {
                cmd += "\r\n";
            }
            ser_.write(cmd);
            ROS_DEBUG("Sent command: %s", command.c_str());
        } catch (const std::exception& e) {
            ROS_ERROR("Failed to send command: %s", e.what());
        }
    }
    
    void sendCommandWithResponse(const std::string& command) {
        try {
            std::string cmd = command;
            if (cmd.find("\r\n") == std::string::npos) {
                cmd += "\r\n";
            }
            ser_.write(cmd);
            
            // 等待响应
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            if (ser_.available() > 0) {
                std::string response = ser_.read(ser_.available());
                ROS_DEBUG("Command: %s, Response: %s", command.c_str(), response.c_str());
            }
        } catch (const std::exception& e) {
            ROS_ERROR("Failed to send command with response: %s", e.what());
        }
    }
    
    void reverseBytes(uint8_t* data, size_t length) {
        for (size_t i = 0; i < length / 2; ++i) {
            std::swap(data[i], data[length - 1 - i]);
        }
    }
    
    bool parseSensorData(const std::vector<uint8_t>& data, uint16_t& package_no, std::vector<double>& forces) {
        try {
            // 检查帧头
            if (data.size() < 31 || data[0] != 0xAA || data[1] != 0x55) {
                ROS_DEBUG("Invalid data header, data size: %zu", data.size());
                return false;
            }
            
            // 获取包编号 (位置4-5，高字节在前)
            package_no = (static_cast<uint16_t>(data[4]) << 8) | data[5];
            
            // 解析6个通道的力数据（与Python版本完全一致）
            forces.resize(6);
            for (int i = 0; i < 6; ++i) {
                uint8_t raw_bytes[4];
                // 复制4字节数据
                std::copy(data.begin() + 6 + i * 4, data.begin() + 10 + i * 4, raw_bytes);
                
                // 与Python版本一致：先反转字节序
                reverseBytes(raw_bytes, 4);
                
                // 与Python版本一致：按大端序解析浮点数（相当于Python的struct.unpack('>f')）
                uint32_t int_val = (static_cast<uint32_t>(raw_bytes[0]) << 24) |
                                   (static_cast<uint32_t>(raw_bytes[1]) << 16) |
                                   (static_cast<uint32_t>(raw_bytes[2]) << 8)  |
                                   static_cast<uint32_t>(raw_bytes[3]);
                
                // 转换为IEEE 754浮点数
                float force_value;
                std::memcpy(&force_value, &int_val, sizeof(float));
                forces[i] = static_cast<double>(force_value);
            }
            
            return true;
        } catch (const std::exception& e) {
            ROS_ERROR("Data parsing error: %s", e.what());
            return false;
        }
    }
    
    bool isValidForce(double force, double min_val = -1000.0, double max_val = 1000.0) {
        return force >= min_val && force <= max_val && std::isfinite(force);
    }
    
    bool syncData() {
        try {
            // 查找帧头 0xAA 0x55（与Python版本完全一致的逻辑）
            while (running_) {
                uint8_t byte1;
                // 逐字节读取，超时设置为100ms
                if (ser_.read(&byte1, 1) == 1) {
                    if (byte1 == 0xAA) {
                        uint8_t byte2;
                        if (ser_.read(&byte2, 1) == 1) {
                            if (byte2 == 0x55) {
                                return true;  // 找到帧头
                            }
                        }
                    }
                } else {
                    // 读取超时，检查是否需要继续
                    std::this_thread::sleep_for(std::chrono::milliseconds(1));
                }
            }
            return false;
        } catch (const std::exception& e) {
            ROS_ERROR("Data synchronization failed: %s", e.what());
            return false;
        }
    }
    
    void readDataThread() {
        std::vector<uint8_t> buffer;
        bool header_found = false;
        uint16_t expected_length = 0;
        
        // 频率测量初始化
        auto last_freq_publish = std::chrono::steady_clock::now();
        const auto freq_publish_interval = std::chrono::milliseconds(1000); // 每秒发布一次频率
        
        ROS_INFO("Force Sensor: Data reading thread started, waiting for data...");
        
        while (running_ && ros::ok()) {
            try {
                if (ser_.available() > 0) {
                    std::string data = ser_.read(ser_.available());
                    
                    for (char byte : data) {
                        buffer.push_back(static_cast<uint8_t>(byte));
                        
                        if (!header_found) {
                            // 寻找帧头 0xAA 0x55
                            if (buffer.size() >= 2 && 
                                buffer[buffer.size()-2] == 0xAA && 
                                buffer[buffer.size()-1] == 0x55) {
                                buffer.clear();
                                buffer.push_back(0xAA);
                                buffer.push_back(0x55);
                                header_found = true;
                            } else if (buffer.size() > 1000) {
                                buffer.clear(); // 防止缓冲区无限增长
                            }
                        } else {
                            // 已找到帧头，检查包长度
                            if (buffer.size() == 4) {
                                // 读取包长度 (高字节在前)
                                expected_length = (static_cast<uint16_t>(buffer[2]) << 8) | buffer[3];
                                if (expected_length > 1000 || expected_length < 20) {
                                    // 无效长度，重新寻找帧头
                                    buffer.clear();
                                    header_found = false;
                                    continue;
                                }
                            }
                            
                            // 检查是否接收完整包
                            if (expected_length > 0 && buffer.size() >= (expected_length + 6)) {
                                // 有完整数据包，尝试解析
                                std::vector<uint8_t> packet(buffer.begin(), buffer.begin() + expected_length + 6);
                                buffer.erase(buffer.begin(), buffer.begin() + expected_length + 6);
                                header_found = false;
                                expected_length = 0;
                                
                                // 解析数据包
                                uint16_t package_no;
                                std::vector<double> forces;
                                if (parseSensorData(packet, package_no, forces)) {
                                    // 计算数据频率
                                    updateFrequency();
                                    
                                    if (is_zeroing_) {
                                        processZeroingData(forces);
                                    } else {
                                        // 应用调零偏移并检测漂移
                                        std::vector<double> corrected_forces = forces;
                                        for (size_t i = 0; i < forces.size() && i < zero_offsets_.size(); ++i) {
                                            corrected_forces[i] -= zero_offsets_[i];
                                        }
                                        
                                        // 漂移监测和校正
                                        if (enable_drift_correction_) {
                                            checkAndCorrectDrift(corrected_forces, forces);
                                        }
                                        
                                        // 低通滤波，减少毛刺
                                        std::vector<double> smooth_forces = applyLowPassFilter(corrected_forces);
                                        publishWrench(smooth_forces, false, package_no);
                                    }
                                    
                                    // 始终发布原始数据
                                    publishWrench(forces, true, package_no);
                                }
                            }
                        }
                    }
                }
                
                // 定期发布频率信息
                auto now = std::chrono::steady_clock::now();
                if (now - last_freq_publish >= freq_publish_interval) {
                    publishFrequency();
                    last_freq_publish = now;
                }
                
            } catch (const std::exception& e) {
                ROS_ERROR("Force Sensor: Data reading error: %s", e.what());
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
            
            // Reduce delay to support higher frequency while maintaining system stability
            std::this_thread::sleep_for(std::chrono::milliseconds(1)); // 1ms delay, balance between performance and stability
        }
        
        ROS_INFO("Force Sensor: Data reading thread terminated");
    }
    
    void updateFrequency() {
        auto now = std::chrono::steady_clock::now();
        if (last_packet_time_.time_since_epoch().count() > 0) {
            auto duration = std::chrono::duration_cast<std::chrono::microseconds>(now - last_packet_time_);
            double interval_ms = duration.count() / 1000.0;
            if (interval_ms > 0.1 && interval_ms < 1000) { // 合理范围内
                double freq = 1000.0 / interval_ms;
                frequency_buffer_.push_back(freq);
                if (frequency_buffer_.size() > 20) {
                    frequency_buffer_.pop_front();
                }
                
                // 计算平均频率
                double sum = 0;
                for (double f : frequency_buffer_) {
                    sum += f;
                }
                current_frequency_ = sum / frequency_buffer_.size();
            }
        }
        last_packet_time_ = now;
    }
    
    void publishFrequency() {
        std::lock_guard<std::mutex> lock(data_mutex_);
        std_msgs::Float64 freq_msg;
        freq_msg.data = current_frequency_;
        frequency_pub_.publish(freq_msg);
    }
    
    void processZeroingData(const std::vector<double>& forces) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        
        // Force validation - 调零阶段使用更宽松的验证条件
        bool valid = true;
        for (const auto& force : forces) {
            // 调零阶段使用更大的范围：-2000到2000
            if (!isValidForce(force, -2000.0, 2000.0)) {
                valid = false;
                break;
            }
        }
        
        if (!valid) {
            // 输出无效数据的调试信息
            static int invalid_count = 0;
            if (++invalid_count % 10 == 0) {  // 每10次打印一次
                ROS_WARN("Force Sensor: Invalid data detected during zeroing (count: %d): [%.1f, %.1f, %.1f, %.1f, %.1f, %.1f]",
                         invalid_count, forces[0], forces[1], forces[2], forces[3], forces[4], forces[5]);
            }
            return;  // Skip invalid data
        }
        
        // Add data to each channel
        for (int i = 0; i < 6; ++i) {
            zero_data_[i].push_back(forces[i]);
        }
        
        // Progress notification (2Hz max)
        static auto last_zeroing_print = std::chrono::steady_clock::now();
        auto now = std::chrono::steady_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_zeroing_print);
        
        if (duration.count() > 500) {  // Print every 500ms during zeroing
            ROS_INFO("Force Sensor: Zeroing progress: %zu/%d samples collected", 
                     zero_data_[0].size(), ZERO_SAMPLES);
            last_zeroing_print = now;
        }
        
        if (zero_data_[0].size() >= ZERO_SAMPLES) {
            // Calculate final zero offsets
            for (int i = 0; i < 6; ++i) {
                double sum = 0.0;
                for (double value : zero_data_[i]) {
                    sum += value;
                }
                zero_offsets_[i] = sum / zero_data_[i].size();
            }
            
            is_zeroing_ = false;
            ROS_INFO("Force Sensor: Zeroing completed. Zero offsets: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f]",
                     zero_offsets_[0], zero_offsets_[1], zero_offsets_[2], 
                     zero_offsets_[3], zero_offsets_[4], zero_offsets_[5]);
                     
            // Output zeroing completion message in English
            ROS_INFO("Force sensor zeroing completed. Zero offsets: Fx=%.3f, Fy=%.3f, Fz=%.3f, Mx=%.3f, My=%.3f, Mz=%.3f",
                     zero_offsets_[0], zero_offsets_[1], zero_offsets_[2], 
                     zero_offsets_[3], zero_offsets_[4], zero_offsets_[5]);
        }
    }
    
    void publishWrench(const std::vector<double>& forces, bool is_raw, uint16_t package_no) {
        if (forces.size() != 6) return;
        
        geometry_msgs::WrenchStamped wrench_msg;
        wrench_msg.header.stamp = ros::Time::now();
        wrench_msg.header.frame_id = "force_sensor_frame";
        
        // 填充力和力矩数据
        wrench_msg.wrench.force.x = forces[0];
        wrench_msg.wrench.force.y = forces[1];
        wrench_msg.wrench.force.z = forces[2];
        wrench_msg.wrench.torque.x = forces[3];
        wrench_msg.wrench.torque.y = forces[4];
        wrench_msg.wrench.torque.z = forces[5];
        
        // 使用高频发布 (300Hz) - 降低发布延迟
        if (is_raw) {
            raw_wrench_pub_.publish(wrench_msg);
        } else {
            wrench_pub_.publish(wrench_msg);
        }
        
        // 减少日志频率到每500个包
        static uint16_t log_counter = 0;
        if (++log_counter % 500 == 0) {
            ROS_DEBUG("Force Sensor: Published %s data #%u: F[%.3f,%.3f,%.3f] M[%.3f,%.3f,%.3f] @ %.1fHz",
                      is_raw ? "raw" : "processed", package_no,
                      forces[0], forces[1], forces[2],
                      forces[3], forces[4], forces[5],
                      current_frequency_);
        }
    }
    
    void checkAndCorrectDrift(std::vector<double>& corrected_forces, const std::vector<double>& raw_forces) {
        auto now = std::chrono::steady_clock::now();
        
        // 每5秒检查一次漂移
        if (last_drift_check_.time_since_epoch().count() > 0 && 
            std::chrono::duration_cast<std::chrono::seconds>(now - last_drift_check_).count() < 5) {
            return;
        }
        
        last_drift_check_ = now;
        
        // 添加当前校正后的力值到漂移监测窗口
        for (size_t i = 0; i < 6 && i < corrected_forces.size(); ++i) {
            drift_monitor_[i].push_back(corrected_forces[i]);
            if (drift_monitor_[i].size() > DRIFT_WINDOW_SIZE) {
                drift_monitor_[i].pop_front();
            }
        }
        
        // 检测漂移：如果最近50个值的均值偏离零点超过阈值，进行校正
        bool need_correction = false;
        std::vector<double> drift_offsets(6, 0.0);
        
        for (size_t i = 0; i < 6; ++i) {
            if (drift_monitor_[i].size() >= DRIFT_WINDOW_SIZE) {
                double sum = 0.0;
                for (double val : drift_monitor_[i]) {
                    sum += val;
                }
                double mean = sum / drift_monitor_[i].size();
                
                // 检查是否存在显著漂移（阈值：力>0.1N，力矩>0.01N·m）
                double threshold = (i < 3) ? 0.1 : 0.01;  // 前3个是力，后3个是力矩
                
                if (std::abs(mean) > threshold) {
                    drift_offsets[i] = mean;
                    need_correction = true;
                }
            }
        }
        
        if (need_correction) {
            ROS_INFO("Force Sensor: Drift detected, applying correction...");
            for (size_t i = 0; i < 6; ++i) {
                if (std::abs(drift_offsets[i]) > 0.01) {  // 只校正有意义的漂移
                    zero_offsets_[i] += drift_offsets[i];
                    drift_monitor_[i].clear();  // 清空监测窗口
                    ROS_INFO("Channel %zu drift correction: %.3f", i, drift_offsets[i]);
                }
            }
        }
    }

    // ==== 新增: 低通滤波实现 ====
    std::vector<double> applyLowPassFilter(const std::vector<double>& input) {
        if (!filter_initialized_) {
            filtered_forces_ = input;
            filter_initialized_ = true;
            return filtered_forces_;
        }
        for (size_t i = 0; i < input.size() && i < filtered_forces_.size(); ++i) {
            filtered_forces_[i] = filter_alpha_ * input[i] + (1.0 - filter_alpha_) * filtered_forces_[i];
        }
        return filtered_forces_;
    }
    // ============================
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "force_sensor_reader");
    ros::NodeHandle nh;
    
    try {
        ForceSensorReader sensor_reader(nh);
        
        ROS_INFO("Force sensor reader node started, waiting for zeroing to complete...");
        
        // 等待调零完成，设置30秒超时
        auto start_time = std::chrono::steady_clock::now();
        const auto timeout_duration = std::chrono::seconds(30);
        
        while (ros::ok() && !sensor_reader.isZeroingComplete()) {
            ros::spinOnce();
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            
            // 检查超时
            auto current_time = std::chrono::steady_clock::now();
            if (current_time - start_time > timeout_duration) {
                ROS_WARN("Force Sensor: Zeroing timeout after 30 seconds, continuing with default offsets...");
                break;
            }
        }
        
        if (sensor_reader.isZeroingComplete()) {
            ROS_INFO("Force sensor zeroing complete, starting normal operation");
            
            // Get final zero offsets
            std::vector<double> offsets = sensor_reader.getZeroOffsets();
            ROS_INFO("Final zero offsets: Fx=%.3f, Fy=%.3f, Fz=%.3f, Mx=%.3f, My=%.3f, Mz=%.3f",
                    offsets[0], offsets[1], offsets[2], offsets[3], offsets[4], offsets[5]);
        } else {
            ROS_WARN("Force sensor zeroing not completed, but continuing (may use default zero offsets)");
        }
        
        // 主循环
        ROS_INFO("Force sensor reader node entering main loop...");
        ros::spin();
        
    } catch (const std::exception& e) {
        ROS_ERROR("Force sensor reader node exception: %s", e.what());
        return 1;
    }
    
    ROS_INFO("Force sensor reader node terminated");
    return 0;
}

