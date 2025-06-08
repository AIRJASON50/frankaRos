#include "data_visualizer.h"
#include <iostream>
#include <iomanip>
#include <sstream>

DataVisualizer::DataVisualizer(const std::string& window_name, int max_points)
    : window_name_(window_name), max_points_(max_points),
      window_size_(1200, 800), y_min_(-10.0f), y_max_(10.0f), show_grid_(true) {
    
    // 初始化通道标签
    labels_ = {"Fx", "Fy", "Fz", "Mx", "My", "Mz"};
    
    // 初始化通道颜色 (BGR格式)
    colors_ = {
        cv::Scalar(0, 0, 255),     // 红色 - Fx
        cv::Scalar(0, 255, 0),     // 绿色 - Fy  
        cv::Scalar(255, 0, 0),     // 蓝色 - Fz
        cv::Scalar(0, 255, 255),   // 黄色 - Mx
        cv::Scalar(255, 0, 255),   // 紫色 - My
        cv::Scalar(255, 255, 0)    // 青色 - Mz
    };
    
    // 创建窗口
    cv::namedWindow(window_name_, cv::WINDOW_AUTOSIZE);
    
    std::cout << "数据可视化窗口已创建: " << window_name_ << std::endl;
    std::cout << "按ESC键退出程序" << std::endl;
}

DataVisualizer::~DataVisualizer() {
    cv::destroyWindow(window_name_);
}

void DataVisualizer::addDataPoint(const std::array<float, 6>& forces) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    
    for (int i = 0; i < 6; ++i) {
        data_buffers_[i].push_back(forces[i]);
        
        // 保持缓冲区大小不超过最大值
        if (data_buffers_[i].size() > static_cast<size_t>(max_points_)) {
            data_buffers_[i].pop_front();
        }
    }
}

bool DataVisualizer::updateDisplay() {
    // 创建图像
    cv::Mat image = cv::Mat::zeros(window_size_, CV_8UC3);
    
    // 绘制图表
    drawChart(image);
    
    // 显示图像
    cv::imshow(window_name_, image);
    
    // 检查用户输入
    int key = cv::waitKey(1) & 0xFF;
    if (key == 27) { // ESC键
        return false;
    } else if (key == 'r' || key == 'R') { // R键重置
        resetData();
    }
    
    return true;
}

void DataVisualizer::resetData() {
    std::lock_guard<std::mutex> lock(data_mutex_);
    
    for (auto& buffer : data_buffers_) {
        buffer.clear();
    }
    
    std::cout << "数据已重置" << std::endl;
}

void DataVisualizer::setYRange(float min_val, float max_val) {
    y_min_ = min_val;
    y_max_ = max_val;
}

void DataVisualizer::setShowGrid(bool show_grid) {
    show_grid_ = show_grid;
}

void DataVisualizer::drawChart(cv::Mat& image) {
    // 定义图表区域
    const int margin_left = 80;
    const int margin_right = 150;
    const int margin_top = 50;
    const int margin_bottom = 80;
    
    const int chart_width = window_size_.width - margin_left - margin_right;
    const int chart_height = window_size_.height - margin_top - margin_bottom;
    
    // 绘制背景
    cv::rectangle(image, 
                  cv::Point(margin_left, margin_top),
                  cv::Point(margin_left + chart_width, margin_top + chart_height),
                  cv::Scalar(30, 30, 30), -1);
    
    // 绘制网格
    if (show_grid_) {
        drawGrid(image);
    }
    
    // 绘制坐标轴
    drawAxes(image);
    
    // 绘制数据曲线
    for (int channel = 0; channel < 6; ++channel) {
        drawDataLine(image, channel);
    }
    
    // 绘制图例
    drawLegend(image);
    
    // 显示实时数据值
    std::lock_guard<std::mutex> lock(data_mutex_);
    int text_y = 30;
    for (int i = 0; i < 6; ++i) {
        if (!data_buffers_[i].empty()) {
            float current_value = data_buffers_[i].back();
            std::stringstream ss;
            ss << labels_[i] << ": " << std::fixed << std::setprecision(3) << current_value;
            
            cv::putText(image, ss.str(), cv::Point(10, text_y), 
                       cv::FONT_HERSHEY_SIMPLEX, 0.6, colors_[i], 2);
            text_y += 25;
        }
    }
}

void DataVisualizer::drawGrid(cv::Mat& image) {
    const int margin_left = 80;
    const int margin_top = 50;
    const int chart_width = window_size_.width - margin_left - 150;
    const int chart_height = window_size_.height - margin_top - 80;
    
    const cv::Scalar grid_color(50, 50, 50);
    
    // 绘制水平网格线
    const int h_lines = 10;
    for (int i = 0; i <= h_lines; ++i) {
        int y = margin_top + (chart_height * i) / h_lines;
        cv::line(image, 
                cv::Point(margin_left, y), 
                cv::Point(margin_left + chart_width, y), 
                grid_color, 1);
    }
    
    // 绘制垂直网格线
    const int v_lines = 10;
    for (int i = 0; i <= v_lines; ++i) {
        int x = margin_left + (chart_width * i) / v_lines;
        cv::line(image, 
                cv::Point(x, margin_top), 
                cv::Point(x, margin_top + chart_height), 
                grid_color, 1);
    }
}

void DataVisualizer::drawAxes(cv::Mat& image) {
    const int margin_left = 80;
    const int margin_top = 50;
    const int chart_width = window_size_.width - margin_left - 150;
    const int chart_height = window_size_.height - margin_top - 80;
    
    const cv::Scalar axis_color(200, 200, 200);
    
    // X轴
    cv::line(image, 
            cv::Point(margin_left, margin_top + chart_height), 
            cv::Point(margin_left + chart_width, margin_top + chart_height), 
            axis_color, 2);
    
    // Y轴
    cv::line(image, 
            cv::Point(margin_left, margin_top), 
            cv::Point(margin_left, margin_top + chart_height), 
            axis_color, 2);
    
    // Y轴标签
    const int y_labels = 5;
    for (int i = 0; i <= y_labels; ++i) {
        float value = y_max_ - (y_max_ - y_min_) * i / y_labels;
        int y = margin_top + (chart_height * i) / y_labels;
        
        std::stringstream ss;
        ss << std::fixed << std::setprecision(1) << value;
        
        cv::putText(image, ss.str(), cv::Point(10, y + 5), 
                   cv::FONT_HERSHEY_SIMPLEX, 0.5, axis_color, 1);
    }
    
    // 坐标轴标题
    cv::putText(image, "Time", 
               cv::Point(margin_left + chart_width/2 - 20, window_size_.height - 20), 
               cv::FONT_HERSHEY_SIMPLEX, 0.7, axis_color, 2);
    
    cv::putText(image, "Force/Moment", 
               cv::Point(10, margin_top + chart_height/2), 
               cv::FONT_HERSHEY_SIMPLEX, 0.7, axis_color, 2);
}

void DataVisualizer::drawLegend(cv::Mat& image) {
    const int legend_x = window_size_.width - 140;
    const int legend_y_start = 60;
    const int line_spacing = 30;
    
    cv::putText(image, "Legend:", cv::Point(legend_x, legend_y_start - 20), 
               cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 255), 2);
    
    for (int i = 0; i < 6; ++i) {
        int y = legend_y_start + i * line_spacing;
        
        // 绘制颜色线条
        cv::line(image, 
                cv::Point(legend_x, y), 
                cv::Point(legend_x + 30, y), 
                colors_[i], 3);
        
        // 绘制标签
        cv::putText(image, labels_[i], cv::Point(legend_x + 40, y + 5), 
                   cv::FONT_HERSHEY_SIMPLEX, 0.6, colors_[i], 2);
    }
}

void DataVisualizer::drawDataLine(cv::Mat& image, int channel) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    
    if (data_buffers_[channel].size() < 2) {
        return;
    }
    
    const int margin_left = 80;
    const int margin_top = 50;
    const int chart_width = window_size_.width - margin_left - 150;
    const int chart_height = window_size_.height - margin_top - 80;
    
    const auto& data = data_buffers_[channel];
    const size_t data_size = data.size();
    
    // 绘制连接线
    for (size_t i = 1; i < data_size; ++i) {
        int x1 = margin_left + (chart_width * (i - 1)) / max_points_;
        int x2 = margin_left + (chart_width * i) / max_points_;
        
        int y1 = valueToPixel(data[i - 1], chart_height, margin_top);
        int y2 = valueToPixel(data[i], chart_height, margin_top);
        
        cv::line(image, cv::Point(x1, y1), cv::Point(x2, y2), colors_[channel], 2);
    }
}

int DataVisualizer::valueToPixel(float value, int chart_height, int chart_top) {
    // 将数据值映射到像素坐标
    float normalized = (y_max_ - value) / (y_max_ - y_min_);
    return chart_top + static_cast<int>(normalized * chart_height);
} 