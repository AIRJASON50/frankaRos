#pragma once

#include <vector>
#include <deque>
#include <string>
#include <mutex>
#include <opencv2/opencv.hpp>

/**
 * @brief 六维力数据实时可视化类
 */
class DataVisualizer {
public:
    /**
     * @brief 构造函数
     * @param window_name 窗口名称
     * @param max_points 每个通道最大数据点数
     */
    DataVisualizer(const std::string& window_name = "六维力传感器实时数据", int max_points = 1000);
    
    /**
     * @brief 析构函数
     */
    ~DataVisualizer();

    /**
     * @brief 添加新的数据点
     * @param forces 六维力数据数组
     */
    void addDataPoint(const std::array<float, 6>& forces);

    /**
     * @brief 更新显示窗口
     * @return 继续运行返回true，用户按ESC退出返回false
     */
    bool updateDisplay();

    /**
     * @brief 清零所有数据
     */
    void resetData();

    /**
     * @brief 设置Y轴显示范围
     * @param min_val Y轴最小值
     * @param max_val Y轴最大值
     */
    void setYRange(float min_val, float max_val);

    /**
     * @brief 设置是否显示网格
     * @param show_grid 是否显示网格
     */
    void setShowGrid(bool show_grid);

private:
    std::string window_name_;
    int max_points_;
    
    // 数据存储
    std::array<std::deque<float>, 6> data_buffers_;
    std::mutex data_mutex_;
    
    // 显示参数
    cv::Size window_size_;
    float y_min_, y_max_;
    bool show_grid_;
    
    // 通道标签和颜色
    std::array<std::string, 6> labels_;
    std::array<cv::Scalar, 6> colors_;
    
    /**
     * @brief 绘制图表
     * @param image 输出图像
     */
    void drawChart(cv::Mat& image);
    
    /**
     * @brief 绘制网格
     * @param image 输出图像
     */
    void drawGrid(cv::Mat& image);
    
    /**
     * @brief 绘制坐标轴
     * @param image 输出图像
     */
    void drawAxes(cv::Mat& image);
    
    /**
     * @brief 绘制图例
     * @param image 输出图像
     */
    void drawLegend(cv::Mat& image);
    
    /**
     * @brief 绘制数据曲线
     * @param image 输出图像
     * @param channel 通道索引 (0-5)
     */
    void drawDataLine(cv::Mat& image, int channel);
    
    /**
     * @brief 将数据值转换为像素坐标
     * @param value 数据值
     * @param chart_height 图表高度
     * @param chart_top 图表顶部Y坐标
     * @return 像素Y坐标
     */
    int valueToPixel(float value, int chart_height, int chart_top);
}; 