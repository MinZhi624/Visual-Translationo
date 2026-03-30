#pragma once
#include <opencv2/core.hpp>
#include <string>
#include <vector>

/// <summary>
/// 将多个图像拼接显示在一个窗口中（2x2 布局）
/// </summary>
/// <param name="window_name">窗口名称</param>
/// <param name="images">图像向量，支持 3 或 4 张图</param>
/// <param name="labels">标签文本（可选）</param>
void showMultiImages(const std::string& window_name, 
                     const std::vector<cv::Mat>& images,
                     const std::vector<std::string>& labels = {});
