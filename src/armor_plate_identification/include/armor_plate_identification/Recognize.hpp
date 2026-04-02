#pragma once
#include <opencv2/core.hpp>

/// @brief 查找目标颜色，对图像进行颜色分割
/// @param img 待识别的图像
/// @return 目标颜色的图像
cv::Mat findTargetColor(cv::Mat& img);

/// @brief 图像预处理
/// @param img 颜色查找好后的图像
/// @return 得到预处理后的图像（二值化）
cv::Mat preProcessing(cv::Mat& img);