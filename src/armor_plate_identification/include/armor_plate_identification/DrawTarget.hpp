#pragma once
#include <opencv2/core.hpp>

/// <summary>
/// 画旋转矩形
/// </summary>
/// <param name="img">被画的图像</param>
/// <param name="target_rect">要画的旋转矩形</param>
void drawRotatedRect(cv::Mat img, cv::RotatedRect target_rect);