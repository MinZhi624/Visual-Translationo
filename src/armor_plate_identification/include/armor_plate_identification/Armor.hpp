#pragma once
#include <opencv2/core.hpp>
#include <builtin_interfaces/msg/time.hpp>

/// @brief 灯条类
class Lights
{
public:
	// 基本信息
	cv::Point2f center_;
	cv::RotatedRect rect_;
	cv::Point2f top_;
	cv::Point2f bottom_;
	double angle_;
	double length_;
	double width_;
	int area_;
	// 匹配信息
	bool is_paired_;
	int id_;
	Lights() = default;
};

class Armor
{
public:
	std::array<Lights, 2> paired_lights_; // 按x轴从左到右排列的两个灯条
	std::vector<cv::Point2f> points_; // 按照顺时针顺序排列的四个点
    cv::Mat number_roi_;
	std::string number_;
	float confidence_ = 0.0f;
	Armor() = default;
};

struct ImageSave{
	builtin_interfaces::msg::Time img_stamp;
	cv::Mat img;
};