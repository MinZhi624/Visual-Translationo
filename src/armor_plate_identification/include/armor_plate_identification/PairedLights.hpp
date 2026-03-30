#pragma once
#include <opencv2/core.hpp>

/// @brief 灯条类
class Lights
{
public:
	cv::Point2f center_;
	cv::RotatedRect rect_;
	cv::Point2f top_;
	cv::Point2f bottom_;
	double angle_;
	double length_;
	double width_;
	int area_;
	// 暂时先不用 is_paired_
	bool is_paired_;

	Lights();
};


/// <summary>
/// 灯条匹配管理类
/// </summary>
class PairedLights
{
private:

std::vector<Lights> find_lights_;
std::vector<std::array<Lights, 2>> paired_lights_;

public:
	int num_lights_ = 0;
	// ==调参列表== //
	// Contuors筛选参数
	int MIN_CONTOURS_AREA = 100;
	float MIN_CONTOURS_RATIO = 0.1f;
	float MAX_CONTOURS_RATIO = 0.5f;
	// 灯条匹配参
	float MAX_ANGLE_DIFF = 5.0f; // 角度差这里不是比值！ 
	float MIN_LENGTH_RATIO = 0.5f;
	float MIN_X_DIFF_RATIO = 0.3f;
	float MAX_Y_DIFF_RATIO = 0.3f;
	float MAX_DISTANCE_RATIO = 1.0f;
	float MIN_DISTANCE_RATIO = 0.2f;
	// =========== //

	std::vector<std::vector<cv::Point>> findLightsContours(cv::Mat& img_thre);
	std::vector<Lights> findLightLines(std::vector<std::vector<cv::Point>>& contours);
	bool checkPairLights(const Lights& light_left, const Lights& light_right);
	std::vector<std::array<Lights, 2>> matchLights(std::vector<Lights>& all_lights);
	
	// ================================= //
	
	// 主要要调用函数
	void findPairedLights(cv::Mat& img_thre);
	// 画出匹配好的图
	void drawPairedLights(cv::Mat& img);
	
	std::vector<std::vector<cv::Point2f>> getPairedLightPoints() const;
};