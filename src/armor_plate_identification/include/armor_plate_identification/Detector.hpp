#pragma once
#include "armor_plate_identification/Armor.hpp"
#include "armor_plate_identification/NumberClassifier.hpp"
#include <opencv2/core.hpp>
#include <string>
#include <vector>

/** @brief 预处理调试图像数据 */
struct PreprocessDebug {
	cv::Mat blue_dim_thre;   // BLUE 通道二值图
	cv::Mat gray_thre;       // GRAY 通道二值图
	cv::Mat merged_thre;     // 合并后二值图
	std::vector<std::pair<cv::Rect, int>> fragment_info;  // 每个 BLUE 区域的碎片数
};

// 传参结构体 //
struct LightParams {
	int min_contours_area_;
	// 短边 / 长边的比值
	float min_contours_ratio_;
	float max_contours_ratio_;
};
struct ArmorParams {
	float max_angle_diff_;
	float min_length_ratio_;
	float min_x_diff_ratio_;
	float max_y_diff_ratio_;
	float max_distance_ratio_;
	float min_distance_ratio_;
	std::string target_color_;
};

class Detector
{
private:
	std::vector<Light> find_lights_;
	std::vector<Armor> armors_;
	std::vector<Armor> rejected_armors_;
	NumberClassifier classifier_;

	// LightParams
	int min_contours_area_;
	float min_contours_ratio_;
	float max_contours_ratio_;
	// ArmorParams
	float max_angle_diff_;
	float min_length_ratio_;
	float min_x_diff_ratio_;
	float max_y_diff_ratio_;
	float max_distance_ratio_;
	float min_distance_ratio_;
	Color target_color_;
	// 预处理阈值
	int gray_threshold_;
	int color_threshold_;

	void reset();

	std::vector<Light> findLights(cv::Mat& img_thre, const cv::Mat& img_bgr);

	bool checkLightGeometry(const std::vector<cv::Point>& contour) const;
	bool checkLightColor(const Light& light_left, const Light& light_right) const;
	bool checkArmorGeometry(const Armor& armor) const;

	std::vector<Armor> matchLights(std::vector<Light>& all_lights, const cv::Mat& img_bgr);

	cv::Mat getArmorPattern(const cv::Mat& img_bgr, const Armor& armor) const;

public:
    Detector() = default;
    Detector(const std::string& config_path, float number_threshold,
             const LightParams& light_params, const ArmorParams& armor_params,
             int gray_threshold, int color_threshold);
	int num_lights_ = 0;

	cv::Mat preprocess(const cv::Mat& img_bgr, PreprocessDebug* debug_out = nullptr) const;
	void detectArmors(cv::Mat& img_thre, const cv::Mat& img_bgr);

	const std::vector<Armor>& getArmors() const { return armors_; }
	const std::vector<Light>& getLights() const { return find_lights_; }
	std::vector<cv::Mat> getRejectedNumberRois();
};
