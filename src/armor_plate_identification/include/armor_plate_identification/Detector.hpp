#pragma once
#include "armor_plate_identification/Armor.hpp"
#include "armor_plate_identification/NumberClassifier.hpp"
#include <opencv2/core.hpp>
#include <vector>

/** @brief 预处理调试图像数据 */
struct PreprocessDebug {
	cv::Mat blue_dim_thre;   // BLUE 通道二值图
	cv::Mat gray_thre;       // GRAY 通道二值图
	cv::Mat merged_thre;     // 合并后二值图
	std::vector<std::pair<cv::Rect, int>> fragment_info;  // 每个 BLUE 区域的碎片数
};

class Detector
{
private:
	std::vector<Light> find_lights_;
	std::vector<Armor> armors_;
	NumberClassifier classifier_;
	// 数字图像提取数据
	static constexpr int WARP_HEIGHT = 28;
	static constexpr int WARP_WIDTH = 32;
	static constexpr int LIGHT_HEIGHT = 10;
	static constexpr int TOP_LIGHT_Y = (WARP_HEIGHT - LIGHT_HEIGHT) / 2 - 1;
	static constexpr int BOTTOM_LIGHT_Y = TOP_LIGHT_Y + LIGHT_HEIGHT;

	static constexpr int MIN_CONTOURS_AREA = 30;
	static constexpr float MIN_CONTOURS_RATIO = 0.06f;
	static constexpr float MAX_CONTOURS_RATIO = 0.5;
	cv::Size ROI_SIZE = cv::Size(20, 28);
	// 四个点安装顺时针的顺序，从左上角开始
	std::vector<cv::Point2f> NUMBER_TARGET_POINTS = {
		cv::Point2f(0, TOP_LIGHT_Y),
		cv::Point2f(WARP_WIDTH - 1, TOP_LIGHT_Y),
		cv::Point2f(WARP_WIDTH - 1, BOTTOM_LIGHT_Y),
		cv::Point2f(0, BOTTOM_LIGHT_Y)
	};

	void reset();

	std::vector<Light> findLights(cv::Mat& img_thre, const cv::Mat& img_bgr);
	
	bool checkLightGeometry(const std::vector<cv::Point>& contour) const;
	bool checkLightColor(const Light& light_left, const Light& light_right) const;
	bool checkArmorGeometry(const Armor& armor) const;
	
	std::vector<Armor> matchLights(std::vector<Light>& all_lights, const cv::Mat& img_bgr);

	cv::Mat getNumberROI(const cv::Mat& img_bgr, const Armor& armor);


public:
    Detector() = default;
    Detector(const std::string& model_path, float threshold);
	int num_lights_ = 0;
	// ==调参列表== //
	// Contuors筛选参数
	
	// 灯条匹配参
	float MAX_ANGLE_DIFF = 5.0f; // 角度差这里不是比值！
	float MIN_LENGTH_RATIO = 0.5f;
	float MIN_X_DIFF_RATIO = 0.3f;
	float MAX_Y_DIFF_RATIO = 5.0f;
	float MAX_DISTANCE_RATIO = 1.0f;
	float MIN_DISTANCE_RATIO = 0.2f;
	Color target_color_ = Color::BLUE;
	// 预处理阈值
	int GRAY_THRESHOLD = 100;
	int COLOR_THRESHOLD = 100;

	cv::Mat preprocess(const cv::Mat& img_bgr, PreprocessDebug* debug_out = nullptr) const;
	void detectArmors(cv::Mat& img_thre, const cv::Mat& img_bgr);

	const std::vector<Armor>& getArmors() const { return armors_; }
	std::vector<cv::Mat> getNumberRois();
	cv::Mat getNumberROI_AABB(const cv::Mat& img_bgr, const Armor& armor) const;
	void drawArmors(cv::Mat& img);
	void drawAllLights(cv::Mat& img);
};

void drawRotatedRect(cv::Mat& img, const cv::RotatedRect& rect, const cv::Scalar& color = cv::Scalar(207, 216, 129), int thickness = 2);
void drawRotatedRect(cv::Mat& img, const cv::Point2f& p1, const cv::Point2f& p2, const cv::Point2f& p3, const cv::Point2f& p4, const cv::Scalar& color = cv::Scalar(207, 216, 129), int thickness = 2);
