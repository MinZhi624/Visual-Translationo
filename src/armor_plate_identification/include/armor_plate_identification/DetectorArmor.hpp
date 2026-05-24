#pragma once
#include <opencv2/core.hpp>
#include <builtin_interfaces/msg/time.hpp>
#include <Eigen/Geometry>
#include "Eigen/src/Core/Matrix.h"
#include <vector>

/** @brief 预处理调试图像数据 */
struct PreprocessDebug {
    cv::Mat blue_dim_thre;   // BLUE/RED 通道二值图
    cv::Mat gray_thre;       // GRAY 通道二值图
    cv::Mat merged_thre;     // 合并后二值图
    std::vector<std::pair<cv::Rect, int>> fragment_info;  // 每个 color 区域的碎片数
};

/*  enum class
	这里采用enum class，这是个更现代的方法：
	1. 作用域
	2. 安全，防止隐式转换
*/

enum class ArmorType
{
	SMALL,
	LARGE
};

enum class Color
{
	RED,
	BLUE,
	NONE
};

enum class ArmorName : int
{
	ONE = 1,
	TWO = 2,
	THREE = 3,
	FOUR = 4,
	FIVE = 5,
	NONE = 0
};

inline Color stringToColor(const std::string& s)
{
	if (s == "RED") return Color::RED;
	if (s == "BLUE") return Color::BLUE;
	return Color::NONE;
}
// 临时映射：label_id → ArmorName（按 label_cnn.txt 顺序）
inline ArmorName intToArmorName(int id)
{
	switch (id) {
		case 0: return ArmorName::THREE;
		case 1: return ArmorName::FOUR;
		default: return ArmorName::NONE;
	}
}
inline std::string armorNameToString(const ArmorName& name)
{
	switch (name) {
		case ArmorName::ONE: return "1";
		case ArmorName::TWO: return "2";
		case ArmorName::THREE: return "3";
		case ArmorName::FOUR: return "4";
		case ArmorName::FIVE: return "5";
		default: return "NONE";
	}
}


/** @brief 灯条类 */
class Light
{
public:
	// 基本信息
	cv::RotatedRect rect_; // ellipse_rect 用于画图
	cv::Point2f center_;
	cv::Point2f top_;
	cv::Point2f bottom_;
	double angle_;
	double length_;
	double width_;
	int area_;
	int id_ = -1;
	Color color_;

	Light() = default;
	Light(cv::RotatedRect ellipse_rect, cv::RotatedRect min_rect, Color color);

static Color getLightColor(const cv::Mat& img_bgr,const cv::RotatedRect& rect, const std::vector<cv::Point>& contour);
};

/** @brief 装甲板类 */
class DetectorArmor
{
private:
	// 匹配常量
	static constexpr float DIST_RATIO_THRESH = 2.8f;  // 大/小装甲板分界阈值
public:
	// 基本信息
	std::array<Light, 2> paired_lights_; // 按x轴从左到右排列的两个灯条
	std::vector<cv::Point2f> points_; // 按照顺时针顺序排列的四个点
	Eigen::Vector3d xyz_camera_; 	// 相机坐标系下的装甲板位置
	Eigen::Quaterniond q_camera_; // 相机坐标系下的装甲板姿态
	float image_distance_to_center_;
	// 数字识别信息
	ArmorType type_;
	ArmorName name_;
    cv::Mat number_roi_; //数字识别
    cv::Mat pattern_;  // 去重
	float confidence_;

	// 匹配信息
	double angle_diff_;
	double length_ratio_;
	double y_diff_ratio_;
	double x_diff_ratio_;
	double distance_ratio_;

	DetectorArmor() = default;
	DetectorArmor(Light& light_left, Light& light_right);
};

/** @brief 图像保存结构体 */
struct ImageSave{
	builtin_interfaces::msg::Time img_stamp;
	cv::Mat img;
};
