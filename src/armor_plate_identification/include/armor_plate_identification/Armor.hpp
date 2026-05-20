#pragma once
#include <opencv2/core.hpp>
#include <builtin_interfaces/msg/time.hpp>
#include <string>

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

enum class ArmorName
{
	ONE,
	TWO,
	THREE,
	FOUR,
	FIVE,
	NONE
};

inline Color stringToColor(const std::string& s)
{
	if (s == "RED") return Color::RED;
	if (s == "BLUE") return Color::BLUE;
	return Color::NONE;
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
class Armor
{
private:
	// 匹配常量
	static constexpr float DIST_RATIO_SMALL = 2.0f;  // 小装甲板 dist_ratio 目标（140×125mm）
	static constexpr float DIST_RATIO_LARGE = 3.3f;  // 大装甲板 dist_ratio 目标（235×127mm）
	static constexpr float DIST_RATIO_THRESH = 2.5f;  // 大/小装甲板分界阈值
public:
	std::array<Light, 2> paired_lights_; // 按x轴从左到右排列的两个灯条
	std::vector<cv::Point2f> points_; // 按照顺时针顺序排列的四个点
	ArmorType type_;
	ArmorName name_;
	// 数字识别信息
    cv::Mat number_roi_;
	std::string number_;
	float confidence_ = 0.0f;
	
	// 匹配信息
	double angle_diff_;
	double length_ratio_;
	double y_diff_ratio_;
	double x_diff_ratio_;
	double distance_ratio_;
	float score_;
	
	Armor() = default;
	Armor(Light& light_left, Light& light_right, 
		// 用于计算分数
		float max_angle_diff, float max_y_diff_ratio);
};

/** @brief 图像保存结构体 */
struct ImageSave{
	builtin_interfaces::msg::Time img_stamp;
	cv::Mat img;
};
