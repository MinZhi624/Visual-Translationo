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
	int id_;

	Lights();
};

class PairedLights
{
private:

std::vector<Lights> find_lights_;
std::vector<std::array<Lights, 2>> paired_lights_;
std::vector<std::vector<cv::Point2f>> paired_lights_points_;

/// @brief 初始化（重置）参数
void init();

public:
	int num_lights_ = 0;
	// ==调参列表== //
	// Contuors筛选参数
	int MIN_CONTOURS_AREA = 100;
	float MIN_CONTOURS_RATIO = 0.06f;
	float MAX_CONTOURS_RATIO = 0.5;
	// 灯条匹配参
	float MAX_ANGLE_DIFF = 5.0f; // 角度差这里不是比值！ 
	float MIN_LENGTH_RATIO = 0.5f;
	float MIN_X_DIFF_RATIO = 0.3f;
	float MAX_Y_DIFF_RATIO = 5.0f;
	float MAX_DISTANCE_RATIO = 1.0f;
	float MIN_DISTANCE_RATIO = 0.2f;
	// =========== //

	// ================ //
	
	/// @brief 找到所有灯条的轮廓，进行的简单的筛选--面积还有长宽比
	/// @param img_thre 预处理后二值化图像
	/// @return 所有灯条的轮廓
	std::vector<std::vector<cv::Point>> findLightsContours(cv::Mat& img_thre);
	
	/// @brief 将灯条拟合成直线
	/// @param contours 所有灯条的轮廓
	/// @return 所有灯条的信息
	std::vector<Lights> findLightLines(std::vector<std::vector<cv::Point>>& contours);
	
	/// @brief 查询所有灯条，通过简单的几何条件约束得到并进行初次匹配结果
	/// @param light_left 左灯条
	/// @param light_right 右灯条
	/// @return 是否匹配成功
	bool checkPairLights(const Lights& light_left, const Lights& light_right);
	
	/// @brief 计算两个灯条的匹配得分
	/// @param light_left 左灯条
	/// @param light_right 右灯条
	/// @return 匹配得分
	float computePairScore(const Lights& light_left, const Lights& light_right);
	
	/// @brief 得到所有灯条的匹配结果，匹配的主函数
	/// @param all_lights 所有灯条的信息
	/// @return 匹配好的灯条对
	std::vector<std::array<Lights, 2>> matchLights(std::vector<Lights>& all_lights);
	// ================= //
	
	/// @brief 这个灯条匹配类的主函数。找到并匹配好灯条
	/// @param img_thre 预处理后二值化图像
	void findPairedLights(cv::Mat& img_thre);
	
	/// @brief 画出所有匹配好的灯条对
	/// @param img 要绘制的图像
	void drawPairedLights(cv::Mat& img);

	/// @brief 得到所有匹配好的灯条按照顺时针的顺序排列
	/// @return 所有匹配好的灯条按照顺时针的顺序排列的四个点
	std::vector<std::vector<cv::Point2f>> getPairedLightPoints() const {return paired_lights_points_;}

///////////////////////// debug ////////////////////////////////////
	/// @brief 测试用的，检查所有的灯条轮廓，并标记bottom还有top
	/// @param img 要绘制的图像
	void drawAllLights(cv::Mat& img);
};

/// @brief 画出一个旋转矩形
/// @param img 要绘制的图像
/// @param rect 要画的旋转矩形
/// @param color 颜色(默认是蒂芙尼蓝RGB (129，216，207))
/// @param thickness 线条粗细（默认是2）
void drawRotatedRect(cv::Mat& img, const cv::RotatedRect& rect, const cv::Scalar& color = cv::Scalar(207, 216, 129), int thickness = 2);

/// @brief 四个顺时针的点画一个旋转矩形
/// @param img 要绘制的图像
/// @param p1 左上角点 
/// @param p2 右上角点 
/// @param p3 右下角点 
/// @param p4 左下角点 
/// @param color 颜色(默认是蒂芙尼蓝RGB (129，216，207))
/// @param thickness 线条粗细（默认是2）
void drawRotatedRect(cv::Mat& img, const cv::Point2f& p1, const cv::Point2f& p2, const cv::Point2f& p3, const cv::Point2f& p4, const cv::Scalar& color = cv::Scalar(207, 216, 129), int thickness = 2);