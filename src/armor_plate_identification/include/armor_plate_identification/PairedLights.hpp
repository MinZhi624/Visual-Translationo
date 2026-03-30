#pragma once
#include <opencv2/core.hpp>

/// <summary>
/// 灯条匹配管理类
/// </summary>
class PairedLights
{
private:

std::vector<std::vector<cv::Point2f>> paired_light_points_;

public:
int num_lights_ = 0;
// ==调参列表== //
// 灯条匹配参
float MIN_LENGTH_RATIO = 0.5f;
float MIN_PARALLEL = 0.5f;
float MIN_X_DIFF_RATIO = 0.3f;
float MAX_Y_DIFF_RATIO = 0.3f;
// 这里是 长边：小边
float MAX_DISTANCE_RATIO = 5.0f;
float MIN_DISTANCE_RATIO = 1.0f;
// Contuors筛选参数
int MIN_CONTOURS_AREA = 100;
float MIN_CONTOURS_RATIO = 0.1f;
float MAX_CONTOURS_RATIO = 0.5f;
// =========== //

	// ========================== //
	/// <summary>
	/// 找到灯条轮廓
	/// </summary>
	/// <param name="img_thre">二值化处理后的图像</param>
	/// <returns>灯条轮廓</returns>
	std::vector<std::vector<cv::Point>> findLightsContours(cv::Mat& img_thre);
	/// <summary>
	/// 将灯条拟合成一条线（一条线的两个端点）
	/// </summary>
	/// <param name="contours">灯条轮廓</param>
	/// <returns></returns>
	std::vector<std::vector<cv::Point2f>> findLightLines(std::vector<std::vector<cv::Point>>& contours);
	/// <summary>
	/// 检测灯条是否匹配
	/// </summary>
	/// <param name="p1">第一个点</param>
	/// <param name="p2">第二个点</param>
	/// <param name="p3">第三个点</param>
	/// <param name="p4">第四个点</param>
	/// <returns>是否匹配</returns>
	bool checkPairLights(cv::Point2f p1, cv::Point2f p2, cv::Point2f p3, cv::Point2f p4);
	/// <summary>
	/// 匹配灯条后，用外接旋转矩形框住
	/// </summary>
	/// <param name="all_lights">所有拟合直线后的灯条</param>
	/// <returns>所有匹配好的灯条</returns>
	std::vector<std::vector<cv::Point2f>> matchLights(std::vector<std::vector<cv::Point2f>>& all_lights);
	
	// ================================= //
	

	/// <summary>
	/// 找到匹配的灯条
	/// </summary>
	/// <param name="img_thre">预处理后的图片</param>
	void findPairedLights(cv::Mat& img_thre);

	/// <summary>
	/// 画出所有的匹配好的灯条
	/// </summary>
	/// <param name="img">目标图像</param>
	void drawPairedLights(cv::Mat& img);
	
	std::vector<std::vector<cv::Point2f>> getPairedLightPoints() const { return paired_light_points_; }
};

//工具

/// <summary>
/// 给直线的点按y排序，这里是引用
/// </summary>
/// <param name="lines">直线的连两个端点</param>
/// <returns>整理后的两个端点</returns>
std::vector<cv::Point2f> sortRotatedRectPoints(cv::RotatedRect& rect);
void drawQuadrangle(cv::Mat& img, cv::Point2f p1, cv::Point2f p2, cv::Point2f p3, cv::Point2f p4, cv::Scalar color);

/// <summary>
/// 判断两个直线的平行程度, p1 - p2连成的直线, p3 - p4连成的直线
/// </summary>
/// <param name="p1">直线1点1</param>
/// <param name="p2">直线1点2</param>
/// <param name="p3">直线2点1</param>
/// <param name="p4">直线2点2</param>
/// <returns>平行程度[0,1]</returns>
double getParallelDegree(cv::Point2f p1, cv::Point2f p2, cv::Point2f p3, cv::Point2f p4);