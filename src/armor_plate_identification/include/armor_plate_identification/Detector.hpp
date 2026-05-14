#pragma once
#include "armor_plate_identification/Armor.hpp"
#include <opencv2/core/types.hpp>
#include <vector>

/// @brief 预处理调试图像数据
struct PreprocessDebug {
	cv::Mat blue_dim_thre;   // BLUE 通道二值图
	cv::Mat gray_thre;       // GRAY 通道二值图
	cv::Mat merged_thre;     // 合并后二值图
	std::vector<std::pair<cv::Rect, int>> fragment_info;  // 每个 BLUE 区域的碎片数
};

class Detector
{
private:
	std::vector<Lights> find_lights_;
	std::vector<Armor> armors_;
	std::vector<cv::RotatedRect> color_rejected_rects_;
	// 数字图像提取数据
	const int WARP_HEIGHT = 28;
	const int WARP_WIDTH = 32;
	const int LIGHT_HEIGHT = 10;
	const int TOP_LIGHT_Y = (WARP_HEIGHT - LIGHT_HEIGHT) / 2 - 1;
	const int BOTTOM_LIGHT_Y = TOP_LIGHT_Y + LIGHT_HEIGHT;
	const cv::Size ROI_SIZE = cv::Size(20, 28);
	// 四个点安装顺时针的顺序，从左上角开始
	const std::vector<cv::Point2f> NUMBER_TARGET_POINTS = {
		cv::Point2f(0, TOP_LIGHT_Y),
		cv::Point2f(WARP_WIDTH - 1, TOP_LIGHT_Y),
		cv::Point2f(WARP_WIDTH - 1, BOTTOM_LIGHT_Y),
		cv::Point2f(0, BOTTOM_LIGHT_Y)
	};
	/// @brief 初始化（重置）参数
	void init();
	// ================ //
	/// @brief 找到所有灯条的轮廓，进行的简单的筛选--面积还有长宽比
	/// @param img_thre 预处理后二值化图像
	/// @param img_bgr 原始图像
	/// @return 所有灯条的轮廓
	std::vector<std::vector<cv::Point>> findLightsContours(cv::Mat& img_thre, const cv::Mat& img_bgr);
	
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
	std::vector<Armor> matchLights(std::vector<Lights>& all_lights);

	/// @brief 得到装甲板的号码ROI
	/// @param img_bgr 原始图像
	/// @param armor 一个装甲板的信息
	/// @return 号码ROI图像
	cv::Mat getNumberROI(const cv::Mat& img_bgr, const Armor& armor);

	/// @brief 判断一个灯条的颜色是否符合要求
	/// @param img_bgr 原始图像
	/// @param rect 灯条的最小外接矩形
	/// @param contour 灯条的轮廓
	/// @return 是否符合要求
	bool TargetColorDectect(const cv::Mat& img_bgr,const cv::RotatedRect& rect, const std::vector<cv::Point>& contour);
	
	bool checkLightGeometry(const std::vector<cv::Point>& contour);
	// =======DEBUG========== //
	std::vector<cv::Mat> number_origin_rois_;
	int outputPictureCounts_ = 0;
	std::string outputPath_ = "./picture";
	bool is_star_save_ = false;
public:
	int num_lights_ = 0;
	// ==调参列表== //
	// Contuors筛选参数
	int MIN_CONTOURS_AREA = 30;
	float MIN_CONTOURS_RATIO = 0.06f;
	float MAX_CONTOURS_RATIO = 0.5;
	// 灯条匹配参
	float MAX_ANGLE_DIFF = 5.0f; // 角度差这里不是比值！ 
	float MIN_LENGTH_RATIO = 0.5f;
	float MIN_X_DIFF_RATIO = 0.3f;
	float MAX_Y_DIFF_RATIO = 5.0f;
	float MAX_DISTANCE_RATIO = 1.0f;
	float MIN_DISTANCE_RATIO = 0.2f;
	std::string TARGET_COLOR = "BLUE"; // "RED" 或 "BLUE"
	// 预处理阈值
	int GRAY_THRESHOLD = 100;
	int COLOR_THRESHOLD = 100;

	// =========== //

	/// @brief 双通道预处理：BLUE_dim 定位 + GRAY 精确轮廓，合并为二值图
	/// @param img_bgr 原始 BGR 图像
	/// @param debug_out [可选] 输出调试图像数据，传 nullptr 则不填充
	/// @return 合并后的二值图
	cv::Mat preprocess(const cv::Mat& img_bgr, PreprocessDebug* debug_out = nullptr);
	
	/// @brief 这个灯条匹配类的主函数。找到并匹配好灯条
	/// @param img_thre 预处理后二值化图像
	/// @param img_bgr 原始图像
	void detectArmors(cv::Mat& img_thre, const cv::Mat& img_bgr);
	
	/// @brief 画出所有匹配好的灯条对
	/// @param img 要绘制的图像
	void drawArmors(cv::Mat& img);

	/// @brief 画出颜色验证未通过的轮廓（黄色框）
	/// @param img 要绘制的图像
	void drawColorRejected(cv::Mat& img);

	/// @brief 得到所有匹配好的灯条按照顺时针的顺序排列
	/// @return 所有匹配好的灯条按照顺时针的顺序排列的四个点
	const std::vector<Armor>& getArmors() const { return armors_; }

///////////////////////// debug ////////////////////////////////////
	/// @brief 测试用的，检查所有的灯条轮廓，并标记bottom还有top
	/// @param img 要绘制的图像
	void drawAllLights(cv::Mat& img);

	/// @brief 测试用的，显示原始图像所有装甲板号码的ROI
	void showNumberROI(); 

	/// @brief 测试用的，显示所有二值化装甲板号码ROI
	void showNumberBinaryROI();

	/// @brief 设置是否保存图片
	/// @param is_star_save 是否保存图片
	void setSave(bool is_star_save) { is_star_save_ = is_star_save; }

	void saveNumberRoi();
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