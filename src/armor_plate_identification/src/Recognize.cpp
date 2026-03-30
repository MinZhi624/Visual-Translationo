#include "armor_plate_identification/Recognize.hpp"
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

cv::Mat findTargetColor(cv::Mat& img)
{
	// Blue - Red
	cv::Mat channels[3];
	cv::split(img, channels);
	return channels[0] - channels[2];
}

cv::Mat preProcessing(cv::Mat& img)
{
	// 通过大津法二值化
	cv::Mat img_thre;
	cv::threshold(img, img_thre, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
	
	// 先不用
	// 3 * 3膨胀
	// cv::Mat img_dilate;
	// cv::Mat kernal = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
	// cv::dilate(img_thre, img_dilate, kernal);
	return img_thre;
}

