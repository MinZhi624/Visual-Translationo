#pragma once
#include "ScatterPlotData.hpp"

const int PLOT_MARGIN = 50;

class ScatterPlot
{
	friend ScatterPlotData;
private:
	cv::Mat plot_;
public:
	ScatterPlot();
	ScatterPlot(cv::Mat& plot);
	/// <summary>
	/// 画出散点
	/// </summary>
	/// <param name="data">散点数据</param>
	/// <param name="color">绘制颜色</param>
	void drawPlot(ScatterPlotData* data, cv::Scalar color);
	void drawPlot(std::deque<float>& data, cv::Scalar color);
	/// <summary>
	/// 绘制X轴 （y半的地方）
	/// </summary>
	void drawXAxis();

	/// <summary>
	/// 清除画板
	/// </summary>
	void clearPlot();

	cv::Mat& getPlot() { return plot_; }
};