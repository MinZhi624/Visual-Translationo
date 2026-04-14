#include "ScatterPlot.hpp"
#include <opencv2/imgproc.hpp>

ScatterPlot::ScatterPlot()
{
	plot_ = cv::Mat(400, 600, CV_8UC3, cv::Scalar(0, 0, 0));
}
ScatterPlot::ScatterPlot(cv::Mat& plot)
{
    plot_ = plot;
}

void ScatterPlot::drawPlot(ScatterPlotData* data, cv::Scalar color)
{
    std::deque<float>target_speeds = data->getData();
    drawPlot(target_speeds, color);
}
void ScatterPlot::drawPlot(std::deque<float>& data, cv::Scalar color)
{
    int numPoints = data.size();
    for (int i = 0; i < numPoints; i++) {
        //映射到坐标轴位置
        int x = PLOT_MARGIN + (i * (plot_.cols - PLOT_MARGIN *2)) / MAX_POINTS;
        int y = (plot_.rows / 2) - (int)(data[i] * 15);
        // 绘制散点
        cv::circle(plot_, cv::Point(x, y), 2, color, cv::FILLED);
    }
}
void ScatterPlot::drawXAxis() { cv::line(plot_, cv::Point(PLOT_MARGIN, (plot_.rows / 2)), cv::Point(2000, (plot_.rows / 2)), cv::Scalar(255, 255, 255), 2); }
void ScatterPlot::clearPlot() { plot_.setTo(cv::Scalar(0, 0, 0)); }