#pragma once
#include <opencv2/core.hpp>
#include <deque>

const int MAX_POINTS = 1200;

class ScatterPlotData
{
protected:
	std::deque<float> data_;
public:
	/// <summary>
	/// 保存数据
	/// </summary>
	/// <param name="Datum">要保存的数据</param>
	void saveDatum(float Datum);
	std::deque<float> getData() const { return data_; }
};
