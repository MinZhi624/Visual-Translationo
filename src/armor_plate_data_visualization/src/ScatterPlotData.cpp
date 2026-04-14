#include "ScatterPlotData.hpp"

void ScatterPlotData::saveDatum(float datum)
{
	data_.push_back(datum);
	if (data_.size() > MAX_POINTS) {
		data_.pop_front();
	}
}