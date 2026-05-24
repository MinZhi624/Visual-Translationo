#pragma once
#include "armor_plate_identification/DetectorArmor.hpp"

#include <opencv2/dnn.hpp>

class NumberClassifier
{
private:
    std::string config_path_;
    cv::dnn::Net net_;
public:
    float threshold_;
    NumberClassifier() = default;
    NumberClassifier(const std::string & config_path, float threshold);

    void classify(DetectorArmor& armor);
    bool checkArmorName(const DetectorArmor& armor) const;

    static bool checkArmorType(const DetectorArmor& armor);
    static cv::Mat getNumberROI(const cv::Mat& img_bgr, const DetectorArmor& armor);
};
