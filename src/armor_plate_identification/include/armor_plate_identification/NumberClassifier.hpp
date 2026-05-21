#pragma once
#include "armor_plate_identification/Armor.hpp"

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

    void classify(Armor& armor);
    bool checkArmorName(const Armor& armor) const;

    static bool checkArmorType(const Armor& armor);
    static cv::Mat getNumberROI(const cv::Mat& img_bgr, const Armor& armor);
};
