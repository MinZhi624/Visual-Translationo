#pragma once
#include "armor_plate_identification/DetectorArmor.hpp"

#include <opencv2/dnn.hpp>
#include <openvino/openvino.hpp>
#include <openvino/runtime/compiled_model.hpp>

class NumberClassifier
{
private:
    std::string config_path_;
    // Opencv 
    cv::dnn::Net net_;
    // OpenVINO
    ov::Core core_;
    ov::CompiledModel compiled_model_;

    static cv::Mat softmax(const cv::Mat& logits);
    static float sigmoid(float x);
public:
    float threshold_;
    NumberClassifier() = default;
    NumberClassifier(const std::string & config_path, float threshold);

    void classifyFromOpenVino(DetectorArmor& armor);
    void classify(DetectorArmor& armor);
    
    bool checkArmorName(const DetectorArmor& armor) const;

    static bool checkArmorType(const DetectorArmor& armor);
    static cv::Mat getNumberROI(const cv::Mat& img_bgr, const DetectorArmor& armor);
};
