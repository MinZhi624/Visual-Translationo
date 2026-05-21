#include "armor_plate_identification/NumberClassifier.hpp"

#include <opencv2/dnn/dnn.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include "armor_plate_identification/Armor.hpp"

NumberClassifier::NumberClassifier(
    const std::string& config_path,
    float threshold
) : config_path_(config_path), threshold_(threshold)
{
    net_ = cv::dnn::readNetFromONNX(config_path + "/model/number_cnn.onnx");
}

bool NumberClassifier::checkArmorName(const Armor& armor) const
{
    return armor.name_ != ArmorName::NONE && armor.confidence_ > threshold_;
}

bool NumberClassifier::checkArmorType(const Armor& armor)
{
    // 检查数字是否对应装甲板
    if (armor.type_ == ArmorType::LARGE) {
        // 大装甲板只有 一 这个数字
        return armor.name_ == ArmorName::ONE;
    } else {
        // 小装甲板除了1
        return armor.name_ != ArmorName::ONE;
    }
}

void NumberClassifier::classify(Armor& armor)
{
    if (armor.number_roi_.empty()) {
        armor.confidence_ = 0.0;
        armor.name_ = ArmorName::NONE;
        return;
    }
    cv::Mat image = armor.number_roi_;
    cv::Mat blob;
    cv::dnn::blobFromImage(image, blob, 1.0 / 255.0, cv::Size(20, 28), 0, false, false);
    net_.setInput(blob);
    cv::Mat outputs = net_.forward();
    float max_prob = *std::max_element(outputs.begin<float>(), outputs.end<float>());
    cv::Mat softmax_prob;
    cv::exp(outputs - max_prob, softmax_prob);
    float sum = static_cast<float>(cv::sum(softmax_prob)[0]);
    softmax_prob /= sum;
    double confidence;
    cv::Point class_id_point;
    cv::minMaxLoc(softmax_prob.reshape(1, 1), nullptr, &confidence, nullptr, &class_id_point);
    int label_id = class_id_point.x;
    armor.confidence_ = static_cast<float>(confidence);
    armor.name_ = intToArmorName(label_id);
}

cv::Mat NumberClassifier::getNumberROI(const cv::Mat& img_bgr, const Armor& armor)
{
    static const int WARP_HEIGHT = 28;

    static const int LIGHT_HEIGHT = 12;
    static const int SMALL_ARMOR_WIDTH = 32;
    static const int LARGE_ARMOR_WIDTH = 54;

    static const cv::Size ROI_SIZE = cv::Size(20, 28);
    
    static const int TOP_LIGHT_Y = (WARP_HEIGHT - LIGHT_HEIGHT) / 2 - 1;
    static const int BOTTOM_LIGHT_Y = TOP_LIGHT_Y + LIGHT_HEIGHT;
    static const int WARP_WIDTH = (armor.type_ == ArmorType::LARGE) ? LARGE_ARMOR_WIDTH : SMALL_ARMOR_WIDTH;

    static const std::vector<cv::Point2f> NUMBER_TARGET_POINTS = {
        cv::Point2f(0, TOP_LIGHT_Y),
        cv::Point2f(WARP_WIDTH - 1, TOP_LIGHT_Y),
        cv::Point2f(WARP_WIDTH - 1, BOTTOM_LIGHT_Y),
        cv::Point2f(0, BOTTOM_LIGHT_Y)
    };

    // 保证是有内容的装甲板
    if (armor.points_.size() != 4) return cv::Mat();
    // 透视转换
    auto rotation_matrix = cv::getPerspectiveTransform(armor.points_, NUMBER_TARGET_POINTS);
    cv::Mat number_roi;
    cv::warpPerspective(img_bgr, number_roi, rotation_matrix, cv::Size(WARP_WIDTH, WARP_HEIGHT));
    // 数字部分ROI
    number_roi = number_roi(cv::Rect(cv::Point((WARP_WIDTH - ROI_SIZE.width) / 2, 0), ROI_SIZE));
    // 预处理：灰度化
    cv::cvtColor(number_roi, number_roi, cv::COLOR_BGR2GRAY);
    return number_roi;
}
