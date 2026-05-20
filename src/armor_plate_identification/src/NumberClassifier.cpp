#include "armor_plate_identification/NumberClassifier.hpp"

#include <opencv2/dnn/dnn.hpp>
#include <opencv2/imgproc.hpp>
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
    // 检查数字是否合法
    return armor.name_ != ArmorName::NONE;
    // 检查置信度是否足够高
    return armor.confidence_ > threshold_;
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
