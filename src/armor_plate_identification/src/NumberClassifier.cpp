#include "armor_plate_identification/DetectorArmor.hpp"
#include "armor_plate_identification/NumberClassifier.hpp"

#include <opencv2/dnn/dnn.hpp>
#include <opencv2/imgcodecs.hpp>

#include <opencv2/imgproc.hpp>
#include <openvino/runtime/infer_request.hpp>
#include <openvino/runtime/properties.hpp>

NumberClassifier::NumberClassifier(
    const std::string& config_path,
    float threshold
) : config_path_(config_path), threshold_(threshold)
{
    // Opencv DNN
    net_ = cv::dnn::readNetFromONNX(config_path + "/model/number_cnn.onnx");
    // OpenVINO 
    auto model = core_.read_model(config_path + "/model/number_cnn.onnx");
    compiled_model_ = core_.compile_model(model, "AUTO", ov::hint::performance_mode(ov::hint::PerformanceMode::LATENCY));
}

bool NumberClassifier::checkArmorName(const DetectorArmor& armor) const
{
    return armor.name_ != ArmorName::NONE && armor.confidence_ > threshold_;
}

bool NumberClassifier::checkArmorType(const DetectorArmor& armor)
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

cv::Mat NumberClassifier::softmax(const cv::Mat& logits)
{
    float max_prob = *std::max_element(logits.begin<float>(), logits.end<float>());
    cv::Mat prob;
    cv::exp(logits - max_prob, prob);
    float sum = static_cast<float>(cv::sum(prob)[0]);
    prob /= sum;
    return prob;
}

float NumberClassifier::sigmoid(float x)
{
    return 1.0f / (1.0f + std::exp(-x));
}

void NumberClassifier::classify(DetectorArmor& armor)
{
    if (armor.number_roi_.empty()) {
        armor.confidence_ = 0.0;
        armor.name_ = ArmorName::NONE;
        return;
    }
    cv::Mat image = armor.number_roi_;
    cv::Mat blob;
    cv::dnn::blobFromImage(image, blob, 1.0 / 255.0, cv::Size(28, 28), 0, false, false);
    net_.setInput(blob);
    cv::Mat outputs = net_.forward();  // shape: (1, 11) -> [obj, cls0..cls9]

    // 分离 obj 和 cls
    float obj_logit = outputs.at<float>(0, 0);
    cv::Mat cls_logits = outputs.colRange(1, 11);  // shape: (1, 10)

    // 对 cls 部分单独做 softmax
    cv::Mat softmax_prob = softmax(cls_logits);

    // 10 类 → 6 类：合并 0+6~9 为 NONE，1~5 不变
    const float* p = softmax_prob.ptr<float>();
    float probs[6] = {
        p[0] + p[6] + p[7] + p[8] + p[9],  // NONE
        p[1],                                // ONE
        p[2],                                // TWO
        p[3],                                // THREE
        p[4],                                // FOUR
        p[5]                                 // FIVE
    };
    int label_id = static_cast<int>(std::max_element(probs, probs + 6) - probs);

    // 综合置信度 = sigmoid(obj) * 类别概率
    armor.confidence_ = sigmoid(obj_logit) * probs[label_id];
    armor.name_ = intToArmorName(label_id);
}
void NumberClassifier::classifyFromOpenVino(DetectorArmor& armor)
{
    if(armor.number_roi_.empty()) {
        armor.confidence_ = 0.0;
        armor.name_ = ArmorName::NONE;
        return;
    }
    cv::Mat image = armor.number_roi_;
    cv::Mat blob;
    cv::dnn::blobFromImage(image, blob, 1.0 / 255.0, cv::Size(28, 28), 0, false, false);

    ov::InferRequest infer_request = compiled_model_.create_infer_request();

    ov::Tensor input_tensor(
        ov::element::f32,
        {1, 1, 28, 28},
        blob.ptr<float>()
    );

    infer_request.set_input_tensor(input_tensor);
    infer_request.infer();

    auto output_tensor = infer_request.get_output_tensor();

    float *output_data = output_tensor.data<float>();
    cv::Mat cls_logits = cv::Mat(1, 10, CV_32F, const_cast<float*>(output_data + 1));
    cv::Mat softmax_prob = softmax(cls_logits);
    const float* p = softmax_prob.ptr<float>();
    float probs[6] = {
        p[0] + p[6] + p[7] + p[8] + p[9],  // NONE
        p[1],                                // ONE
        p[2],                                // TWO
        p[3],                                // THREE
        p[4],                                // FOUR
        p[5]                                 // FIVE
    };
    int label_id = static_cast<int>(std::max_element(probs, probs + 6) - probs);

    // 综合置信度 = sigmoid(obj) * 类别概率
    armor.confidence_ = sigmoid(output_data[0]) * probs[label_id];
    armor.name_ = intToArmorName(label_id);
}


cv::Mat NumberClassifier::getNumberROI(const cv::Mat& img_bgr, const DetectorArmor& armor)
{
    const std::vector<cv::Point2f> dst_pts = {
        {0.f, 7.f}, {0.f, 21.f}, {28.f, 21.f}, {28.f, 7.f}
    };
    std::vector<cv::Point2f> src_pts = {
        armor.paired_lights_.front().top_,
        armor.paired_lights_.front().bottom_,
        armor.paired_lights_.back().bottom_,
        armor.paired_lights_.back().top_
    };
    auto M = cv::getPerspectiveTransform(src_pts, dst_pts);
    cv::Mat roi;
    cv::warpPerspective(img_bgr, roi, M, cv::Size(28, 28));
    cv::cvtColor(roi, roi, cv::COLOR_BGR2GRAY);
    return roi;
}
