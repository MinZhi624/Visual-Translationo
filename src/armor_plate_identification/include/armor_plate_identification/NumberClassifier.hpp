#pragma once
#include "armor_plate_identification/Armor.hpp"

#include <opencv2/dnn.hpp>

class NumberClassifier
{
private:
    cv::dnn::Net net_; // DNN 模型
    std::vector<std::string> class_names_; // 类型列表
    std::vector<std::string> ignore_classes_; // 需要忽略的类型列表
public:
    double threshold_;
    NumberClassifier() = default;
    NumberClassifier(
        const std::string& model_path,
        const std::string& label_path,
        double threshold,
        const std::vector<std::string>& ignore_classes = {}
    );

    /**
     * @brief 对装甲板进行数字识别，填充 number_ 和 confidence_ 字段，然后过滤低置信度/忽略类别
     * @param armors 待识别的装甲板列表，识别结果会直接填充并过滤此列表
     */
    void classify(std::vector<Armor>& armors);

    /**
     * @brief 检查单个装甲板是否合格（id 不在忽略列表中且置信度 >= 阈值）
     * @param armor 待检查的装甲板
     * @return true 合格，false 不合格
     */
    bool checkClassify(const Armor& armor);

};

/**
 * @brief 画出一个装甲板的数字识别结果，包括识别出的数字和置信度
 * @param img 要绘制的图像
 * @param armor 要绘制的装甲板信息
 */
void drawNumberTest(cv::Mat& img, const Armor& armor);

/**
 * @brief 画出所有装甲板的数字识别结果，包括识别出的数字和置信度
 * @param img 要绘制的图像
 * @param armors 要绘制的装甲板信息列表
 */
void drawAllNumberTest(cv::Mat& img, const std::vector<Armor>& armors);
