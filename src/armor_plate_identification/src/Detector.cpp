#include "armor_plate_identification/Detector.hpp"
#include <opencv2/core/types.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <algorithm>
#include "armor_plate_identification/NumberClassifier.hpp"
////////////////////// Detector /////////////////////////
Detector::Detector(const std::string& model_path, float threshold)
    : classifier_(model_path, threshold)
{
}

void Detector::reset()
{
    find_lights_.clear();
    armors_.clear();
    num_lights_ = 0;
}

cv::Mat Detector::preprocess(const cv::Mat& img_bgr, PreprocessDebug* debug_out) const
{
    // COLOR 阈值
    std::vector<cv::Mat> bgr;
    cv::split(img_bgr, bgr);
    cv::Mat color_ch = (target_color_ == Color::BLUE) ? bgr[0] : bgr[2];
    cv::Mat target_color_dim_thre;
    cv::threshold(color_ch, target_color_dim_thre, COLOR_THRESHOLD, 255, cv::THRESH_BINARY);

    // GRAY 阈值
    cv::Mat gray;
    cv::cvtColor(img_bgr, gray, cv::COLOR_BGR2GRAY);
    cv::Mat gray_thre;
    cv::threshold(gray, gray_thre, GRAY_THRESHOLD, 255, cv::THRESH_BINARY);

    // 对每个 color_dim 区域，检查 GRAY 碎片数，构建合并二值图
    std::vector<std::vector<cv::Point>> target_color_contours;
    cv::findContours(target_color_dim_thre, target_color_contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    // 过滤面积太小或点数不足的 BLUE 区域
    target_color_contours.erase(
        std::remove_if(target_color_contours.begin(), target_color_contours.end(),
            [](const auto& c) { return c.size() < 6 || cv::contourArea(c) < MIN_CONTOURS_AREA; }),
        target_color_contours.end()
    );
    std::vector<std::pair<cv::Rect, int>> fragment_info;

    // 以空白为底图，GRAY 碎片==1 用 GRAY，>=3 为噪音跳过，其余用 BLUE_dim
    cv::Mat img_thre = cv::Mat::zeros(img_bgr.size(), CV_8UC1);
    cv::Mat roi_mask;
    for (const auto& target_color_contour : target_color_contours) {
        cv::Rect color_bb = cv::boundingRect(target_color_contour);
        cv::Rect safe_bb = color_bb & cv::Rect(0, 0, gray_thre.cols, gray_thre.rows);
        cv::Mat gray_roi = gray_thre(safe_bb);

        std::vector<std::vector<cv::Point>> gray_roi_contours;
        cv::findContours(gray_roi, gray_roi_contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        int valid_count = 0;
        for (const auto& c : gray_roi_contours) {
            if (cv::contourArea(c) > 5) valid_count++;
        }

        fragment_info.push_back({safe_bb, valid_count});

        if (valid_count == 1) {
            gray_thre(safe_bb).copyTo(img_thre(safe_bb));
        } else if (valid_count >= 3) {
            // 噪音，跳过
        } else {
            // 0 or 2：用 BLUE_dim 覆盖
            roi_mask = cv::Mat::zeros(safe_bb.size(), CV_8UC1);
            std::vector<cv::Point> local_contour;
            local_contour.reserve(target_color_contour.size());
            for (const auto& pt : target_color_contour) {
                local_contour.push_back(pt - cv::Point(safe_bb.x, safe_bb.y));
            }
            cv::fillConvexPoly(roi_mask, local_contour, 255);
            target_color_dim_thre(safe_bb).copyTo(img_thre(safe_bb), roi_mask);
        }
    }
    // 闭运算
    cv::Mat kernal_dilate = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 5));
    cv::Mat kernal_erode = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 5));
    cv::dilate(img_thre, img_thre, kernal_dilate);
    cv::erode(img_thre, img_thre, kernal_erode);
    // 填充调试图像数据
    if (debug_out) {
        debug_out->blue_dim_thre = target_color_dim_thre.clone();
        debug_out->gray_thre = gray_thre.clone();
        debug_out->merged_thre = img_thre.clone();
        debug_out->fragment_info = fragment_info;
    }
    return img_thre;
}

void Detector::detectArmors(cv::Mat& img_thre, const cv::Mat& img_bgr)
{
    // 初始化（重置）参数
    reset();
    // 找到灯条并构造 Light 对象
    find_lights_ = findLights(img_thre, img_bgr);
    // 按x轴排序,从左到右
    std::sort(find_lights_.begin(), find_lights_.end(), [](const Light& a, const Light& b) {
        return a.center_.x < b.center_.x;
    });
    // 匹配灯条
    armors_ = matchLights(find_lights_, img_bgr);
    // 得到装甲板的号码ROI
    for (auto& armor : armors_) {
        armor.number_roi_ = getNumberROI(img_bgr, armor);
    }
    // 更新装甲板数量
    num_lights_ = armors_.size();
}
std::vector<Light> Detector::findLights(cv::Mat& img_thre, const cv::Mat& img_bgr)
{
    std::vector<std::vector<cv::Point>> contours;
    std::vector<Light> lights_list;
    cv::findContours(img_thre, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
    for (const auto& contour : contours) {
        if (!checkLightGeometry(contour)) continue;
        cv::RotatedRect min_rect = cv::minAreaRect(cv::Mat(contour));
        Color color = Light::getLightColor(img_bgr, min_rect, contour);
        // 构造 Light（椭圆拟合提供方向和绘制框，minAreaRect 提供尺寸约束）
        cv::RotatedRect ellipse_rect = cv::fitEllipse(contour);
        Light light(ellipse_rect, min_rect, color);
        light.id_ = static_cast<int>(lights_list.size());
        lights_list.push_back(light);
    }
    return lights_list;
}
////////// ===== CHECK 函数 ===== /////////
bool Detector::checkLightGeometry(const std::vector<cv::Point>& contour) const
{
    // 检查轮廓点的个数 
    if (contour.size() <= 6) return false;
    int area = cv::contourArea(contour);
    // 检查面积
    if (area <= MIN_CONTOURS_AREA) return false;
    // 检查长宽比
    cv::RotatedRect min_rect = cv::minAreaRect(cv::Mat(contour));
    double long_length = std::max(min_rect.size.width, min_rect.size.height);
    double short_length = std::min(min_rect.size.width, min_rect.size.height);
    double ratio = short_length / long_length;
    return ratio > MIN_CONTOURS_RATIO && ratio < MAX_CONTOURS_RATIO;
}
bool Detector::checkArmorGeometry(const Armor& armor) const
{
    // 角度差，灯条长度比
    if (armor.angle_diff_ > MAX_ANGLE_DIFF || armor.length_ratio_ < MIN_LENGTH_RATIO) return false;
    // X差比率，y差比率
    if (armor.x_diff_ratio_ < MIN_X_DIFF_RATIO || armor.y_diff_ratio_ > MAX_Y_DIFF_RATIO) return false;
    // 距离比率
    if (armor.distance_ratio_ > MAX_DISTANCE_RATIO || armor.distance_ratio_ < MIN_DISTANCE_RATIO) return false;
    return true;
}
bool Detector::checkLightColor(const Light& light_left, const Light& light_right) const
{
    return light_left.color_ == light_right.color_ && light_left.color_ == target_color_;
}
std::vector<Armor> Detector::matchLights(std::vector<Light>& all_lights, const cv::Mat& img_bgr)
{
    std::vector<Armor> candidates;
    for (size_t i = 0; i < all_lights.size(); i++) {
        for (size_t j = i + 1; j < all_lights.size(); j++) {
            if (!checkLightColor(all_lights[i], all_lights[j])) continue;
            Armor armor(all_lights[i], all_lights[j]);
            // 几何判断
            if (!checkArmorGeometry(armor)) continue;
            // 数字判读
            armor.number_roi_ = getNumberROI(img_bgr, armor);
            armor.pattern_ = getNumberROI_AABB(img_bgr, armor);
            classifier_.classify(armor);
            if(!classifier_.checkArmorName(armor)) continue;
            if(!NumberClassifier::checkArmorType(armor)) continue;;
            candidates.push_back(armor);
        }
    }
    
    // 去重：共享灯条的装甲板只保留最优解
    std::vector<bool> removed(candidates.size(), false);
    for (size_t i = 0; i < candidates.size(); i++) {
        if (removed[i]) continue;
        for (size_t j = i + 1; j < candidates.size(); j++) {
            if (removed[j]) continue;
            int li = candidates[i].paired_lights_[0].id_;
            int ri = candidates[i].paired_lights_[1].id_;
            int lj = candidates[j].paired_lights_[0].id_;
            int rj = candidates[j].paired_lights_[1].id_;
            // 没有共用灯条，跳过
            if (li != lj && li != rj && ri != lj && ri != rj) continue;
            // 共用同一侧灯条：保留 ROI 面积小的（匹配更精确）
            if (li == lj || ri == rj) {
                int area_i = candidates[i].number_roi_.cols * candidates[i].number_roi_.rows;
                int area_j = candidates[j].number_roi_.cols * candidates[j].number_roi_.rows;
                if (area_i <= area_j) removed[j] = true;
                else removed[i] = true;
            }
            // 共用相邻侧灯条：保留置信度高的
            else {
                if (candidates[i].confidence_ >= candidates[j].confidence_) removed[j] = true;
                else removed[i] = true;
            }
        }
    }
    std::vector<Armor> result;
    for (size_t i = 0; i < candidates.size(); i++) {
        if (!removed[i]) result.push_back(candidates[i]);
    }
    return result;
}
////////// ===== 绘制函数 ===== /////////
void Detector::drawArmors(cv::Mat& img)
{
    for (const auto& armor : armors_) {
        // 画交叉线（紫色）
        cv::line(img, armor.points_[0], armor.points_[2], cv::Scalar(255, 0, 255), 2);
        cv::line(img, armor.points_[1], armor.points_[3], cv::Scalar(255, 0, 255), 2);
    }
}
void Detector::drawAllLights(cv::Mat& img)
{
    for (const auto& light : find_lights_) drawRotatedRect(img, light.rect_); // 默认是蓝色
}
////////// ===== 数字识别 ===== /////////
cv::Mat Detector::getNumberROI(const cv::Mat& img_bgr, const Armor& armor)
{
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

cv::Mat Detector::getNumberROI_AABB(const cv::Mat& img_bgr, const Armor& armor) const
{
    if (armor.paired_lights_.size() != 2) return cv::Mat();
    const auto& left = armor.paired_lights_[0];
    const auto& right = armor.paired_lights_[1];

    // top2bottom vector = bottom_ - top_
    cv::Point2f left_tb = left.bottom_ - left.top_;
    cv::Point2f right_tb = right.bottom_ - right.top_;

    // Extend by 1.125x to get full armor corners
    // 1.125 = 0.5 * armor_height / lightbar_length = 0.5 * 126mm / 56mm
    cv::Point2f tl = left.center_ - left_tb * 1.125f;
    cv::Point2f bl = left.center_ + left_tb * 1.125f;
    cv::Point2f tr = right.center_ - right_tb * 1.125f;
    cv::Point2f br = right.center_ + right_tb * 1.125f;

    // AABB clamp to image bounds
    int roi_left = std::max(static_cast<int>(std::min(tl.x, bl.x)), 0);
    int roi_top = std::max(static_cast<int>(std::min(tl.y, tr.y)), 0);
    int roi_right = std::min(static_cast<int>(std::max(tr.x, br.x)), img_bgr.cols);
    int roi_bottom = std::min(static_cast<int>(std::max(bl.y, br.y)), img_bgr.rows);

    if (roi_right <= roi_left || roi_bottom <= roi_top) return cv::Mat();
    return img_bgr(cv::Rect(roi_left, roi_top, roi_right - roi_left, roi_bottom - roi_top)).clone();
}

std::vector<cv::Mat> Detector::getNumberRois()
{
    std::vector<cv::Mat> rois;
    for (const auto& armor : armors_) {
        if (!armor.number_roi_.empty()) {
            rois.push_back(armor.number_roi_.clone());
        }
    }
    return rois;
}

/////////////////// 工具函数 //////////////////////////

void drawRotatedRect(cv::Mat& img, const cv::RotatedRect& rect, const cv::Scalar& color, int thickness)
{
    cv::Point2f vertices[4];
    rect.points(vertices);
    for (int i = 0; i < 4; i++) {
        cv::line(img, vertices[i], vertices[(i + 1) % 4], color, thickness);
    }
}

void drawRotatedRect(cv::Mat& img, const cv::Point2f& p1, const cv::Point2f& p2, const cv::Point2f& p3, const cv::Point2f& p4, const cv::Scalar& color, int thickness)
{
    cv::line(img, p1, p2, color, thickness);
    cv::line(img, p2, p3, color, thickness);
    cv::line(img, p3, p4, color, thickness);
    cv::line(img, p4, p1, color, thickness);
}
