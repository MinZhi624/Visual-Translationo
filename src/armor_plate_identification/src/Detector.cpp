#include "armor_plate_identification/Detector.hpp"
#include <opencv2/core/types.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include "rclcpp/logging.hpp" 
#include <opencv2/highgui.hpp>
#include <opencv2/core/utils/filesystem.hpp>
#include <algorithm>
////////////////////// Detector /////////////////////////
void Detector::init()
{
    find_lights_.clear();
    armors_.clear();
    color_rejected_rects_.clear();
    num_lights_ = 0;
}
bool Detector::checkLightGeometry(const std::vector<cv::Point>& contour)
{
    // 检查轮廓点的个数 
    if (contour.size() < 6) return false;
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

cv::Mat Detector::preprocess(const cv::Mat& img_bgr, PreprocessDebug* debug_out)
{
    // 1. BLUE 通道低阈值初筛
    std::vector<cv::Mat> bgr;
    cv::split(img_bgr, bgr);
    cv::Mat color_ch = (TARGET_COLOR == "BLUE") ? bgr[0] : bgr[2];
    cv::Mat blue_dim_thre;
    cv::threshold(color_ch, blue_dim_thre, COLOR_THRESHOLD, 255, cv::THRESH_BINARY);

    // 2. GRAY 阈值
    cv::Mat gray;
    cv::cvtColor(img_bgr, gray, cv::COLOR_BGR2GRAY);
    cv::Mat gray_thre;
    cv::threshold(gray, gray_thre, GRAY_THRESHOLD, 255, cv::THRESH_BINARY);

    // 3. 对每个 BLUE_dim 区域，检查 GRAY 碎片数，构建合并二值图
    std::vector<std::vector<cv::Point>> blue_contours;
    cv::findContours(blue_dim_thre, blue_contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    // 过滤面积太小或点数不足的 BLUE 区域
    blue_contours.erase(
        std::remove_if(blue_contours.begin(), blue_contours.end(),
            [this](const auto& c) { return c.size() < 6 || cv::contourArea(c) < MIN_CONTOURS_AREA; }),
        blue_contours.end());
    cv::Mat img_thre = cv::Mat::zeros(img_bgr.size(), CV_8UC1);
    std::vector<std::pair<cv::Rect, int>> fragment_info;

    // 复用小 mask，避免循环内重复分配
    cv::Mat roi_mask;

    for (const auto& blue_contour : blue_contours) {
        cv::Rect blue_bb = cv::boundingRect(blue_contour);
        cv::Rect safe_bb = blue_bb & cv::Rect(0, 0, gray_thre.cols, gray_thre.rows);
        if (safe_bb.area() < 10) continue;
        cv::Mat gray_roi = gray_thre(safe_bb);

        // 在 GRAY 子图中找轮廓
        std::vector<std::vector<cv::Point>> gray_roi_contours;
        cv::findContours(gray_roi, gray_roi_contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        int valid_count = 0;
        for (const auto& c : gray_roi_contours) {
            if (cv::contourArea(c) > 5) valid_count++;
        }

        fragment_info.push_back({safe_bb, valid_count});

        if (valid_count == 1) {
            // GRAY 正常 → 直接拷贝 ROI 像素
            gray_thre(safe_bb).copyTo(img_thre(safe_bb));
        } else {
            // GRAY 断裂(2+) 或丢失(0) → 用 BLUE_dim 的像素（小 mask 限定轮廓形状）
            roi_mask = cv::Mat::zeros(safe_bb.size(), CV_8UC1);
            // 轮廓点偏移到 ROI 坐标系
            std::vector<cv::Point> local_contour;
            local_contour.reserve(blue_contour.size());
            for (const auto& pt : blue_contour) {
                local_contour.push_back(pt - cv::Point(safe_bb.x, safe_bb.y));
            }
            cv::fillConvexPoly(roi_mask, local_contour, 255);
            blue_dim_thre(safe_bb).copyTo(img_thre(safe_bb), roi_mask);
        }
    }

    // 4. 形态学处理合并后的二值图
    cv::Mat kernal_dilate = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 5));
    cv::Mat kernal_erode = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 5));
    cv::dilate(img_thre, img_thre, kernal_dilate);
    cv::erode(img_thre, img_thre, kernal_erode);

    // 5. 填充调试图像数据
    if (debug_out) {
        debug_out->blue_dim_thre = blue_dim_thre.clone();
        debug_out->gray_thre = gray_thre.clone();
        debug_out->merged_thre = img_thre.clone();
        debug_out->fragment_info = fragment_info;
    }

    return img_thre;
}

void Detector::detectArmors(cv::Mat& img_thre, const cv::Mat& img_bgr)
{
    // 初始化（重置）参数
    init();
    // 找到灯条然后拟合成直线
    std::vector<std::vector<cv::Point>> valued_contours = findLightsContours(img_thre, img_bgr);
    find_lights_ = findLightLines(valued_contours);
    // 拿到灯条后匹配 按x轴排序,从左到右
    std::sort(find_lights_.begin(), find_lights_.end(), [](const Lights& a, const Lights& b) {
        return a.center_.x < b.center_.x;
    });
    // 匹配灯条
    armors_ = matchLights(find_lights_);
    // 得到装甲板的号码ROI
    for (auto& armor : armors_) {
        armor.number_roi_ = getNumberROI(img_bgr, armor);
    }
    // 更新装甲板数量
    num_lights_ = armors_.size();
}
std::vector<std::vector<cv::Point>> Detector::findLightsContours(cv::Mat& img_thre, const cv::Mat& img_bgr)
{
    std::vector<std::vector<cv::Point>> contours;
    std::vector<std::vector<cv::Point>> target_contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(img_thre, contours, hierarchy, cv::RETR_CCOMP, cv::CHAIN_APPROX_NONE);
    // 选出合法的轮廓 --1. 面积不能太小， --2. 长宽比不能太大
    for (const auto& contour : contours) {
        if (!checkLightGeometry(contour)) continue;
        cv::RotatedRect min_rect = cv::minAreaRect(cv::Mat(contour));
        if (TargetColorDectect(img_bgr, min_rect, contour)) {
            target_contours.push_back(contour);
        } else {
            color_rejected_rects_.push_back(min_rect);
        }
    }
    return target_contours;
}
std::vector<Lights> Detector::findLightLines(std::vector<std::vector<cv::Point>>& contours)
{
    std::vector<Lights> lights_list;
    // 结合椭圆和 minAreaRect 的优点：
    // 1. 用椭圆拟合得到准确的方向
    // 2. 用 minAreaRect 的边界限制端点，防止超出轮廓
    for (auto & contour : contours) {
        if (contour.size() < 5) continue;  // 椭圆拟合至少需要 5 个点
        // 1. 椭圆拟合提供准确方向
        cv::RotatedRect ellipse_rect = cv::fitEllipse(contour);
        // 2. minAreaRect 提供边长边界约束
        cv::RotatedRect min_rect = cv::minAreaRect(contour);
        // 3. 用规范化后的椭圆角度计算中轴线单位方向向量
        double angle_rad = (ellipse_rect.angle + 90) * CV_PI / 180.0f;
        cv::Point2f dir = cv::Point2f(cos(angle_rad), sin(angle_rad));
        if (abs(dir.y) > 0.8f) {
            if (dir.y > 0) dir  = -dir;
        }
        else if(dir.x < 0) {
            dir = -dir;
        }
        // 这里采用点是为了方便计算
        double len = cv::norm(dir);
        if (len < 1e-6) continue;
        dir = dir / len;
        // 4. 计算中心点和半长
        cv::Point2f center = ellipse_rect.center;
        double half_len = std::max(min_rect.size.width, min_rect.size.height) / 2;

        // 5. 计算最终端点（限制在 minAreaRect 范围内）
        cv::Point2f final_top = center + dir * half_len;
        cv::Point2f final_bottom = center - dir * half_len;

        // 6. 填充 Lights 对象
        Lights light;
        light.rect_ = ellipse_rect;  // 用规范化的 ellipse_rect 作为绘制框
        light.top_ = final_top;
        light.bottom_ = final_bottom;
        light.center_ = center;
        light.length_ = half_len * 2.0f;
        light.angle_ = atan2(dir.y, dir.x);
        light.id_ = static_cast<int>(lights_list.size());
        lights_list.push_back(light);
    }
    return lights_list;
}

bool Detector::TargetColorDectect(const cv::Mat& img_bgr,const cv::RotatedRect& rect, const std::vector<cv::Point>& contour)
{
    cv::Rect bbox = rect.boundingRect();
    // 裁剪到图像边界内，防止 ROI 越界崩溃
    bbox &= cv::Rect(0, 0, img_bgr.cols, img_bgr.rows);
    if (bbox.empty()) return false;

    // 1. 生成轮廓 mask（只统计轮廓内部像素，且腐蚀后只看中心，避开光晕边缘）
    // 这样避免双重循环然后得到目标内容
    cv::Mat mask = cv::Mat::zeros(bbox.size(), CV_8UC1);
    std::vector<cv::Point> local_contour;
    local_contour.reserve(contour.size());
    for (const auto& pt : contour) {
        local_contour.emplace_back(pt - cv::Point(bbox.x, bbox.y));
    }
    cv::drawContours(mask, std::vector<std::vector<cv::Point>>{local_contour}, -1, 255, -1);
    cv::erode(mask, mask, cv::Mat());
    // 2. 提取 ROI 并分离 B/R 通道，用 mask 求平均
    cv::Mat roi = img_bgr(bbox);
    std::vector<cv::Mat> bgr;
    cv::split(roi, bgr);
    double mean_b = cv::mean(bgr[0], mask)[0];
    double mean_r = cv::mean(bgr[2], mask)[0];
    // 3. 比例判断，对光照变化更鲁棒 --> 避免杂光(白)
    bool is_red  = (mean_r / (mean_b + 1.0)) > 1.1;
    bool is_blue = (mean_b / (mean_r + 1.0)) > 1.1;
    return (is_red && TARGET_COLOR == "RED") || (is_blue && TARGET_COLOR == "BLUE");
}

bool Detector::checkPairLights(const Lights& light_left, const Lights& light_right) {
    //判断逻辑 
    float diff = std::abs((light_left.angle_ - light_right.angle_) * 180.0 / CV_PI);
    diff = std::min(diff, 180 - diff);
    float length_ratio = std::min(light_left.length_, light_right.length_) / std::max(light_left.length_, light_right.length_);
    //角度差（方向） -> 边长比例（距离）
    if (diff > MAX_ANGLE_DIFF || length_ratio < MIN_LENGTH_RATIO) return false;
    // 相机坐标系下面的距离
    double global_x_diff = light_right.center_.x - light_left.center_.x;   // 已经按x排过序，自然 >= 0
    double global_y_diff = light_right.center_.y - light_left.center_.y;   // 保留符号！
    // 计算灯条的局部坐标系下面的距离
    // local_x -> 垂直于灯条的方向
    // local_y -> 沿着灯条的方向
    double sinx = std::sin(light_left.angle_);
    double cosx = std::cos(light_left.angle_);
    double local_x = std::abs(-global_x_diff * sinx + global_y_diff * cosx);
    double local_y = std::abs(global_x_diff * cosx + global_y_diff * sinx);
    // 再判断x差比率和y差比率和相距距离与灯条长度比值
    double men_length = (light_left.length_ + light_right.length_) / 2.0;
    double x_diff_ratio = local_x / men_length;
    double y_diff_ratio = local_y / men_length;
    double distance_ratio = men_length / cv::norm(light_left.center_ - light_right.center_) ;
    if ( x_diff_ratio < MIN_X_DIFF_RATIO || y_diff_ratio > MAX_Y_DIFF_RATIO
        || distance_ratio > MAX_DISTANCE_RATIO || distance_ratio < MIN_DISTANCE_RATIO
    )  return false;
    // 全部检测通过
    return true;
}
float Detector::computePairScore(const Lights& light_left, const Lights& light_right) {
    // 1. 平行度：角度差越小越好
    float diff = std::abs((light_left.angle_ - light_right.angle_) * 180.0 / CV_PI);
    diff = std::min(diff, 180.0f - diff);
    float angle_score = 1.0f - std::min(diff / MAX_ANGLE_DIFF, 1.0f);

    // 2. 长度比：越接近1越好
    float length_ratio = std::min(light_left.length_, light_right.length_) / std::max(light_left.length_, light_right.length_);

    // 3. 中心距/灯条长度比：越接近2.0（标准装甲板宽高比）越好
    double men_length = (light_left.length_ + light_right.length_) / 2.0;
    double distance = cv::norm(light_left.center_ - light_right.center_);
    double dist_ratio = distance / men_length;
    float dist_score = 1.0f - std::min(std::abs((float)dist_ratio - 2.0f), 1.0f);

    // 4. y差比：沿灯条方向的偏移越小越好
    double global_x_diff = light_right.center_.x - light_left.center_.x;
    double global_y_diff = light_right.center_.y - light_left.center_.y;
    double sinx = std::sin(light_left.angle_);
    double cosx = std::cos(light_left.angle_);
    double local_y = std::abs(global_x_diff * cosx + global_y_diff * sinx);
    float y_diff_ratio = static_cast<float>(local_y / men_length);
    float y_score = 1.0f - std::min(y_diff_ratio / MAX_Y_DIFF_RATIO, 1.0f);

    return angle_score + length_ratio + dist_score + y_score;
}
std::vector<Armor> Detector::matchLights(std::vector<Lights>& all_lights)
{
    struct Candidate {
        std::array<Lights, 2> pair;
        int idx1;
        int idx2;
        float score;
    };
    std::vector<Candidate> candidates;
    for (size_t i = 0; i < all_lights.size(); i++) {
        for (size_t j = i + 1; j < all_lights.size(); j++) {
            if (checkPairLights(all_lights[i], all_lights[j])) {
                float score = computePairScore(all_lights[i], all_lights[j]);
                candidates.push_back({{all_lights[i], all_lights[j]}, static_cast<int>(i), static_cast<int>(j), score});
            }
        }
    }
    // 按 score 降序排序，优先保留"最像装甲板"的配对
    std::sort(candidates.begin(), candidates.end(), [](const Candidate& a, const Candidate& b) {
        return a.score > b.score;
    });
    // NMS 去重：一个灯条只能属于一个装甲板
    std::vector<Armor> armors;
    std::vector<bool> used(all_lights.size(), false);
    for (const auto& c : candidates) {
        if (!used[c.idx1] && !used[c.idx2]) {
            Armor armor;
            armor.paired_lights_ = c.pair;
            armor.points_ = {c.pair[0].top_, c.pair[1].top_, c.pair[1].bottom_, c.pair[0].bottom_};
            armors.push_back(armor);
            used[c.idx1] = true;
            used[c.idx2] = true;
        }
    }
    return armors;
}

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
    for (const auto& light : find_lights_) {
        drawRotatedRect(img, light.rect_);
        cv::putText(img, "top", light.top_, cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255, 0, 255));
        cv::putText(img, "bottom", light.bottom_, cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255, 0, 255));
    }
}

void Detector::drawColorRejected(cv::Mat& img)
{
    for (const auto& rect : color_rejected_rects_) {
        drawRotatedRect(img, rect, cv::Scalar(0, 255, 255), 2);  // 黄色
    }
}

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
    number_origin_rois_.push_back(number_roi.clone());
    // 预处理：灰度化 + 二值化
    cv::cvtColor(number_roi, number_roi, cv::COLOR_BGR2GRAY);
    return number_roi;
}
void Detector::saveNumberRoi()
{
    static int outputPictureCounts = 0;
    for(const auto& armor : armors_) {
        if (outputPictureCounts < 20 && is_star_save_) {
        // 确保输出目录存在
            if (!cv::utils::fs::createDirectories(outputPath_)) {
                RCLCPP_WARN_ONCE(rclcpp::get_logger("Detector"),
                    "Failed to create directory: %s", outputPath_.c_str());
            }
            std::string file_name = outputPath_ + "/roi_" + std::to_string(++outputPictureCounts_) + ".png";
            if (!cv::imwrite(file_name, armor.number_roi_.clone())) {
                RCLCPP_WARN_ONCE(rclcpp::get_logger("Detector"),
                    "Failed to write image: %s", file_name.c_str());
            }
            RCLCPP_INFO(rclcpp::get_logger("Detector"), "保存成功");
        }
        if (outputPictureCounts >= 20 && is_star_save_) {
            outputPictureCounts = 0;
            is_star_save_ = false;
            RCLCPP_INFO(rclcpp::get_logger("Detector"), "阶段性保存结束");
        }
    }
}

void Detector::showNumberROI()
{
    if(number_origin_rois_.empty()) return;
    // 简单拼接显示所有号码ROI
    cv::Mat concat_img;
    cv::hconcat(number_origin_rois_, concat_img);
    cv::imshow("Number Origin ROIs", concat_img);
    // 清除数据，避免重复显示
    number_origin_rois_.clear();
}

void Detector::showNumberBinaryROI()
{
    if (armors_.empty()) return;
    std::vector<cv::Mat> valid_rois;
    for (const auto& armor : armors_) {
        if (!armor.number_roi_.empty()) {
            valid_rois.push_back(armor.number_roi_);
        }
    }
    cv::Mat concat_img;
    cv::hconcat(valid_rois, concat_img);
    cv::imshow("Number ROIs", concat_img);
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
