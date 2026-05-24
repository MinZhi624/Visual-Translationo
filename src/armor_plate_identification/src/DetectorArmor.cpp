#include "armor_plate_identification/DetectorArmor.hpp"
#include <opencv2/imgproc.hpp>
#include <cmath>
#include <algorithm>

////////////////////// Light /////////////////////////

Light::Light(cv::RotatedRect ellipse_rect, cv::RotatedRect min_rect, Color color)
    : rect_(ellipse_rect), color_(color)
{
    // 这里用ellipse_rect来画图
    // 用椭圆角度计算方向，用 minAreaRect 尺寸约束端点
    center_ = min_rect.center;
    double angle_rad = (ellipse_rect.angle + 90) * CV_PI / 180.0;
    cv::Point2f dir = cv::Point2f(std::cos(angle_rad), std::sin(angle_rad));
    if (std::abs(dir.y) > 0.8f) {
        if (dir.y > 0) dir = -dir;
    } else if (dir.x < 0) {
        dir = -dir;
    }
    double len = cv::norm(dir);
    if (len < 1e-6) return;
    dir = dir / len;
    double half_len = std::max(min_rect.size.width, min_rect.size.height) / 2;
    top_ = center_ + dir * half_len;
    bottom_ = center_ - dir * half_len;
    angle_ = std::atan2(dir.y, dir.x);
    length_ = half_len * 2.0;
    width_ = std::min(min_rect.size.width, min_rect.size.height);
    area_ = static_cast<int>(min_rect.size.width * min_rect.size.height);
}

Color Light::getLightColor(const cv::Mat& img_bgr, const cv::RotatedRect& rect, const std::vector<cv::Point>& contour)
{
    cv::Rect bbox = rect.boundingRect();
    bbox &= cv::Rect(0, 0, img_bgr.cols, img_bgr.rows);
    if (bbox.empty()) return Color::NONE;

    // 生成轮廓 mask
    cv::Mat mask = cv::Mat::zeros(bbox.size(), CV_8UC1);
    std::vector<cv::Point> local_contour;
    local_contour.reserve(contour.size());
    for (const auto& pt : contour) {
        local_contour.push_back(pt - cv::Point(bbox.x, bbox.y));
    }
    cv::drawContours(mask, std::vector<std::vector<cv::Point>>{local_contour}, -1, 255, -1);
    cv::erode(mask, mask, cv::Mat());

    // 提取 ROI 并分离 B/R 通道
    cv::Mat roi = img_bgr(bbox);
    std::vector<cv::Mat> bgr;
    cv::split(roi, bgr);

    // 这样可以避免白色
    double mean_b = cv::mean(bgr[0], mask)[0];
    double mean_r = cv::mean(bgr[2], mask)[0];

    bool is_red  = (mean_r / (mean_b + 1.0)) > 1.1;
    bool is_blue = (mean_b / (mean_r + 1.0)) > 1.1;
    if (is_red) return Color::RED;
    if (is_blue) return Color::BLUE;
    return Color::NONE;
}

////////////////////// DetectorArmor /////////////////////////

DetectorArmor::DetectorArmor(Light& light_left, Light& light_right)
{
    paired_lights_ = {light_left, light_right};
    points_ = {light_left.top_, light_right.top_, light_right.bottom_, light_left.bottom_};

    // ===== 几何参数计算 ===== //
    // 角度差（度）
    double diff = std::abs((light_left.angle_ - light_right.angle_) * 180.0 / CV_PI);
    diff = std::min(diff, 180.0 - diff);
    angle_diff_ = diff;
    // 长度比
    length_ratio_ = std::min(light_left.length_, light_right.length_) / std::max(light_left.length_, light_right.length_);

    // 灯条局部坐标系下的距离
    double global_x_diff = light_right.center_.x - light_left.center_.x;
    double global_y_diff = light_right.center_.y - light_left.center_.y;
    double sinx = std::sin(light_left.angle_);
    double cosx = std::cos(light_left.angle_);
    double local_x = std::abs(-global_x_diff * sinx + global_y_diff * cosx);
    double local_y = std::abs(global_x_diff * cosx + global_y_diff * sinx);
    double men_length = (light_left.length_ + light_right.length_) / 2.0;
    x_diff_ratio_ = local_x / men_length;
    y_diff_ratio_ = local_y / men_length;
    // 距离比 = 平均灯条长度 / 中心距
    double distance = cv::norm(light_left.center_ - light_right.center_);
    double dist_ratio = distance / men_length;
    distance_ratio_ = men_length / distance;
    // ===== 计算装甲板类型 ===== //
    type_ = (dist_ratio > DIST_RATIO_THRESH) ? ArmorType::LARGE : ArmorType::SMALL;
}
