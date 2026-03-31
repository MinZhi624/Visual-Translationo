#include "armor_plate_identification/PairedLights.hpp"
#include "armor_plate_identification/DrawTarget.hpp"
#include <opencv2/imgproc.hpp>

#ifdef DEBUG_INDENTIFICATION
    #include <iostream>
#endif
#include <rclcpp/rclcpp.hpp>
    
Lights::Lights() :
    center_(0, 0), top_(0, 0), bottom_(0, 0),
    angle_(0), length_(0), width_(0), area_(0),
    is_paired_(false)
{}

void PairedLights::findPairedLights(cv::Mat& img_thre)
{
    // 找到灯条然后拟合成直线
    std::vector<std::vector<cv::Point>> valued_contours = findLightsContours(img_thre);
    find_lights_ = findLightLines(valued_contours);
    // 拿到灯条后匹配 按x轴排序,从左到右
    std::sort(find_lights_.begin(), find_lights_.end(), [](const Lights& a, const Lights& b) {
        return a.center_.x < b.center_.x;
    });
    // 匹配灯条
    paired_lights_ = matchLights(find_lights_);
    // 更新灯条数量
    num_lights_ = paired_lights_.size();
}

std::vector<std::vector<cv::Point>> PairedLights::findLightsContours(cv::Mat& img_thre)
{
    std::vector<std::vector<cv::Point>> contours;
    std::vector<std::vector<cv::Point>> target_contours;
    cv::findContours(img_thre, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    // 选出合法的轮廓 --1. 面积不能太小， --2. 长宽比不能太大
    for (const auto& contour : contours) {
        int area = cv::contourArea(contour);
        if (area > MIN_CONTOURS_AREA) {
            cv::RotatedRect min_rect = cv::minAreaRect(cv::Mat(contour));
            double long_length = std::max(min_rect.size.width, min_rect.size.height);
            double short_length = std::min(min_rect.size.width, min_rect.size.height);
            double ratio =short_length / long_length;
            if (ratio < MAX_CONTOURS_RATIO && ratio > MIN_CONTOURS_RATIO) {
                target_contours.push_back(contour);
            }
        }
    }
    return target_contours;
}
// 规范化 RotatedRect：mode=0 保证 width>=height；mode=1 保证 height 为长边且 angle 在 [-45,45)
static cv::RotatedRect& adjustRec(cv::RotatedRect& rec, int mode)
{
    using std::swap;
    float& width = rec.size.width;
    float& height = rec.size.height;
    float& angle = rec.angle;

    if(mode == 0) // WIDTH_GREATER_THAN_HEIGHT
    {
        if(width < height)
        {
            swap(width, height);
            angle += 90.0;
        }
    }

    while(angle >= 90.0) angle -= 180.0;
    while(angle < -90.0) angle += 180.0;

    if(mode == 1) // ANGLE_TO_UP
    {
        if(angle >= 45.0)
        {
            swap(width, height);
            angle -= 90.0;
        }
        else if(angle < -45.0)
        {
            swap(width, height);
            angle += 90.0;
        }
    }

    return rec;
}

std::vector<Lights> PairedLights::findLightLines(std::vector<std::vector<cv::Point>>& contours)
{
    std::vector<Lights> lights_list;
    // 结合椭圆和 minAreaRect 的优点：
    // 1. 用椭圆拟合得到准确的方向
    // 2. 用 minAreaRect 的边界限制端点，防止超出轮廓
    for (auto & contour : contours) {
        // 1. 椭圆拟合提供准确方向，并规范化
        cv::RotatedRect ellipse_rect = cv::fitEllipse(contour);
        adjustRec(ellipse_rect, 0); // ANGLE_TO_UP：height 为长边，angle 在 [-45, 45)

        // 2. minAreaRect 提供边界约束（原始即可，只取最长边做限制）
        cv::RotatedRect min_rect = cv::minAreaRect(cv::Mat(contour));

        // 3. 用规范化后的椭圆角度计算中轴线单位方向向量
        float angle_rad = ellipse_rect.angle * CV_PI / 180.0f;
        cv::Point2f dir(std::cos(angle_rad), std::sin(angle_rad));
        float len = cv::norm(dir);
        if (len < 1e-6) continue;
        dir = dir / len;

        // 4. 计算中心点和半长
        cv::Point2f center = ellipse_rect.center;
        float ellipse_half_len = ellipse_rect.size.width * 0.5f;  // adjustRec 后 width 为长边
        float min_half_len = std::max(min_rect.size.width, min_rect.size.height) * 0.5f;
        float final_half_len = std::min(ellipse_half_len, min_half_len);

        // 5. 计算最终端点（限制在 minAreaRect 范围内）
        cv::Point2f final_top = center - dir * final_half_len;
        cv::Point2f final_bottom = center + dir * final_half_len;
        // 按照y最后再来一次排序，确保 top_ 在图像上方（y 较小）
        if (final_top.y > final_bottom.y) {
            std::swap(final_top, final_bottom);
        }

        // 6. 填充 Lights 对象
        Lights light;
        light.rect_ = ellipse_rect;  // 用规范化的 ellipse_rect 作为绘制框
        light.top_ = final_top;
        light.bottom_ = final_bottom;
        light.center_ = center;
        light.length_ = final_half_len * 2.0f;
        light.angle_ = std::atan2(dir.y, dir.x) * 180.0f / CV_PI;
        lights_list.push_back(light);
    }
    return lights_list;
}
bool PairedLights::checkPairLights(const Lights& light_left, const Lights& light_right) {
    //判断逻辑 
    float diff = std::abs(light_left.angle_ - light_right.angle_);
    diff = std::min(diff, 180 - diff);
    float length_ratio = std::min(light_left.length_, light_right.length_) / std::max(light_left.length_, light_right.length_);
    //角度差（方向） -> 边长比例（距离）
    if (diff > MAX_ANGLE_DIFF  || length_ratio < MIN_LENGTH_RATIO ) {
#ifdef DEBUG_INDENTIFICATION
        RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "当前是交差过大");
#endif
        return false;
    }
    if (length_ratio < MIN_LENGTH_RATIO) {
#ifdef DEBUG_INDENTIFICATION
        RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "当前是距两个灯条之间 长度 相差太远");
#endif
        return false;
    }

    // 建立以灯条平均方向为基准的局部坐标系，消除相机俯仰/滚转带来的透视畸变影响
    // 先处理 180° 周期性，确保两个角度在同一半周期内再平均
    float a1 = light_left.angle_ * CV_PI / 180.0f;
    float a2 = light_right.angle_ * CV_PI / 180.0f;
    if (std::abs(a1 - a2) > CV_PI / 2.0f) {
        if (a1 < a2) a1 += CV_PI;
        else a2 += CV_PI;
    }
    float avg_angle_rad = (a1 + a2) / 2.0f;
    // 规范回 [0, PI)
    while (avg_angle_rad >= CV_PI) avg_angle_rad -= CV_PI;
    while (avg_angle_rad < 0.0f) avg_angle_rad += CV_PI;

    float cos_a = std::cos(avg_angle_rad);
    float sin_a = std::sin(avg_angle_rad);

    float dx = light_right.center_.x - light_left.center_.x;
    float dy = light_right.center_.y - light_left.center_.y;

    // 旋转到局部坐标系：local_x 为装甲板宽度方向（垂直于灯条），local_y 为灯条长度方向（平行于灯条）
    float local_x = std::abs(-dx * sin_a + dy * cos_a);
    float local_y = std::abs(dx * cos_a + dy * sin_a);

    // 再判断x差比率和y差比率和相距距离与灯条长度比值
    double men_length = (light_left.length_ + light_right.length_) / 2.0;
    double x_diff_ratio = local_x / men_length;
    double y_diff_ratio = local_y / men_length;
    double distance_ratio = men_length / cv::norm(light_left.center_ - light_right.center_) ;
    if ( x_diff_ratio < MIN_X_DIFF_RATIO) {
#ifdef DEBUG_INDENTIFICATION
        RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "当前是距两个灯条之间 X 相差太远");
#endif
        return false;
    }
    if ( y_diff_ratio > MAX_Y_DIFF_RATIO ) {
#ifdef DEBUG_INDENTIFICATION
        RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "当前是距两个灯条之间 Y 相差太远");
#endif
        return false;
    }
    if ( distance_ratio > MAX_DISTANCE_RATIO) {
#ifdef DEBUG_INDENTIFICATION
        RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "当前是距两个灯条之间 距离 相差太 大");
#endif
        return false;
    }
    if (distance_ratio < MIN_DISTANCE_RATIO) {
#ifdef DEBUG_INDENTIFICATION
        RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "当前是距两个灯条之间 距离 相差太 小");
#endif
        return false;
    }
    // 全部检测通过
    return true;
}
std::vector<std::array<Lights, 2>> PairedLights::matchLights(std::vector<Lights>& all_lights)
{
    std::vector<std::array<Lights, 2>> paired_lights;
    for (size_t i = 0; i < all_lights.size(); i++) {
        for (size_t j = i + 1; j < all_lights.size(); j++) {
            if (checkPairLights(all_lights[i], all_lights[j])) {
                std::array<Lights, 2> pair_lights = { all_lights[i], all_lights[j] };
                paired_lights.push_back(pair_lights);
            }
        }
    }
    return paired_lights;
}
void PairedLights::drawAllLights(cv::Mat& img)
{
    for (const auto& light : find_lights_) {
        drawRotatedRect(img, light.rect_);
    }
}

void PairedLights::drawPairedLights(cv::Mat& img)
{
    for (const auto& pair_lights : paired_lights_) {
        // 这个是画出角点
        cv::circle(img, pair_lights[0].top_, 2, cv::Scalar(255, 0, 255), -1);
        cv::circle(img, pair_lights[0].bottom_, 2, cv::Scalar(255, 0, 255), -1);
        cv::circle(img, pair_lights[1].top_, 2, cv::Scalar(255, 0, 255), -1);
        cv::circle(img, pair_lights[1].bottom_, 2, cv::Scalar(255, 0, 255), -1);
        // 画出灯条
        cv::line(img, pair_lights[0].top_, pair_lights[0].bottom_, cv::Scalar(255, 0, 255), 1);
        cv::line(img, pair_lights[1].top_, pair_lights[1].bottom_, cv::Scalar(255, 0, 255), 1);
        cv::line(img, pair_lights[0].top_, pair_lights[1].top_, cv::Scalar(255, 0, 255), 1);
        cv::line(img, pair_lights[0].bottom_, pair_lights[1].bottom_, cv::Scalar(255, 0, 255), 1);
    }
}

std::vector<std::vector<cv::Point2f>> PairedLights::getPairedLightPoints() const
{
    std::vector<std::vector<cv::Point2f>> paired_points;
    for (const auto& pair_lights : paired_lights_) {
        std::vector<cv::Point2f> points = { pair_lights[0].top_, pair_lights[0].bottom_, pair_lights[1].top_, pair_lights[1].bottom_ };
        paired_points.push_back(points);
        
    }
    return paired_points;
}
