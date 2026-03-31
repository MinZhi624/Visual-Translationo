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
            cv::Rect bbox = cv::boundingRect(contour);
            float ratio = (float)bbox.width / bbox.height;
            if (ratio < MAX_CONTOURS_RATIO && ratio > MIN_CONTOURS_RATIO)
            {
                target_contours.push_back(contour);
            }
        }
    }
    return target_contours;
}
std::vector<Lights> PairedLights::findLightLines(std::vector<std::vector<cv::Point>>& contours)
{
    std::vector<Lights> lights_list;
    // 结合椭圆和 minAreaRect 的优点：
        // 1. 用椭圆拟合得到准确的方向和端点位置
        // 2. 用 minAreaRect 的边界限制椭圆端点，防止超出轮廓
        for (auto & contour : contours) {
            // 1. 获取椭圆拟合结果（准确方向）
            cv::RotatedRect ellipse_rect = cv::fitEllipse(contour);
            cv::Point2f e_points[4];
            ellipse_rect.points(e_points);
            // 按y排序，上面两点和下面两点
            std::sort(e_points, e_points + 4, [](const cv::Point2f& a, const cv::Point2f& b) {
                return a.y < b.y;
            });
            // 椭圆中轴线端点
            cv::Point2f e_top = (e_points[0] + e_points[1]) / 2;
            cv::Point2f e_bottom = (e_points[2] + e_points[3]) / 2;

            // 2. 获取 minAreaRect 边界（范围约束）
            cv::RotatedRect min_rect = cv::minAreaRect(cv::Mat(contour));
            cv::Point2f m_points[4];
            min_rect.points(m_points);
            std::sort(m_points, m_points + 4, [](const cv::Point2f& a, const cv::Point2f& b) {
                return a.y < b.y;
            });
            // minAreaRect 的中轴线端点（边界）
            cv::Point2f m_top = (m_points[0] + m_points[1]) / 2;
            cv::Point2f m_bottom = (m_points[2] + m_points[3]) / 2;
            // 3. 结合：椭圆端点限制在 minAreaRect 边界内
            // 计算椭圆方向向量
            cv::Point2f dir = e_bottom - e_top;
            float len = cv::norm(dir);
            if (len < 1e-6) continue;
            dir = dir / len;  // 单位向量
            // 限制方法：从椭圆中心沿方向延伸，但长度不超过 minAreaRect 的半长
            cv::Point2f center = (e_top + e_bottom) / 2;
            float m_half_len = cv::norm(m_top - m_bottom) / 2;
            // 重新计算：从中心沿椭圆方向，但限制在 minAreaRect 范围内
            cv::Point2f final_top = center - dir * std::min(cv::norm(e_top - center),(double) m_half_len);
            cv::Point2f final_bottom = center + dir * std::min(cv::norm(e_bottom - center),(double) m_half_len);
            // 4. 填充Lights对象并保存
            Lights light;
            light.top_ = final_top;
            light.bottom_ = final_bottom;
            light.center_ = center;
            light.length_ = cv::norm(final_top - final_bottom);
            light.angle_ = atan2(dir.y, dir.x) * 180 / CV_PI;
            lights_list.push_back(light);            
        }
    return lights_list;
}
bool PairedLights::checkPairLights(const Lights& light_left, const Lights& light_right) {
    //判断逻辑 
    float diff = std::abs(light_left.angle_ - light_right.angle_);
    diff = std::min(diff, 180 - diff);
    float length_ratio = std::min(light_left.length_, light_left.length_) / std::max(light_left.length_, light_left.length_);
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

    // 再判断x差比率和y差比率和相距距离与灯条长度比值
    double men_length = (light_left.length_ + light_right.length_) / 2;
    double x_diff_ratio = std::abs(light_left.center_.x - light_right.center_.x) / men_length;
    double y_diff_ratio = std::abs(light_left.center_.y - light_right.center_.y) / men_length;
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
