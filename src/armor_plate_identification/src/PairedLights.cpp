#include "armor_plate_identification/PairedLights.hpp"
#include <opencv2/imgproc.hpp>

#ifdef DEBUG_INDENTIFICATION
    #include <iostream>
    #include <rclcpp/rclcpp.hpp>
#endif


/////////////////////// Lights /////////////////////////////// 
Lights::Lights() :
    center_(0, 0), top_(0, 0), bottom_(0, 0),
    angle_(0), length_(0), width_(0), area_(0),
    is_paired_(false), id_(-1)
{}

////////////////////// PairedLights /////////////////////////
void PairedLights::init()
{
    find_lights_.clear();
    paired_lights_.clear();
    paired_lights_points_.clear();
    num_lights_ = 0;
}

void PairedLights::findPairedLights(cv::Mat& img_thre)
{
    // 初始化（重置）参数
    init();
    // 找到灯条然后拟合成直线
    std::vector<std::vector<cv::Point>> valued_contours = findLightsContours(img_thre);
    find_lights_ = findLightLines(valued_contours);
    // 拿到灯条后匹配 按x轴排序,从左到右
    std::sort(find_lights_.begin(), find_lights_.end(), [](const Lights& a, const Lights& b) {
        return a.center_.x < b.center_.x;
    });
    // 匹配灯条
    paired_lights_ = matchLights(find_lights_);
    // 将匹配好的装甲板4个点存储下来
    for (auto& lights : paired_lights_) {
        std::vector<cv::Point2f> points = {lights[0].top_, lights[1].top_, lights[1].bottom_, lights[0].bottom_};
        paired_lights_points_.push_back(points);
    }
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
std::vector<Lights> PairedLights::findLightLines(std::vector<std::vector<cv::Point>>& contours)
{
    std::vector<Lights> lights_list;
    // 结合椭圆和 minAreaRect 的优点：
    // 1. 用椭圆拟合得到准确的方向
    // 2. 用 minAreaRect 的边界限制端点，防止超出轮廓
    for (auto & contour : contours) {
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

bool PairedLights::checkPairLights(const Lights& light_left, const Lights& light_right) {
    //判断逻辑 
    float diff = std::abs((light_left.angle_ - light_right.angle_) * 180.0 / CV_PI);
    diff = std::min(diff, 180 - diff);
    float length_ratio = std::min(light_left.length_, light_right.length_) / std::max(light_left.length_, light_right.length_);
    //角度差（方向） -> 边长比例（距离）
    if (diff > MAX_ANGLE_DIFF) {
#ifdef DEBUG_INDENTIFICATION
        RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "交差过大 差距为\t%lf", diff);
#endif
        return false;
    }
    if (length_ratio < MIN_LENGTH_RATIO) {
#ifdef DEBUG_INDENTIFICATION
        RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "长度 相差 太大\t%lf", length_ratio);
#endif
        return false;
    }
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
    if ( x_diff_ratio < MIN_X_DIFF_RATIO) {
#ifdef DEBUG_INDENTIFICATION
        RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "X 差太近\t%lf",x_diff_ratio);
#endif
        return false;
    }
    if ( y_diff_ratio > MAX_Y_DIFF_RATIO) {
#ifdef DEBUG_INDENTIFICATION
        RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Y 差太远\t%lf",y_diff_ratio);
#endif
        return false;
    }
    if ( distance_ratio > MAX_DISTANCE_RATIO) {
#ifdef DEBUG_INDENTIFICATION
        RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "距离 相差 太 大\t%lf",distance_ratio);
#endif
        return false;
    }
    if (distance_ratio < MIN_DISTANCE_RATIO) {
#ifdef DEBUG_INDENTIFICATION
        RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "距离 相差 太 小\t%lf",distance_ratio);
#endif
        return false;
    }
    // 全部检测通过
    return true;
}
float PairedLights::computePairScore(const Lights& light_left, const Lights& light_right) {
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

    return angle_score + length_ratio + dist_score;
}
std::vector<std::array<Lights, 2>> PairedLights::matchLights(std::vector<Lights>& all_lights)
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
    std::vector<std::array<Lights, 2>> paired_lights;
    std::vector<bool> used(all_lights.size(), false);
    for (const auto& c : candidates) {
        if (!used[c.idx1] && !used[c.idx2]) {
            paired_lights.push_back(c.pair);
            used[c.idx1] = true;
            used[c.idx2] = true;
        }
    }
    return paired_lights;
}

void PairedLights::drawPairedLights(cv::Mat& img)
{
    for (const auto& points : paired_lights_points_) {
        
        for (int i = 0; i < 4; i++) {
            // 画出点
            cv::circle(img, points[i], 3, cv::Scalar(0, 255, 0), -1);
            cv::putText(img, std::to_string(i), points[i], cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(255, 0, 255));
            // 画出线条
            cv::line(img, points[i], points[(i + 1) % 4], cv::Scalar(0, 255, 0), 2);
        }
    }
}
void PairedLights::drawAllLights(cv::Mat& img)
{
    for (const auto& light : find_lights_) {
        drawRotatedRect(img, light.rect_);
        cv::putText(img, "top", light.top_, cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255, 0, 255));
        cv::putText(img, "bottom", light.bottom_, cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255, 0, 255));
    }
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