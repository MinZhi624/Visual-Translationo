#include "armor_plate_identification/PairedLights.hpp"
#include "armor_plate_identification/DrawTarget.hpp"
#include <opencv2/imgproc.hpp>

#include <iostream>

void PairedLights::findPairedLights(cv::Mat& img_thre)
{
    // 找到灯条然后拟合成直线
    std::vector<std::vector<cv::Point>> valued_contours = findLightsContours(img_thre);
    std::vector<std::vector<cv::Point2f>> all_lights = findLightLines(valued_contours);
    // 匹配灯条
    paired_light_points_ = matchLights(all_lights);
    // 更新灯条数量
    num_lights_ = paired_light_points_.size();
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
std::vector<std::vector<cv::Point2f>> PairedLights::findLightLines(std::vector<std::vector<cv::Point>>& contours)
{
    std::vector<std::vector<cv::Point2f>> light_lines;
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
            // 4. 保存结果
            light_lines.push_back({final_top, final_bottom});            
        }
    return light_lines;
}
bool PairedLights::checkPairLights(cv::Point2f p1, cv::Point2f p2, cv::Point2f p3, cv::Point2f p4) {
    //判断逻辑 
    //角度（方向） -> 边长比例（距离）
    float parallel = getParallelDegree(p1, p2, p3, p4);
    double left_length = cv::norm(p1 - p2);
    double right_length = cv::norm(p3 - p4);
    float length_ratio = left_length / right_length;
    //先判断角度 和 边长比例判断是否合理
    if (parallel < MIN_PARALLEL  || length_ratio < MIN_LENGTH_RATIO ) return false;
    // 再判断x差比率和y差比率和相距距离与灯条长度比值
    double men_length = (left_length + right_length) / 2;
    cv::Point2f left_center = (p1 + p2) / 2;
    cv::Point2f right_center = (p3 + p4) / 2;
    double x_diff_ratio = std::abs(left_center.x - right_center.x) / men_length;
    double y_diff_ratio = std::abs(left_center.y - right_center.y) / men_length;
    double distance_ratio = cv::norm(left_center - right_center) / men_length;
    if ( x_diff_ratio < MIN_X_DIFF_RATIO
         || y_diff_ratio > MAX_Y_DIFF_RATIO 
         || distance_ratio > MAX_DISTANCE_RATIO
         || distance_ratio < MIN_DISTANCE_RATIO 
    ) return false;
   
    return true;
}
std::vector<std::vector<cv::Point2f>> PairedLights::matchLights(std::vector<std::vector<cv::Point2f>>& all_lights)
{
    std::vector<std::vector<cv::Point2f>> paired_light_points;
    //遍历 C(n,2)
    for (size_t i = 0; i < all_lights.size(); i++) {
        const auto& light1 = all_lights[i];
        cv::Point2f p1 = light1[0];
        cv::Point2f p2 = light1[1];
        for (size_t j = i + 1; j < all_lights.size(); j++) {
            const auto& light2 = all_lights[j];
            cv::Point2f p3 = light2[0];
            cv::Point2f p4 = light2[1];
            if (checkPairLights(p1, p2, p3, p4)) {
                paired_light_points.push_back({p4, p2, p3, p1 });
            }
        }
    }
    return paired_light_points;
}
void PairedLights::drawPairedLights(cv::Mat& img)
{
    for (const auto& light_points : paired_light_points_) {
        //画出矩形
        drawQuadrangle(img, light_points[0], light_points[1], light_points[3], light_points[2], cv::Scalar(0, 255, 0));
        //标出四个点
        for (int j = 0; j < 4; j++) {
            std::string test = "(" + std::to_string((int)light_points[j].x) + "," + std::to_string((int)light_points[j].y) + ")";
            cv::putText(img, test, light_points[j], cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255, 0, 255));
        }
    }
}


std::vector<cv::Point2f> sortRotatedRectPoints(cv::RotatedRect& rect)
{
    cv::Point2f points[4];
    rect.points(points);
    //对y经行排序
    std::sort(points, points + 4, [](const cv::Point2f p1, const cv::Point2f p2) { return p1.y < p2.y; });
    for (int i = 0; i < 2; i++) {
        //对x进行排序
        std::sort(points + 2 * i, points + 2 * (i + 1), [](const cv::Point2f p1, const cv::Point2f p2) { return p1.x < p2.x; });
    }
    return std::vector<cv::Point2f>{points[0], points[1], points[2], points[3]};
}
void drawQuadrangle(cv::Mat& img, cv::Point2f p1, cv::Point2f p2, cv::Point2f p3, cv::Point2f p4, cv::Scalar color)
{
    cv::line(img, p1, p2, color, 2);
    cv::line(img, p2, p3, color, 2);
    cv::line(img, p3, p4, color, 2);
    cv::line(img, p4, p1, color, 2);
}

double getParallelDegree(cv::Point2f p1, cv::Point2f p2, cv::Point2f p3, cv::Point2f p4)
{
    cv::Vec2f vec1(p2.x - p1.x, p2.y - p1.y);
    cv::Vec2f vec2(p4.x - p3.x, p4.y - p3.y);
    if (cv::norm(vec1) < 1e-6 || cv::norm(vec2) < 1e-6) {
        return 0.0;
    }
    // 归一化
    vec1 = vec1 / cv::norm(vec1);
    vec2 = vec2 / cv::norm(vec2);
    // 计算点积
    double dot = vec1[0] * vec2[0] + vec1[1] * vec2[1];
    // 计算夹角（弧度）
    double angle_diff = acos(fmin(fmax(dot, -1.0), 1.0));
    // 考虑方向相反的情况
    angle_diff = std::min(angle_diff, CV_PI - angle_diff);
    // 转换为平行程度：0度=1（完全平行），90度=0（垂直）
    double parallel_degree = 1.0 - (angle_diff / (CV_PI / 2));
    return std::max(0.0, std::min(1.0, parallel_degree));
}