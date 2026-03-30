#include "armor_plate_identification/DrawTarget.hpp"
#include <opencv2/imgproc.hpp>

void drawRotatedRect(cv::Mat img, cv::RotatedRect target_rect)
{
    cv::Point2f vertices[4];
    target_rect.points(vertices);
    // 绘制四条边
    for (int i = 0; i < 4; i++) {
        //骚气荧光绿
        cv::line(img, vertices[i], vertices[(i + 1) % 4], cv::Scalar(20, 255, 57), 2);
    }
}