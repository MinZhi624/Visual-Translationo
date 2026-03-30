#include "armor_plate_identification/TestFunc.hpp"
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

void showMultiImages(const std::string& window_name, 
                     const std::vector<cv::Mat>& images,
                     const std::vector<std::string>& labels)
{
    if (images.empty()) return;
    
    // 固定输出窗口大小
    const int grid_cols = 2;  // 2列
    const int grid_rows = 2;  // 2行
    const int cell_width = 640;
    const int cell_height = 480;
    const int text_height = 30;  // 标签高度
    
    // 创建画布（2x2 网格）
    cv::Mat canvas((cell_height + text_height) * grid_rows, 
                   cell_width * grid_cols, 
                   CV_8UC3, cv::Scalar(0, 0, 0));
    
    for (size_t i = 0; i < images.size() && i < 4; ++i) {
        int row = i / grid_cols;
        int col = i % grid_cols;
        
        // 转换单通道为三通道
        cv::Mat img_display;
        if (images[i].channels() == 1) {
            cv::cvtColor(images[i], img_display, cv::COLOR_GRAY2BGR);
        } else {
            img_display = images[i].clone();
        }
        
        // resize 到固定大小
        cv::Mat img_resized;
        cv::resize(img_display, img_resized, cv::Size(cell_width, cell_height));
        
        // 计算放置位置
        int x = col * cell_width;
        int y = row * (cell_height + text_height);
        
        // 复制图像到画布
        cv::Rect roi(x, y + text_height, cell_width, cell_height);
        img_resized.copyTo(canvas(roi));
        
        // 绘制标签
        std::string label = (i < labels.size()) ? labels[i] : ("Image " + std::to_string(i + 1));
        cv::putText(canvas, label, cv::Point(x + 10, y + 25),
                    cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 255), 2);
    }
    
    cv::imshow(window_name, canvas);
}
