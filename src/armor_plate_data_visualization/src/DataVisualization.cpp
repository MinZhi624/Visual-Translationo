#include "ScatterPlotData.hpp"
#include "ScatterPlot.hpp"

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "rclcpp/rclcpp.hpp"
#include "armor_plate_interfaces/msg/debug_tracker.hpp"

class DataVisualizaiton : public rclcpp::Node
{
public:
    DataVisualizaiton() : Node("data_visualization_node_cpp")
    {
        RCLCPP_INFO(this->get_logger(), "数据化节点创建成功!");
        plot_img_ = cv::Mat(800, 1920, CV_8UC3, cv::Scalar(0, 0, 0));
        cv::Mat yaw_roi = plot_img_(cv::Rect(0, 0, 1920, 400));
        cv::Mat pitch_roi = plot_img_(cv::Rect(0, 400, 1920, 400));
        yaw_plot_ = ScatterPlot(yaw_roi);
        pitch_plot_ = ScatterPlot(pitch_roi);

        debug_sub_ = this->create_subscription<armor_plate_interfaces::msg::DebugTracker>(
            "debug_tracker",
            50,
            std::bind(&DataVisualizaiton::msgCallBack, this, std::placeholders::_1));
    }

private:
    rclcpp::Subscription<armor_plate_interfaces::msg::DebugTracker>::SharedPtr debug_sub_;
    ScatterPlotData observed_yaw_;
    ScatterPlotData observed_pitch_;
    ScatterPlotData filtered_yaw_;
    ScatterPlotData filtered_pitch_;
    cv::Mat plot_img_;
    ScatterPlot yaw_plot_;
    ScatterPlot pitch_plot_;

    void msgCallBack(const armor_plate_interfaces::msg::DebugTracker & msg)
    {
        // 处理数据
        observed_yaw_.saveDatum(msg.measurement_yaw);
        observed_pitch_.saveDatum(msg.measurement_pitch);
        filtered_yaw_.saveDatum(msg.filter_yaw);
        filtered_pitch_.saveDatum(msg.filter_pitch);

        // 绘制 Yaw（上半部分）
        yaw_plot_.clearPlot();
        yaw_plot_.drawXAxis();
        // 观测值为白色
        yaw_plot_.drawPlot(&observed_yaw_, cv::Scalar(255, 255, 255));
        // 过滤值为紫色
        yaw_plot_.drawPlot(&filtered_yaw_, cv::Scalar(255, 0, 255));

        // 绘制 Pitch（下半部分）
        pitch_plot_.clearPlot();
        pitch_plot_.drawXAxis();
        // 观测值为白色
        pitch_plot_.drawPlot(&observed_pitch_, cv::Scalar(255, 255, 255));
        // 过滤值为紫色
        pitch_plot_.drawPlot(&filtered_pitch_, cv::Scalar(255, 0, 255));

        // 添加文字标注
        cv::putText(plot_img_, "Yaw (white=original, purple=filtered)",
                    cv::Point(PLOT_MARGIN + 10, 30),
                    cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 0), 2);
        cv::putText(plot_img_, "Pitch (white=original, purple=filtered)",
                    cv::Point(PLOT_MARGIN + 10, 430),
                    cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 0), 2);

        cv::imshow("plot", plot_img_);
        cv::waitKey(1);
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DataVisualizaiton>());
    rclcpp::shutdown();
    return 0;
}
