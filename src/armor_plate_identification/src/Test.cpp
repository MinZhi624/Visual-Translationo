// 这个主要是一个测试文件，在没有相机的时候测试
#include "armor_plate_identification/Recognize.hpp"
#include "armor_plate_identification/PairedLights.hpp"
#include "armor_plate_identification/TestFunc.hpp"
#include "armor_plate_identification/PoseSolver.hpp"

#include "rclcpp/rclcpp.hpp"

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <algorithm>
#include <thread>
#include <chrono>

class Test : public rclcpp::Node
{
private:
    cv::VideoCapture c_;
    cv::Mat img_show_;
    PairedLights lights_;
    PoseSolver pose_solver_;
    
    rclcpp::TimerBase::SharedPtr timer_;
#ifdef DEBUG_BASE
    int play_delay_ms_ = 0; // 播放延迟，越大越慢
    int x = 10, y = 30, line_h = 25;
#endif
    DebugParamController debug_controller_;

    void init(const std::string& video_path)
    {
        // ==== 相机初始化 ==== //
        c_.open(video_path);
        // ==== 匹配参数初始化 ==== //
        lights_.MAX_ANGLE_DIFF = 10.0f;
        lights_.MIN_LENGTH_RATIO = 0.6f;
        lights_.MIN_X_DIFF_RATIO = 0.75f;
        lights_.MAX_Y_DIFF_RATIO = 1.0f;
        lights_.MAX_DISTANCE_RATIO = 0.8f;
        lights_.MIN_DISTANCE_RATIO = 0.1f;
        this->timer_ = this->create_wall_timer(std::chrono::milliseconds(5000),  std::bind(&Test::info, this));
        // ===== 初始化PoseSolver ===== //
        // 装甲板坐标系点左上角是0,顺时针排列
        std::vector<cv::Point3f> world_points_;
        world_points_.push_back(cv::Point3f(-67.5f, -27.5f, 0)); // 0
        world_points_.push_back(cv::Point3f(67.5f, -27.5f, 0)); // 1
        world_points_.push_back(cv::Point3f(67.5f, 27.5f, 0)); // 2
        world_points_.push_back(cv::Point3f(-67.5f, 27.5f, 0)); // 3
        // 初始化相机内参
        cv::Mat camera_matrix_ = (cv::Mat_<double>(3, 3) <<
            2374.54248, 0., 698.85288,
            0., 2377.53648, 520.8649,
            0., 0., 1.);
        // 相机畸变系数
        cv::Mat distortion_coefficients_ = (cv::Mat_<double>(1, 5) <<
            -0.059743, 0.355479, -0.000625, 0.001595, 0.000000);
        // 卡尔曼滤波 这里采用二阶的KF
        MyKalmanFilter myKF(2, 1);
        Eigen::MatrixXf A(2, 2);
        A << 1, 0.02,
            0, 1;
        myKF.setTransitionMatrix(A);
        Eigen::MatrixXf H(1, 2);
        H << 1, 0;
        myKF.setMeasurementMatrix(H);
        myKF.setErrorCovPost(Eigen::MatrixXf::Identity(2, 2));
        myKF.setStatePost(Eigen::MatrixXf::Zero(2, 1));
        myKF.setProcessNoiseCov(Eigen::MatrixXf::Identity(2, 2) * 0.05);
	    myKF.setMeasurementNoiseCov(Eigen::MatrixXf::Identity(1, 1) * 0.1);
        pose_solver_ = PoseSolver(world_points_,camera_matrix_, distortion_coefficients_, myKF, myKF);
#ifdef DEBUG_INDENTIFICATION
        RCLCPP_INFO(this->get_logger(), "灯条匹配识别DEBUG模式开启");
#endif
#ifdef DEBUG_PREPROCESSING
        RCLCPP_INFO(this->get_logger(), "图像预处理DEBUG模式开启");
#endif
#ifdef DEBUG_POSE
        RCLCPP_INFO(this->get_logger(), "姿态估计DEBUG模式开启");
#endif
    }
    void info()
    {
        // 计时器 5s发布一次信息
#ifdef DEBUG_INDENTIFICATION
        RCLCPP_INFO(this->get_logger(), "MAX_ANGLE_DIFF: %f, MAX_Y_DIFF_RATIO: %f, MIN_DISTANCE_RATIO: %f, MAX_DISTANCE_RATIO: %f, MIN_LENGTH_RATIO: %f, MIN_X_DIFF_RATIO: %f",
        lights_.MAX_ANGLE_DIFF, lights_.MAX_Y_DIFF_RATIO, lights_.MIN_DISTANCE_RATIO, lights_.MAX_DISTANCE_RATIO, lights_.MIN_LENGTH_RATIO, lights_.MIN_X_DIFF_RATIO
        );
#endif
    }
    void Identification(cv::Mat& image)
    {
        cv::Mat mask = findTargetColor(image);
        cv::Mat img_thre = preProcessing(mask);
        
        // 直接调用 findPairedLights 完成检测和匹配
        lights_.findPairedLights(img_thre);
        lights_.drawPairedLights(img_show_);
        
#ifdef DEBUG_PREPROCESSING
        // 预处理四图拼接显示
        // 绘制目标区域
        cv::Mat img_target;
        cv::bitwise_and(image, image, img_target, img_thre);
        std::vector<cv::Mat> images = {image, mask, img_thre, img_target};
        std::vector<std::string> labels = {"Original", "Color Mask", "Preprocessed", "Target Region"};
        showMultiImages("PreProcessions-View", images, labels);
#endif
#ifdef DEBUG_INDENTIFICATION
        debug_controller_.drawParams(img_show_, lights_, x, y, line_h);
        lights_.drawAllLights(img_show_);
        debug_controller_.drawDebugInfo(img_show_, play_delay_ms_, true, x, y, line_h);
#endif
    }
    void SolvePose()
    {
        // 每一个匹配好的灯条解算
        for (const auto& points : lights_.getPairedLightPoints()) {
            pose_solver_.solve(points);
            pose_solver_.solve(points);
#ifdef DEBUG_POSE
            pose_solver_.drawPose(img_show_);            
#endif
        }
    }
    void controlParams()
    {
        int key = cv::waitKey(1);
        if (key == -1) return;

        // ESC：退出
        if (key == 27) {
            rclcpp::shutdown();
            return;
        }

        // P：暂停
        if (key == 'p' || key == 'P') {
            RCLCPP_INFO(this->get_logger(), "暂停，按任意键继续...");
            cv::waitKey(0);
            return;
        }

#ifdef DEBUG_BASE
        if (debug_controller_.handleKey(key, lights_, play_delay_ms_, this->get_logger())) {
            return;
        }
#endif
    }
public:
    Test(std::string video_path) : Node("test_node_cpp")
    {
        RCLCPP_INFO(this->get_logger(), "测试节点已经启动");
        init(video_path);
        cv::Mat frame;
        while(true)
        {
            c_ >> frame;
            if (frame.empty()) {
                RCLCPP_INFO(this->get_logger(), "视频播放结束");
                return;
            }
            if (!rclcpp::ok()) return;
            img_show_ = frame.clone();
            // 图像处理
            Identification(frame);
            // 解算
            SolvePose();
            // 图片展示
            cv::imshow("img_show_", img_show_);
            // 按键控制
            controlParams();
            // 打印
            rclcpp::spin_some(this->get_node_base_interface());
            // 控制速度
#ifdef DEBUG_BASE
            std::this_thread::sleep_for(std::chrono::milliseconds(play_delay_ms_));
#endif
        }
        RCLCPP_INFO(this->get_logger(), "测试节点已经结束");
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    if (argc != 3) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "请输入正确的参数，输入视屏地址");
        return 1;
    }
    auto node = std::make_shared<Test>(argv[1]);
    rclcpp::shutdown();
    return 0;
}
