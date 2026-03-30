// // 这是之前项目的文件，临时用于查看
// #include "GlobleFile.hpp"
// #include "Recognize.hpp"
// #include "PairedLights.hpp"
// #include "SolvePnP.hpp"
// #include "ScatterPlotData.hpp"
// #include "FilterPlotData.hpp"
// #include "DataManagement.hpp"

// #include <opencv2/highgui.hpp>
// #include <opencv2/calib3d.hpp>
// #include <cv_bridge/cv_bridge.h>

// #include "rclcpp/rclcpp.hpp"
// #include "sensor_msgs/msg/image.hpp"
// #include "geometry_msgs/msg/point.hpp"
// #include "tf2_ros/transform_broadcaster.h"
// #include "geometry_msgs/msg/transform_stamped.hpp"
// #include "tf2/LinearMath/Quaternion.h"
// #include "second_test_interfaces/msg/yaws_msg.hpp"

// class Detector : public rclcpp::Node
// {
// public:
//     Detector() : Node("detector_node_cpp") , 
//     data_management(observed_yaws1_, observed_yaws2_)
//     {
//         RCLCPP_INFO(this->get_logger(), "处理节点创建成功");
//         // 创建坐标变换发布器
//         broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
//         // 创建Yaws消息发布器
//         yaws_pub_ = this->create_publisher<second_test_interfaces::msg::YawsMsg>("detector/yaws_msg", 10);
//         // 接受图像
//         img_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
//             "/camera/image_raw", 
//             50,
//             std::bind(&Detector::receiveCallBack, this, std::placeholders::_1)
//         );
//     }
// private:
//     //数据
//     std::deque<cv::Point3f> history_points_;
//     ScatterPlotData observed_yaws1_;
//     ScatterPlotData observed_yaws2_;
//     DataManagement data_management;
//     FillterPlotData filtered_yaws1_;
//     FillterPlotData filtered_yaws2_;
//     // 计时
//     double t_max = 0;
//     // 话题服务
//     rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub_;
//     rclcpp::Publisher<second_test_interfaces::msg::YawsMsg>::SharedPtr yaws_pub_;
//     // 坐标变换
//     std::shared_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
//     // 时间相关
//     rclcpp::Time last_stamp_; 
//     rclcpp::Time current_stamp_;
//     std_msgs::msg::Header header_;
//     void receiveCallBack(const sensor_msgs::msg::Image::SharedPtr msg)
//     {
//         header_ = msg->header;
//         current_stamp_ = header_.stamp;
//         double dt = 0.02;
//         if (last_stamp_.nanoseconds() != 0) {
//             dt = (current_stamp_ - last_stamp_).seconds();
//         }
//         cv_bridge::CvImagePtr cv_ptr;
//         try {
//             cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
//         } catch (cv_bridge::Exception& e) {
//             RCLCPP_ERROR(this->get_logger(), "图片转换错误,错误提示: %s", e.what());
//             return;
//         }
//         cv::Mat& frame = cv_ptr->image;
//         // 计时
// 		double t = (double)cv::getTickCount();
//         // 处理
//         cv::Mat mask = findTargetColor(frame);
// 		cv::Mat img_thre = preProcessing(mask);
// 		//找到灯条
// 		PairedLights lights;
// 		lights.findPairedLights(img_thre);
// 		//pnp解算
//         std::vector<float> observed_yaws;
// 		cv::Point2f center;
// 		for (int i = 0; i < lights.num_lights_; i++) {
// 			SolvePnP pnp(lights.getPairedLightPoints()[i]);
// 			history_points_.push_back((cv::Point3f)pnp.getTvec());
// 			//保留250个数据点用于计算旋转中心
// 			if (history_points_.size() > 250) {
// 				history_points_.pop_front();
// 			}
// 			cv::Point3f rotated_center = fitSphereCenter(history_points_);
// 			center = pnp.reprojection(rotated_center);
// 			//保存数据
// 			float yaw = pnp.getYaw();
// 			observed_yaws.push_back(yaw);
//             // 发布相对位姿
//             broadcastTransform(pnp.getTvec(), pnp.getRvec());
// 			//打印文字
// 			std::string text = "Yaw:" + std::to_string(yaw) + " Pitch:" + std::to_string(pnp.getPitch()) + " Distance:" + std::to_string(pnp.getDistance());
// 			cv::putText(frame, text, cv::Point(10, 30 + 30 * i), cv::FONT_HERSHEY_PLAIN, 1.5, cv::Scalar(255, 0, 255), 2);
// 		}
// 		// 数据保存以及卡尔曼滤波 (保存同步)
// 		if(lights.num_lights_ != 0) {
// 			data_management.updateState(lights.num_lights_);
// 			data_management.saveDatum(observed_yaws);
// 			filtered_yaws1_.filterData(observed_yaws1_, dt);
// 			filtered_yaws2_.filterData(observed_yaws2_, dt);
//             // 发布数据
//             publishYaw();
// 		}
// 		// 绘制图像
// 		lights.drawPairedLights(frame); // 匹配的灯条
// 		cv::circle(frame, center, 10, cv::Scalar(255, 255, 255), cv::FILLED);//旋转中心
//         // 计时展示
//         t = ((double)cv::getTickCount() - t) * 1000 / cv::getTickFrequency();
// 		t_max = std::max(t, t_max);
// 		cv::putText(frame, ("t: " + std::to_string(t) + " ms"), cv::Point(10, 100), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 255), 2);
// 		cv::putText(frame, ("t_max: " + std::to_string(t_max) + " ms"), cv::Point(10, 160), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 255), 2);
//         // 显示处理后的图像
//         cv::imshow("image", frame);
//         cv::waitKey(1);
//     }

//     void broadcastTransform(const cv::Mat& tvec, const cv::Mat& rvec)
//     {
//         geometry_msgs::msg::TransformStamped tf;
//         tf.header.stamp = header_.stamp;
//         tf.header.frame_id = "camera";
//         tf.child_frame_id = "armor_plate";
//         // 平移（mm->m）
//         tf.transform.translation.x = tvec.at<double>(0) / 1000.0;
//         tf.transform.translation.y = tvec.at<double>(1) / 1000.0;
//         tf.transform.translation.z = tvec.at<double>(2) / 1000.0;
//         // 旋转向量 -> 四元数
//         double theta = cv::norm(rvec);
//         if (theta < 1e-12) {
//             tf.transform.rotation.w = 1.0;
//             tf.transform.rotation.x = 0.0;
//             tf.transform.rotation.y = 0.0;
//             tf.transform.rotation.z = 0.0;
//         } else {
//             cv::Vec3d axis = (cv::Vec3d)rvec / theta;
//             double half = theta * 0.5;
//             double w = std::cos(half);
//             double s = std::sin(half);
//             tf.transform.rotation.w = w;
//             tf.transform.rotation.x = axis[0] * s;
//             tf.transform.rotation.y = axis[1] * s;
//             tf.transform.rotation.z = axis[2] * s;
//         }
//         broadcaster_->sendTransform(tf);
//     }
//     void publishYaw()
//     {
//         second_test_interfaces::msg::YawsMsg msg;
//         msg.observed_yaw1 = observed_yaws1_.getData().back();
//         msg.observed_yaw2 = observed_yaws2_.getData().back();
//         msg.filtered_yaw1 = filtered_yaws1_.getData().back();
//         msg.filtered_yaw2 = filtered_yaws2_.getData().back();
//         yaws_pub_->publish(msg);
//     }
// };
// int main(int argc, char** argv)
// {
//     rclcpp::init(argc, argv);
//     rclcpp::spin(std::make_shared<Detector>());
//     rclcpp::shutdown();
//     return 0;
// }