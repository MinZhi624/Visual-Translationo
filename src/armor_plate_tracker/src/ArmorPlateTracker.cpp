#include "rclcpp/rclcpp.hpp"
#include "armor_plate_interfaces/msg/armor_plates.hpp"
#include "armor_plate_interfaces/msg/debug_tarcker.hpp"
#include "armor_plate_tracker/Tracker.hpp"

#include <vector>

using armor_plate_interfaces::msg::ArmorPlates;
using armor_plate_interfaces::msg::DebugTarcker;

class ArmorPlateTracker : public rclcpp::Node
{
private:
    rclcpp::Subscription<ArmorPlates>::SharedPtr armor_plates_sub_;
    rclcpp::Publisher<DebugTarcker>::SharedPtr debug_tracker_pub_;
    Tracker tracker_;

    void init()
    {
        armor_plates_sub_ = this->create_subscription<ArmorPlates>(
            "armor_plates", 
            10,
            std::bind(&ArmorPlateTracker::armor_plates_callback, this, std::placeholders::_1)
        );

        tracker_.setMaxLostTime(0.5);           // 最大丢失0.5秒
        tracker_.setMutationThreshold(3.0f, 2.0f);  // yaw突变3度，pitch突变2度
        tracker_.Init();

        debug_tracker_pub_ = this->create_publisher<DebugTarcker>("debug_tracker", 10);

        RCLCPP_INFO(this->get_logger(), "ArmorPlateTracker 初始化完成，开始订阅 /armor_plates");
    }

    void armor_plates_callback(const ArmorPlates::SharedPtr msg)
    {
        double current_time = this->now().seconds();
        size_t num = msg->armor_plates.size();
        
        RCLCPP_INFO(this->get_logger(), "接收到 %zu 个装甲板信息", num);

        std::vector<geometry_msgs::msg::Point> positions;
        std::vector<float> image_distances;
        positions.reserve(num);
        image_distances.reserve(num);

        for (size_t i = 0; i < num; ++i) {
            positions.push_back(msg->armor_plates[i].pose.position);
            image_distances.push_back(msg->armor_plates[i].image_distance_to_center);
            const auto& p = msg->armor_plates[i].pose.position;
            RCLCPP_INFO(this->get_logger(), "  [%zu] tvec: (%.3f, %.3f, %.3f), image_dist: %.2f",
                        i, p.x, p.y, p.z, msg->armor_plates[i].image_distance_to_center);
        }

        tracker_.Update(positions, image_distances, current_time);

        DebugTarcker debug_msg;
        debug_msg.orignal_yaw = tracker_.getMeasuredYaw();
        debug_msg.orignal_pitch = tracker_.getMeasuredPitch();
        debug_msg.filter_yaw = tracker_.getYaw();
        debug_msg.filter_pitch = tracker_.getPitch();
        debug_tracker_pub_->publish(debug_msg);

        if (tracker_.isLost()) {
            RCLCPP_WARN(this->get_logger(), "目标丢失，已丢失 %.3f 秒", tracker_.getLostTime(current_time));
        } else {
            float yaw = tracker_.getYaw();
            float pitch = tracker_.getPitch();
            RCLCPP_INFO(this->get_logger(), "跟踪结果 -> yaw: %.2f, pitch: %.2f", yaw, pitch);
        }
    }

public:
    ArmorPlateTracker() : Node("armor_plate_tracker_node_cpp")
    {
        RCLCPP_INFO(this->get_logger(), "Armor Plate Tracker节点创建成功！");
        init();
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ArmorPlateTracker>());
    rclcpp::shutdown();
    return 0;
}
