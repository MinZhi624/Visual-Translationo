#include "armor_plate_planner/ArmorGenerator.hpp"
#include <cmath>

std::vector<armor_plate_interfaces::msg::TrackedArmor> ArmorGenerator::generate(
    const armor_plate_interfaces::msg::TrackedTarget & state) const
{
    std::vector<armor_plate_interfaces::msg::TrackedArmor> armors;
    armors.reserve(4);

    double yaw = state.yaw;
    double r = state.radius;

    for (int i = 0; i < 4; ++i) {
        double armor_angle = yaw + i * M_PI / 2.0;
        bool use_l_h = (i == 1 || i == 3);
        double radius = use_l_h ? r + state.radius_offset : r;
        double z = use_l_h ? state.center_world.z + state.height_offset : state.center_world.z;

        armor_plate_interfaces::msg::TrackedArmor armor;
        armor.position_world.x = state.center_world.x - radius * std::cos(armor_angle);
        armor.position_world.y = state.center_world.y - radius * std::sin(armor_angle);
        armor.position_world.z = z;
        armor.yaw_world = armor_angle;
        armor.armor_id = i;
        armors.push_back(armor);
    }

    return armors;
}
