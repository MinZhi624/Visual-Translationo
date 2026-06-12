#pragma once
#include <cstddef>
#include <functional>

class DetectorArmor;
namespace armor_plate_identification::yaw {
struct YawSearchConfig;
struct YawSearchResult;
}
using YawErrorFunction = std::function<double(double)>;

class IYawSearchObserver {
public:
    virtual ~IYawSearchObserver() = default;
    virtual void onYawSearch(
        const DetectorArmor& armor,
        double center_yaw,
        const armor_plate_identification::yaw::YawSearchConfig& config,
        const armor_plate_identification::yaw::YawSearchResult& result,
        const YawErrorFunction& calculate_error,
        std::size_t armor_index) = 0;
};
