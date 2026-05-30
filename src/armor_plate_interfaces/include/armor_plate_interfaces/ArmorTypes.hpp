#pragma once

#include <string>

enum class ArmorType
{
    SMALL,
    LARGE
};

enum class Color
{
    RED,
    BLUE,
    NONE
};

enum class ArmorName : int
{
    ONE = 1,
    TWO = 2,
    THREE = 3,
    FOUR = 4,
    FIVE = 5,
    NONE = 0
};

inline Color stringToColor(const std::string& s)
{
    if (s == "RED") return Color::RED;
    if (s == "BLUE") return Color::BLUE;
    return Color::NONE;
}

inline ArmorName intToArmorName(int id)
{
    switch (id) {
        case 1: return ArmorName::ONE;
        case 2: return ArmorName::TWO;
        case 3: return ArmorName::THREE;
        case 4: return ArmorName::FOUR;
        case 5: return ArmorName::FIVE;
        default: return ArmorName::NONE;
    }
}

inline std::string armorNameToString(const ArmorName& name)
{
    switch (name) {
        case ArmorName::ONE: return "1";
        case ArmorName::TWO: return "2";
        case ArmorName::THREE: return "3";
        case ArmorName::FOUR: return "4";
        case ArmorName::FIVE: return "5";
        default: return "NONE";
    }
}
