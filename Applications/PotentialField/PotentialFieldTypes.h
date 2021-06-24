#pragma once

#include <cmath>

namespace p_field {

struct Position {
    float x{};
    float y{};

    Position() = default;
    Position(const float& x_in, const float& y_in) : x(x_in), y(y_in) {}
    
    float CalculateDistance(const Position& target) const {
        return std::sqrt((x - target.x) * (x - target.x) + (y - target.y) * (y - target.y));
    }
    float CalculateDistanceSquared(const Position& target) const {
        return (x - target.x) * (x - target.x) + (y - target.y) * (y - target.y);
    }        
};

}