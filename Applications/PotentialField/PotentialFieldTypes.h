#pragma once

#include <cmath>

namespace p_field {

struct Position {
    float x{}; // x is right, y is up, z is towards camera in rp3d
    float z{};

    Position() = default;
    Position(const float& x_in, const float& z_in) : x(x_in), z(z_in) {}
    
    float CalculateDistance(const Position& target) const {
        return std::sqrt((x - target.x) * (x - target.x) + (z - target.z) * (z - target.z));
    }
    float CalculateDistanceSquared(const Position& target) const {
        return (x - target.x) * (x - target.x) + (z - target.z) * (z - target.z);
    }        
};

}