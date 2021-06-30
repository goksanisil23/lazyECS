#pragma once

#include <cmath>
#include <iostream>

namespace p_field {

struct Position {
    float x{}; // x is right, y is up, z is towards camera in rp3d
    float z{};

    Position() = default;
    Position(const float& x_in, const float& z_in) : x(x_in), z(z_in) {}
    
    bool operator == (const Position& pos) const {
        // std::cout << std::abs(this->x - pos.x) << " " << std::abs(this->z - pos.z) << std::endl;
        return std::abs(this->x - pos.x)<0.0001 && std::abs(this->z - pos.z)<0.0001;
        // return (this->x == pos.x) && (this->z == pos.z);
    }

    bool operator != (const Position& pos) const {
        return !(pos == *this);
    }    

    float CalculateDistance(const Position& target) const {
        return std::sqrt((x - target.x) * (x - target.x) + (z - target.z) * (z - target.z));
    }
    float CalculateDistanceSquared(const Position& target) const {
        return (x - target.x) * (x - target.x) + (z - target.z) * (z - target.z);
    }
};

}