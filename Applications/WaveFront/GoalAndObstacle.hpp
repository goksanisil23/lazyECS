#pragma once

#include "WaveFrontTypes.h"
#include <reactphysics3d/reactphysics3d.h>
#include <vector>

class Obstacle {
public:

    explicit Obstacle() = default;

    std::vector<uint16_t> x_idx_; // position of the Obstacle in the 2D grid
    std::vector<uint16_t> z_idx_;

};

class Goal {
public:

    explicit Goal() = default;

    std::vector<uint16_t> x_idx_; // position of the Goal in the 2D grid
    std::vector<uint16_t> z_idx_;
};