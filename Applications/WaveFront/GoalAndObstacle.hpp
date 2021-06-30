#pragma once

#include "WaveFrontTypes.h"
#include <reactphysics3d/reactphysics3d.h>

class Obstacle {
public:

    explicit Obstacle(const Position2D& position = Position2D()) : position_(position) {}

    Position2D position_;

};

class Goal {
public:

    explicit Goal(const Position2D& position = Position2D()) : position_(position) {}

    Position2D position_;
};