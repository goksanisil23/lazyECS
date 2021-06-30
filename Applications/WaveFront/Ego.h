#pragma once

#include "ECSCore/Types.hpp"
#include "Components/Tag.h"
#include "WaveFrontTypes.h"
// #include "GoalAndObstacle.hpp"

#include <vector>
#include <unordered_map>

class Ego {

public:
    explicit Ego(const Position2D& position = Position2D());

    Position2D position_; // simple 2D position of the robot
    bool goalReached_; // if one of the goals have been reached

private:

};