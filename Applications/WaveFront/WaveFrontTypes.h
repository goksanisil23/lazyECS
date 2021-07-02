#pragma once

#include <reactphysics3d/reactphysics3d.h>

using Position2D = rp3d::Vector2;

enum class CellState {
    GOAL,
    FREE,
    OBSTACLE,
    TRAVELED
};

enum class GridColor {
    GOAL = 0xEF00FF, // magenta
    FREE = 0xBAB2B2, // light gray
    OBSTACLE = 0x000000, // black
    TRAVELED = 0xFFE600 // yellow
};
