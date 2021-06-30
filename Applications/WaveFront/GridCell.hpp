#pragma once

#include "WaveFrontTypes.h"
#include <reactphysics3d/mathematics/Vector3.h>

class GridCell {

public:
    GridCell(const rp3d::Vector3& min, const rp3d::Vector3& max) : aabb_(min, max) {}

    rp3d::AABB aabb_; // AABB bounding box representing the volume of the grid cell, also used for intersection/occupancy functions
                      
    Position2D coordinate; // 2D position of the cell
};