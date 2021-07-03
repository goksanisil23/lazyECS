#pragma once

#include "WaveFrontTypes.h"
#include <cstdint>
#include <memory>
#include <reactphysics3d/mathematics/Quaternion.h>
#include <reactphysics3d/mathematics/Transform.h>
#include <reactphysics3d/mathematics/Vector3.h>
#include "lazyECS/Systems/Rendering/RenderingSystem.hpp"

class GridCell {

public:

    // -------------------- Methods -------------------- //

/* 
* Individual grid cell representing a 2D tile within the grid
* It's not an entity in lazyECS, but it's rectangle member is used in debug rendering
* and the AABB member used in intersection check with the rest of the Actors
*
* @param min minimum 3D coordinates of the AABB, in WORLD coordinates
* @param max maximum 3D coordinates of the AABB, in WORLD coordinates
* @param x_idx x coordinate of this cell within the 2D grid
* @param z_idx z coordinate of this cell within the 2D grid
*/
    GridCell(const rp3d::Vector3& min, const rp3d::Vector3& max, const uint16_t& x_idx, const uint16_t& z_idx) : 
        aabb_(min, max), distance_(0), x_idx_(x_idx), z_idx_(z_idx)
    {
        auto center  = aabb_.getCenter();
        auto extents = aabb_.getExtent() * 0.5; // half-extents in x,y,z directions (y wont be used)
        rectangle_ = lazyECS::RenderingSystem::DebugRectangle(
            rp3d::Transform(center, rp3d::Quaternion::identity()), extents, static_cast<uint32_t>(GridColor::FREE) // default light gray-ish colour for unoccupied cells        
        );
    }

    bool UpdateOccupancy(const rp3d::AABB& actor_aabb, const CellState& target_cell_state) {
        if(this->aabb_.testCollision(actor_aabb)) {

            // update the color based on the new state of the cell
            switch (target_cell_state) {
                case CellState::OBSTACLE:
                    rectangle_.color = static_cast<uint32_t>(GridColor::OBSTACLE);
                    distance_ = -1;
                    break;
                case CellState::FREE:
                    rectangle_.color = static_cast<uint32_t>(GridColor::FREE);
                    distance_ = 0;
                    break;
                case CellState::GOAL:
                    rectangle_.color = static_cast<uint32_t>(GridColor::GOAL);
                    distance_ = 1;
                    break;
                default:
                    rectangle_.color = static_cast<uint32_t>(GridColor::TRAVELED);
                    break;
            }
            return true;
        }
        return false;
    }

    // -------------------- Members -------------------- //

    rp3d::AABB aabb_; // AABB bounding box representing the volume of the grid cell, also used for intersection/occupancy functions    
    
    lazyECS::RenderingSystem::DebugRectangle rectangle_;

    int distance_; // assigned value for path planning (-1: occupied by obstacle, 0: free and undiscovered, 1: goal)

    uint16_t x_idx_, z_idx_; // x and z indices of this cell in the 2D grid


};