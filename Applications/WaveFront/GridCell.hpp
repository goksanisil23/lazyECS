#pragma once

#include "WaveFrontTypes.h"
#include <cstdint>
#include <memory>
#include <reactphysics3d/mathematics/Quaternion.h>
#include <reactphysics3d/mathematics/Transform.h>
#include <reactphysics3d/mathematics/Vector3.h>
#include "lazyECS/Systems/Rendering/RenderingSystem.hpp"

class GridCell {

enum GridColor {
    GOAL = 0xFFFF00,
    FREE = 0xBAB2B2,
    OBSTACLE = 0xFFFFFF,
    TRAVELED = 0xFFE600
};

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
    GridCell(const rp3d::Vector3& min, const rp3d::Vector3& max, const uint8_t& x_idx, const uint8_t& z_idx) : 
        aabb_(min, max), x_idx_(x_idx), z_idx_(z_idx), value_(0), rectangle_(nullptr)
    {
        auto center  = aabb_.getCenter();
        auto extents = aabb_.getExtent() * 0.5; // half-extents in x,y,z directions (y wont be used)
        // rectangle_ = std::make_unique<lazyECS::RenderingSystem::DebugRectangle>(
        //     rp3d::Transform(center, rp3d::Quaternion::identity()), extents, GridColor::FREE // default light gray-ish colour for unoccupied cells
        // // rectangle_ = lazyECS::RenderingSystem::DebugRectangle(
        //     // rp3d::Transform(center, rp3d::Quaternion::identity()), extents, GridColor::FREE // default light gray-ish colour for unoccupied cells        
        // );
    }

    bool UpdateOccupancy(const rp3d::AABB& actor_aabb, const int& target_value) {
        if(this->aabb_.testCollision(actor_aabb)) {
            value_ = target_value;
            rectangle_->color = 0xFFFFFF;
            // rectangle_.color = 0xFFFFFF;
            std::cout << "Collision at cell:" << unsigned(x_idx_) << " " << unsigned(z_idx_) << std::endl;
            return true;
        }
        return false;
    }

    void InitRectangle() {
        auto center  = aabb_.getCenter();
        auto extents = aabb_.getExtent() * 0.5; // half-extents in x,y,z directions (y wont be used)            
        *rectangle_ = lazyECS::RenderingSystem::DebugRectangle(        
                rp3d::Transform(center, rp3d::Quaternion::identity()), extents, GridColor::FREE // default light gray-ish colour for unoccupied cells        
        );        
    }

    // -------------------- Members -------------------- //

    rp3d::AABB aabb_; // AABB bounding box representing the volume of the grid cell, also used for intersection/occupancy functions    
    
    lazyECS::RenderingSystem::DebugRectangle* rectangle_;
    // std::unique_ptr<lazyECS::RenderingSystem::DebugRectangle> rectangle_; // pointer to the debug rectangle corresponding to this cell in lazyECS Rendering system
    // lazyECS::RenderingSystem::DebugRectangle rectangle_;

    int value_; // assigned value for path planning (-1: occupied by obstacle, 0: free and undiscovered, 1: goal)

    uint8_t x_idx_, z_idx_; // x and z indices of this cell in the 2D grid

};