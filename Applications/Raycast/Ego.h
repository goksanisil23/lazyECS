#pragma once

#include "RenderingSystem.hpp"
#include "PhysicsSystem.hpp"

#include <memory>

class Ego {

public:
    Ego() = default;
    
    Ego(std::shared_ptr<lazyECS::RenderingSystem> render_sys_in, 
        std::shared_ptr<lazyECS::PhysicsSystem> physics_sys_in,
        const lazyECS::Transform3D& transform_in);

    void step();

    void sensor_step();

private:

    // passing the shared ownership of some systems to the application, for debug visualization, applying force to ego, etc. 
    std::shared_ptr<lazyECS::RenderingSystem> renderSys;
    std::shared_ptr<lazyECS::PhysicsSystem> physicsSys;

    constexpr static float LIDAR_HOR_FOV = 60.0; // degrees
    constexpr static float LIDAR_VERT_FOV = 20.0; // degrees
    constexpr static float LIDAR_HOR_RES = 3.0; // degrees
    constexpr static float LIDAR_VERT_RES = 3.0; // degrees
    constexpr static float LIDAR_RANGE = 10.0; // meters

    float lidar_vert_count, lidar_hor_count;

    // References to the lazyECS::Transform3D components of the ego
    const lazyECS::Transform3D& ego_trans_;
    const rp3d::Vector3 lidar_pos_offset;
};