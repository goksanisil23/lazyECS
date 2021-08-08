#pragma once

#include "RenderingSystem.hpp"
#include "PhysicsSystem.hpp"

#include <memory>

class Ego {

public:
    Ego() = default;
    
    Ego(std::shared_ptr<lazyECS::RenderingSystem> render_sys_in, 
        std::shared_ptr<lazyECS::PhysicsSystem> physics_sys_in,
        const rp3d::Vector3& ego_pos_in, const rp3d::Quaternion& ego_rot_in);

    void step();

    void sensor_step();

private:

    // passing the shared ownership of some systems to the application, for debug visualization, applying force to ego, etc. 
    std::shared_ptr<lazyECS::RenderingSystem> renderSys;
    std::shared_ptr<lazyECS::PhysicsSystem> physicsSys;

    constexpr static float LIDAR_HOR_FOV = 60.0; // degrees
    constexpr static float LIDAR_VERT_FOV = 20.0; // degrees
    constexpr static float LIDAR_HOR_RES = 3.0; // degrees
    constexpr static float LIDAR_VERT_RES = 5.0; // degrees
    constexpr static float LIDAR_RANGE = 10.0; // meters

    float lidar_vert_count, lidar_hor_count;

    // References to the lazyECS::Transform3D components of the ego
    const rp3d::Vector3& ego_pos_;
    const rp3d::Quaternion& ego_rot_;


    std::tuple<float, float, float> quaternion_to_euler(const rp3d::Quaternion&);

};