#pragma once

#include "raylib-cpp.hpp" // for Vector library

namespace lazyECS {

    struct RigidBody {
        raylib::Vector3 velocity;
        raylib::Vector3 acceleration;
    };

}