#pragma once

#include "raylib-cpp.hpp" // for Vector library

namespace lazyECS {

    struct Transform {
        raylib::Vector3 position;
        raylib::Vector3 rotation;
        raylib::Vector3 scale;
    };

}