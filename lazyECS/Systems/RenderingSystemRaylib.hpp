#pragma once

#include "ECSCore/Types.hpp"

#include "raylib-cpp.hpp"

#include <memory>

namespace lazyECS {

class RenderingSystemRaylib : public System {
public:
    void Init();

    void Update(float dt);

    std::shared_ptr<raylib::Window> window;
    std::shared_ptr<raylib::Camera3D> camera;

private:
    // raylib::Window window;
};

}

