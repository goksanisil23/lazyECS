#pragma once

#include "ECSCore/Types.hpp"

#include <reactphysics3d/reactphysics3d.h>

#include <chrono>

namespace lazyECS {

class PhysicsSystem : public System {
public:
    void Init();

    void Update(float dt);

private:
    // PhysicsCommon is a factory module that is used to create physics world and objects and handling memory and logging
    reactphysics3d::PhysicsCommon physicsCommon;

    reactphysics3d::PhysicsWorld* physicsWorld;

    const float timeStep = 1.0f / 60.0f;

    std::chrono::_V2::system_clock::time_point prevFrameTime = std::chrono::high_resolution_clock::now();

    float timeAccumulator;

    rp3d::Transform prevTrans; // Transform value to calculate interpolation

};

}