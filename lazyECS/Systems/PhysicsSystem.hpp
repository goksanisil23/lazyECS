#pragma once

#include "ECSCore/Types.hpp"

#include <reactphysics3d/reactphysics3d.h>

#include <chrono>

namespace lazyECS {

class PhysicsSystem : public System {

public:
    PhysicsSystem();

    void Init();

    void Update(float dt);

private:
    // PhysicsCommon is a factory module that is used to create physics world and objects and handling memory and logging
    rp3d::PhysicsCommon physicsCommon;
    rp3d::PhysicsWorld* physicsWorld;

    float timeStep; // fixed time step for the physics solver iteration
    std::chrono::_V2::system_clock::time_point prevFrameTime;
    float timeAccumulator; // to keep track of the leftover time from last physics iteration
    rp3d::Transform prevTrans; // Transform value to calculate interpolation

};

}