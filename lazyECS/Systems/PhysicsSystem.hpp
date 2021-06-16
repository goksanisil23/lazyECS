#pragma once

#include "ECSCore/Types.hpp"
#include "Components/RigidBody3D.hpp"
#include "Components/Transform3D.hpp"

#include <reactphysics3d/reactphysics3d.h>

#include <chrono>

namespace lazyECS {

struct EngineSettings {

public:
    long double elapsedTime; // Elapsed time (in seconds)
    float timeStep;          // Current time step (in seconds)
    unsigned int numVelocitySolverIterations; // # of velocity solver iterations
    unsigned int numPositionSolverIterations; // # position solver iterations
    bool isSleepingEnabled; // True if sleeping mode is enabled
    float timeBeforeSleep; // Time of inactivity before a body sleep
    float sleepLinearVelocity; // Threshold of lineary velocity for sleeping
    float sleepAngularVelocity; // Threshold of lineary angular for sleeping
    bool isGravityEnabled; // True if gravity is enabled
    rp3d::Vector3 gravity; // Gravity vector

    // Constructor
    EngineSettings() : elapsedTime(0.0f), timeStep(0.0f) {}

    // Return default Physics Engine settings
    static EngineSettings defaultSettings() {
        EngineSettings defaultSettings;

        rp3d::PhysicsWorld::WorldSettings worldSettings;
        defaultSettings.timeStep = 1.0f / 60.0f;
        defaultSettings.numVelocitySolverIterations = worldSettings.defaultVelocitySolverNbIterations;
        defaultSettings.numPositionSolverIterations = worldSettings.defaultPositionSolverNbIterations;
        defaultSettings.isSleepingEnabled = worldSettings.isSleepingEnabled;
        defaultSettings.timeBeforeSleep = worldSettings.defaultTimeBeforeSleep;
        defaultSettings.sleepLinearVelocity = worldSettings.defaultSleepLinearVelocity;
        defaultSettings.sleepAngularVelocity = worldSettings.defaultSleepAngularVelocity;
        defaultSettings.isGravityEnabled = true;

        return defaultSettings;
    }

};

class PhysicsSystem : public System {

public:
    PhysicsSystem();

    void Init();

    void Update();

    const float& GetInterpFactor();

    float timeAccumulator; // to keep track of the leftover time from last physics iteration

    float interpFactor; // interpolation ratio [0,1] calculated based on the left over time in physics period 

private:
    // PhysicsCommon is a factory module that is used to create physics world and objects and handling memory and logging
    rp3d::PhysicsCommon physicsCommon;
    rp3d::PhysicsWorld* physicsWorld;

    // Physics engine settings
    EngineSettings mEngineSettings;

    // Interpolation factor for the bodies in the current frame
    float mInterpolationFactor;

    float timeStep; // fixed time step for the physics solver iteration
    std::chrono::_V2::system_clock::time_point prevFrameTime;
    rp3d::Transform prevTrans; // Transform value to calculate interpolation

};

}