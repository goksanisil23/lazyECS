#pragma once

#include "ECSCore/Types.hpp"
#include "Components/RigidBody3D.hpp"
#include "Components/Transform3D.hpp"
#include "Components/Mesh.hpp"

#include <reactphysics3d/collision/RaycastInfo.h>
#include <reactphysics3d/mathematics/Ray.h>
#include <reactphysics3d/mathematics/Vector3.h>
#include <reactphysics3d/reactphysics3d.h>

#include <chrono>

namespace lazyECS {

// ****************** Engine Settings ****************** //

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

// ****************** RaycastManager ****************** //

class RaycastManager : public rp3d::RaycastCallback {

public:
    RaycastManager() = default;

    float notifyRaycastHit(const rp3d::RaycastInfo& raycastInfo) override {
        hit_points_.push_back(raycastInfo.worldPoint);
        return raycastInfo.hitFraction;
    }

    std::vector<rp3d::Vector3> hit_points_;
};

// ****************** Physics System ****************** //

class PhysicsSystem : public System {

private:

    static constexpr float PHYSICS_TIME_STEP =  0.005;

public:
    PhysicsSystem();

    void Init();

    void SetupSignature();

    void Update();

    const float& GetInterpFactor() const;

    void RayCast(const rp3d::Ray& ray);

    float timeAccumulator; // to keep track of the leftover time from last physics iteration

    float interpFactor; // interpolation ratio [0,1] calculated based on the left over time in physics period

    RaycastManager raycast_manager_;

private:
    // PhysicsCommon is a factory module that is used to create physics world and objects and handling memory and logging
    rp3d::PhysicsCommon physicsCommon;
    rp3d::PhysicsWorld* physicsWorld;

    // Physics engine settings
    EngineSettings mEngineSettings;

    float timeStep; // fixed time step for the physics solver iteration
    std::chrono::_V2::system_clock::time_point prevFrameTime;

};

}