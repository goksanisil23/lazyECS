#pragma once

#include <thread>
#include <vector>
#include <unordered_map>
#include <random>
#include <chrono>

#include "RenderingSystem.hpp"
#include "PhysicsSystem.hpp"
#include "RigidBody3D.hpp"
#include "TagSystem.h"
#include "ECSCore/Orchestrator.hpp"

#include "PotentialFieldTypes.h"
#include "Ego.h"
#include "GoalAndObstacle.hpp"

extern lazyECS::Orchestrator gOrchestrator;

// ----------------- Application specific constants ----------------- //
constexpr float GOAL_STD_DEV_ = 2; // (orig: 2) big sigma since we want to be able to pull the ego wherever it's on the map
constexpr float OBST_STD_DEV_ = 0.1; // small sigma since we only want to push the ego if it gets really close to the obstacle
constexpr float EGO_RADIUS_ = 0.05; // 0.05

constexpr float GRID_SIZE_X = 18.0;
constexpr float GRID_SIZE_Z = 18.0;
constexpr float GRID_RES = 0.2; // 0.25

constexpr float APP_STEP_TIME = 0.01; // 100Hz

class PotentialField {

public:
    // ----------------- Member functions ----------------- //

    PotentialField(bool isFullscreen, int windowWidth, int windowHeight);

    void init();

    void main_loop();

    void resetStaticDebugElements(); // used to draw grid-like debug objects for each episode

    float GetResultantForceAtPoint(const p_field::Position&); // calculates the resultant force at this position, based on all goal & obstacles

    void ResetActorPositions(); // teleports the Goal and Obstacles to a random position

    void UpdateDynamicDebugElements(); // used to draw debug elements associated to moving objects, refreshed at render rate

    // ----------------- Member variables ----------------- //
    std::shared_ptr<lazyECS::RenderingSystem> renderSys; // pointer to the rendering system to be accessed from draw_contents
    std::shared_ptr<lazyECS::PhysicsSystem> physicsSys;
    std::shared_ptr<lazyECS::TagSystem> tagSys;

private:

    std::unordered_map<lazyECS::Entity, Ego> egoActors_;
    std::unordered_map<lazyECS::Entity, Obstacle> obstacleActors_;
    std::unordered_map<lazyECS::Entity, Goal> goalActors_;

    bool goalReached_;
    std::random_device rand_dev_;
    std::default_random_engine rand_eng_;
    std::uniform_int_distribution<int> rand_dist_;

    std::chrono::_V2::system_clock::time_point prevAppTime_;
    float deltaTime_;
    float timeAccumulator_;      
};