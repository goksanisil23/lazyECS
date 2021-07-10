#pragma once

#include <reactphysics3d/mathematics/mathematics_functions.h>
#include <thread>
#include <vector>
#include <unordered_map>
#include <random>
#include <chrono>

#include "RenderingSystem.hpp"
#include "PhysicsSystem.hpp"
#include "RigidBody3D.hpp"
#include "TagSystem.h"
#include "ManualControlSystem.h"
#include "ECSCore/Orchestrator.hpp"

extern lazyECS::Orchestrator gOrchestrator;

class Raycast {
    
public:
    // ----------------- Member functions ----------------- //
    Raycast(bool isFullscreen, int windowWidth, int windowHeight);

    void init();

    void main_loop();

    // ----------------- Member variables ----------------- //
    std::shared_ptr<lazyECS::RenderingSystem> renderSys; // pointer to the rendering system to be accessed from draw_contents
    std::shared_ptr<lazyECS::PhysicsSystem> physicsSys;
    std::shared_ptr<lazyECS::TagSystem> tagSys;
    std::shared_ptr<lazyECS::ManualControlSystem> manControlSys;

private:

    // Application parameters
    float app_step_time_;
    
    std::random_device rand_dev_;
    std::default_random_engine rand_eng_;
    std::uniform_int_distribution<int> rand_dist_;

    std::chrono::_V2::system_clock::time_point prevAppTime_;
    float deltaTime_;
    float timeAccumulator_;     


};