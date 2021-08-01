#pragma once

#include <reactphysics3d/mathematics/mathematics_functions.h>
#include <thread>
#include <vector>
#include <unordered_map>
#include <random>
#include <chrono>

#include "RenderingSystem.hpp"
#include "PhysicsSystem.hpp"
#include "TagSystem.h"
#include "ManualControlSystem.h"

#include "ECSCore/Orchestrator.hpp"

extern lazyECS::Orchestrator gOrchestrator;

// BaseApp is a convenience class that abstracts away lazyECS specific engine setup like registering
// components and systems, configuring them, spawning Entities from the scenario description, etc.
// Users of lazyECS should write their applications by deriving BaseApp class, which gives
// access to the lazyECS systems and components. 
class BaseApp {

public:
    void main_lazyECS_loop();

    explicit BaseApp(bool isFullscreen = false, int windowWidth = 800, int windowHeight = 800);

    // TODO: HAVING VIRTUAL FUNCTION (EVEN IF ITS NOT USED ANYWHERE) CAUSES RUNTIME ERROR
    virtual void main_app_func() {
         std::cout << "default app loop" << std::endl;
    };    

protected:
    // ----------------- Member functions ----------------- //
    // BaseApp(bool isFullscreen = false, int windowWidth = 800, int windowHeight = 800);

    // virtual void main_app_func() = 0; // needs to be implemented by the derived app

    // ----------------- Member variables ----------------- //
    // All systems are listed, even if some won't be needed.
    std::shared_ptr<lazyECS::RenderingSystem> renderSys;
    std::shared_ptr<lazyECS::PhysicsSystem> physicsSys;
    std::shared_ptr<lazyECS::TagSystem> tagSys;
    std::shared_ptr<lazyECS::ManualControlSystem> manControlSys;

    float app_step_time_; // Rate at which Application main loop steps

private:

    std::chrono::_V2::system_clock::time_point prevAppTime_;
    float deltaTime_;
    float timeAccumulator_; 
};