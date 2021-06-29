#pragma once

#include <thread>
#include <chrono>

#include "RenderingSystem.hpp"
#include "PhysicsSystem.hpp"
#include "ECSCore/Orchestrator.hpp"

extern lazyECS::Orchestrator gOrchestrator;


constexpr float APP_STEP_TIME = 0.01; // 100Hz

class Minimal3D {

public:
    // ----------------- Member functions ----------------- //

    Minimal3D(bool isFullscreen, int windowWidth, int windowHeight);

    void main_loop();

    // ----------------- Member variables ----------------- //
    std::shared_ptr<lazyECS::RenderingSystem> renderSys; // pointer to the rendering system to be accessed from draw_contents
    std::shared_ptr<lazyECS::PhysicsSystem> physicsSys;

    std::chrono::_V2::system_clock::time_point prevAppTime_;
    float deltaTime_;
    float timeAccumulator_;       

};