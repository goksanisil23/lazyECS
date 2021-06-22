#pragma once

#include <thread>

#include "RenderingSystem.hpp"
#include "PhysicsSystem.hpp"
#include "ECSCore/Orchestrator.hpp"

extern lazyECS::Orchestrator gOrchestrator;

class Minimal3D {

public:
    // ----------------- Member functions ----------------- //

    // Minimal3D(bool isFullscreen, int windowWidth, int windowHeight, std::shared_ptr<lazyECS::RenderingSystem> renderSys_ptr);
    Minimal3D(bool isFullscreen, int windowWidth, int windowHeight);

    void main_loop();

    // ----------------- Member variables ----------------- //
    std::shared_ptr<lazyECS::RenderingSystem> renderSys; // pointer to the rendering system to be accessed from draw_contents
    std::shared_ptr<lazyECS::PhysicsSystem> physicsSys;

    static bool lazyECS_mainloop_active;

};