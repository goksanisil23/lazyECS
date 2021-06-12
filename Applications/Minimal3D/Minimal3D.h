#pragma once

#include "openglframework.h"

#include <nanogui/opengl.h>
#include <nanogui/nanogui.h>
#include <GLFW/glfw3.h>

#include "../../lazyECS/Systems/Rendering/RenderingSystem.hpp"

#include "ECSCore/Orchestrator.hpp"

extern lazyECS::Orchestrator gOrchestrator;

class Minimal3D : public nanogui::Screen {

public:

    std::shared_ptr<lazyECS::RenderingSystem> renderSys; // pointer to the rendering system to be accessed from draw_contents

    // Minimal3D(bool isFullscreen, int windowWidth, int windowHeight, std::shared_ptr<lazyECS::RenderingSystem> renderSys_ptr);
    Minimal3D(bool isFullscreen, int windowWidth, int windowHeight);

    void draw_contents();

};