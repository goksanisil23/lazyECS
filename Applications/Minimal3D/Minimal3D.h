#pragma once

#include "openglframework.h"

#include <nanogui/opengl.h>
#include <nanogui/nanogui.h>
#include <GLFW/glfw3.h>

#include "../../lazyECS/Systems/Rendering/RenderingSystem.hpp"
#include "../../lazyECS/Systems/PhysicsSystem.hpp"

#include "ECSCore/Orchestrator.hpp"

extern lazyECS::Orchestrator gOrchestrator;

class Minimal3D : public nanogui::Screen {

public:

    std::shared_ptr<lazyECS::RenderingSystem> renderSys; // pointer to the rendering system to be accessed from draw_contents
    std::shared_ptr<lazyECS::PhysicsSystem> physicsSys;

    // Minimal3D(bool isFullscreen, int windowWidth, int windowHeight, std::shared_ptr<lazyECS::RenderingSystem> renderSys_ptr);
    Minimal3D(bool isFullscreen, int windowWidth, int windowHeight);

    virtual void draw_contents() override; // render the contents of the application

    virtual bool mouse_button_event(const nanogui::Vector2i& p, int button, bool down, int modifiers) override;
    virtual bool mouse_motion_event(const nanogui::Vector2i& p, const nanogui::Vector2i& rel, int button, int modifiers) override;
    virtual bool scroll_event(const nanogui::Vector2i& p, const nanogui::Vector2f& rel) override;
    virtual bool resize_event(const nanogui::Vector2i& size) override;
    virtual bool keyboard_event(int key, int scancode, int action, int modifiers) override;
};