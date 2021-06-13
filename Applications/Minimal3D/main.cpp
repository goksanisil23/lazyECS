// #include "lazyECS/Components/Transform3D.hpp"
// #include "lazyECS/Components/RigidBody3D.hpp"

#include "lazyECS/ECSCore/Orchestrator.hpp"

#include "lazyECS/Systems/PhysicsSystem.hpp"
#include "lazyECS/Systems/Rendering/RenderingSystem.hpp"

#include <chrono>
#include <iostream>

#include "nanogui/nanogui.h"

#include "Minimal3D.h"


lazyECS::Orchestrator gOrchestrator; // Orchestrator is a global variable that can be accessed by all systems
                                     // and will exist throughout the lifetime of the program


int main() {

    // ---------- NANOGUI -------------- //
    nanogui::init();

    {
        bool isFullscreen = false;
        // Get the primary monitor
        GLFWmonitor* monitor = glfwGetPrimaryMonitor();
        const GLFWvidmode* mode = glfwGetVideoMode(monitor);
        // Window size
        int windowWidth = mode->width;
        int windowHeight = mode->height;
        if(!isFullscreen) {
            windowWidth *= 0.3;
            windowHeight *= 0.3;
        }

        // nanogui::ref<Minimal3D> minimal3d_app = new Minimal3D(isFullscreen, windowWidth, windowHeight, renderSys);
        nanogui::ref<Minimal3D> minimal3d_app = new Minimal3D(isFullscreen, windowWidth, windowHeight);
        minimal3d_app->set_visible(true);
        nanogui::mainloop(10);

    }

    nanogui::shutdown();


    return 0;
}