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

    // --------------- lAZYECS -------------- //
    gOrchestrator.Init(); // initializes the ECS managers

    // Register the components
    gOrchestrator.RegisterComponentType<lazyECS::Transform3D>();
    gOrchestrator.RegisterComponentType<lazyECS::RigidBody3D>();
    gOrchestrator.RegisterComponentType<lazyECS::Mesh>();

    // Register the systems
    auto physicsSys = gOrchestrator.RegisterSystem<lazyECS::PhysicsSystem>();
    // auto renderSys = gOrchestrator.RegisterSystem<lazyECS::RenderingSystem>();

    // Create the entities and assign components
    std::vector<lazyECS::Entity> entities(10);
    float boxSize[3] = {1.0, 2.0, 3.0};                   
    const int box_x_pos = 0.0;

    for (int i = 0; i < entities.size(); i++) {
        auto entity = entities.at(i);
        entity = gOrchestrator.CreateEntity();

        // Transform component (Initial position)
        reactphysics3d::Vector3 spawn_pos(box_x_pos + i*5.0, 0.0, 0.0);
        reactphysics3d::Quaternion spawn_rot(reactphysics3d::Quaternion::identity());
        lazyECS::Transform3D spawn_trans;
        spawn_trans.rp3d_transform = reactphysics3d::Transform(spawn_pos, spawn_rot);
        spawn_trans.mScalingMatrix = openglframework::Matrix4(  boxSize[0]*0.5f, 0, 0, 0,
                                                                0, boxSize[1]*0.5f, 0, 0,
                                                                0, 0, boxSize[2]*0.5f, 0,
                                                                0, 0, 0, 1);        
        gOrchestrator.AddComponent<lazyECS::Transform3D>(entity,spawn_trans);


        // ADD MESH COMPONENT!!!!!!!!!!!!!!!!
        lazyECS::Mesh mesh;
        gOrchestrator.AddComponent<lazyECS::Mesh>(entity, mesh);

        // Rigid Body component for physical motion
        // lazyECS::RigidBody3D rigid_body; // will be initialized in PhysicsSystem
        // gOrchestrator.AddComponent<lazyECS::RigidBody3D>(entity, rigid_body);
    }

    // Initialize the systems
    // renderSys->Init("/home/goksan/Work/lazyECS/Applications/meshes/cube.obj");

    // -------------------------------------- //

    // nanogui::ref<Minimal3D> minimal3d_app = new Minimal3D(isFullscreen, windowWidth, windowHeight, renderSys);
    nanogui::ref<Minimal3D> minimal3d_app = new Minimal3D(isFullscreen, windowWidth, windowHeight);
    minimal3d_app->set_visible(true);
    nanogui::mainloop(10);


    return 0;
}