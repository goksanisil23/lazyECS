#include "lazyECS/Components/Transform.hpp"
#include "lazyECS/Components/RigidBody.hpp"

#include "lazyECS/ECSCore/Orchestrator.hpp"

#include "lazyECS/Systems/PhysicsSystem.hpp"
#include "lazyECS/Systems/RenderingSystem2D.hpp"

#include <chrono>
#include <iostream>

lazyECS::Orchestrator gOrchestrator;

int main() {

    gOrchestrator.Init(); // initializes the ECS managers

    // Register the components
    gOrchestrator.RegisterComponentType<lazyECS::Transform>();
    gOrchestrator.RegisterComponentType<lazyECS::RigidBody>();

    // Register the systems
    auto physicsSys = gOrchestrator.RegisterSystem<lazyECS::PhysicsSystem>();
    auto renderSys = gOrchestrator.RegisterSystem<lazyECS::RenderingSystem2D>();

    // Initialize the systems
    physicsSys->Init();
    renderSys->Init();

    // Create the entities and assign components
    std::vector<lazyECS::Entity> entities(10);                       
    const int box_x_pos = 0.0;
    for (int i = 0; i < entities.size(); i++) {
        auto entity = entities.at(i);
        entity = gOrchestrator.CreateEntity();

        gOrchestrator.AddComponent<lazyECS::Transform>(entity, lazyECS::Transform{raylib::Vector3(box_x_pos + i*5.0, 0.0, 0.0), // location
                                                                raylib::Vector3(0.0, 0.0, 0.0),       // rotation
                                                                raylib::Vector3(1.0, 1.0, 1.0)        // scale
                                                                });

        gOrchestrator.AddComponent<lazyECS::RigidBody>(entity, lazyECS::RigidBody{raylib::Vector3(0.0, 0.0, 0.0), // velocity
                                                                raylib::Vector3(0.0, 0.0, -8.0) // acceleration
                                                                });      
    }

    float dt = 0.f;

    // while(ctr < 1000) {
    while(!renderSys->window->ShouldClose()) {

        auto startTime = std::chrono::high_resolution_clock::now();

        physicsSys->Update(dt);
        renderSys->Update(dt);


        auto stopTime = std::chrono::high_resolution_clock::now();

        dt = std::chrono::duration<float, std::chrono::seconds::period>(stopTime-startTime).count();

    }

    return 0;
}