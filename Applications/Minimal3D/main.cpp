#include "lazyECS/Components/Transform3D.hpp"
#include "lazyECS/Components/RigidBody3D.hpp"

#include "lazyECS/ECSCore/Orchestrator.hpp"

#include "lazyECS/Systems/PhysicsSystem.hpp"
// #include "lazyECS/Systems/RenderingSystem2D.hpp"

#include <chrono>
#include <iostream>

lazyECS::Orchestrator gOrchestrator;

int main() {

    gOrchestrator.Init(); // initializes the ECS managers

    // Register the components
    gOrchestrator.RegisterComponentType<lazyECS::Transform3D>();
    gOrchestrator.RegisterComponentType<lazyECS::RigidBody3D>();

    // Register the systems
    auto physicsSys = gOrchestrator.RegisterSystem<lazyECS::PhysicsSystem>();
    // auto renderSys = gOrchestrator.RegisterSystem<lazyECS::RenderingSystem2D>();

    // Create the entities and assign components
    std::vector<lazyECS::Entity> entities(10);                       
    const int box_x_pos = 0.0;
    for (int i = 0; i < entities.size(); i++) {
        auto entity = entities.at(i);
        entity = gOrchestrator.CreateEntity();

        reactphysics3d::Vector3 spawn_pos(box_x_pos + i*5.0, 0.0, 0.0);
        reactphysics3d::Quaternion spawn_rot(reactphysics3d::Quaternion::identity());
        lazyECS::Transform3D spawn_trans;
        spawn_trans.rp3d_transform = std::make_shared<reactphysics3d::Transform>(spawn_pos, spawn_rot);

        gOrchestrator.AddComponent<lazyECS::Transform3D>(entity,spawn_trans);

        lazyECS::RigidBody3D rigid_body; // will be initialized in PhysicsSystem

        gOrchestrator.AddComponent<lazyECS::RigidBody3D>(entity, rigid_body);      
    }

    // Initialize the systems
    physicsSys->Init();
    // renderSys->Init();    

    float dt = 0.f;

    // while(ctr < 1000) {
    while(true) {

        auto startTime = std::chrono::high_resolution_clock::now();

        physicsSys->Update(dt);
        // renderSys->Update(dt);

        auto stopTime = std::chrono::high_resolution_clock::now();

        dt = std::chrono::duration<float, std::chrono::seconds::period>(stopTime-startTime).count();
        std::cout << "dt: " << dt << std::endl;

    }

    return 0;
}