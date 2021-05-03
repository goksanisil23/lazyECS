#include "Components/Transform.hpp"
#include "Components/RigidBody.hpp"

#include "ECSCore/Orchestrator.hpp"

#include "Systems/PhysicsSystem.hpp"

#include <chrono>
#include <iostream>

Orchestrator gOrchestrator;

int main() {

    gOrchestrator.Init(); // initializes the ECS managers

    // Register the components
    gOrchestrator.RegisterComponent<Transform>();
    gOrchestrator.RegisterComponent<RigidBody>();

    // Register the systems
    auto physicsSys = gOrchestrator.RegisterSystem<PhysicsSystem>();

    // Initialize the systems
    physicsSys->Init();

    std::vector<Entity> entities(10);

    for (auto& entity : entities) {
        entity = gOrchestrator.CreateEntity();

        gOrchestrator.AddComponent<Transform>(entity, Transform{.position = Vec3(0.0, 0.0, 0.0),
                                                                .rotation = Vec3(0.0, 0.0, 0.0),
                                                                .scale = Vec3(1.0, 1.0, 1.0)
                                                                });

        gOrchestrator.AddComponent<RigidBody>(entity, RigidBody{.acceleration = Vec3(0.0, 0.0, 0.0),
                                                                .velocity = Vec3(0.0, 0.0, 0.0)
                                                                });
    }


    int ctr = 0;

    float dt = 0.0f;

    while(ctr < 1000) {

        auto startTime = std::chrono::high_resolution_clock::now();

        physicsSys->Update(dt);

        ctr++;

        auto stopTime = std::chrono::high_resolution_clock::now();

        dt = std::chrono::duration<float, std::chrono::seconds::period>(stopTime-startTime).count();

    }

    return 0;
}