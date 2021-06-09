#include "lazyECS/Components/Transform3D.hpp"
#include "lazyECS/Components/RigidBody3D.hpp"

#include "lazyECS/ECSCore/Orchestrator.hpp"

#include "lazyECS/Systems/PhysicsSystem.hpp"
#include "lazyECS/Systems/RenderingSystem.hpp"

#include <chrono>
#include <iostream>


lazyECS::Orchestrator gOrchestrator; // Orchestrator is a global variable that can be accessed by all systems
                                     // and will exist throughout the lifetime of the program


int main() {

    gOrchestrator.Init(); // initializes the ECS managers

    // Register the components
    gOrchestrator.RegisterComponentType<lazyECS::Transform3D>();
    gOrchestrator.RegisterComponentType<lazyECS::RigidBody3D>();

    return 0;

}