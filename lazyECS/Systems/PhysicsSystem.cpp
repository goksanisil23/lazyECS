#include "PhysicsSystem.hpp"

#include "Components/RigidBody.hpp"
#include "Components/Transform.hpp"

#include "ECSCore/Orchestrator.hpp"

#include <iostream>

extern lazyECS::Orchestrator gOrchestrator; // expected to be defined globally in main

namespace lazyECS {

void PhysicsSystem::Init(){
    // Set the system signature based on the utilized Components below
    Signature signature;
    signature.set(gOrchestrator.GetComponentTypeId<RigidBody>(), true);
    signature.set(gOrchestrator.GetComponentTypeId<Transform>(), true);
    gOrchestrator.SetSystemSignature<PhysicsSystem>(signature);
}

void PhysicsSystem::Update(float dt) {
    for (auto const& entity : m_entities) {
        RigidBody& rigidBody = gOrchestrator.GetComponent<RigidBody>(entity);
        Transform& transform = gOrchestrator.GetComponent<Transform>(entity);

        transform.position += rigidBody.velocity * dt;
        rigidBody.velocity +=  rigidBody.acceleration* dt;

        std::cout << "entity " << entity << " x,y,z:" << transform.position.x << " " 
                                                      << transform.position.y << " " 
                                                      << transform.position.z << std::endl; 
    }
}

}