#include "PhysicsSystem.hpp"

#include "Components/RigidBody.hpp"
#include "Components/Transform.hpp"

#include "ECSCore/Orchestrator.hpp"

#include "Math/Vec3.hpp"

extern Orchestrator gOrchestrator; // expected to be defined globally in main

void PhysicsSystem::Init(){
    // Set the system signature based on the utilized Components below
    Signature signature;
    signature.set(gOrchestrator.GetComponentTypeId<RigidBody>(), true);
    signature.set(gOrchestrator.GetComponentTypeId<Transform>(), true);
    gOrchestrator.SetSystemSignature<PhysicsSystem>(signature);
}

void PhysicsSystem::Update(float dt) {
    for (auto const& entity : m_entities) {
        auto& rigidBody = gOrchestrator.GetComponent<RigidBody>(entity);
        auto& transform = gOrchestrator.GetComponent<Transform>(entity);
    
        transform.position += rigidBody.velocity * dt;
        rigidBody.velocity +=  rigidBody.acceleration* dt;
        rigidBody.acceleration = Vec3{0.0, 0.0, -9.8};

    }
}
