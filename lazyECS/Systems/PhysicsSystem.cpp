#include "PhysicsSystem.hpp"

#include "ECSCore/Orchestrator.hpp"

#include <iostream>

extern lazyECS::Orchestrator gOrchestrator; // expected to be defined globally in main

namespace lazyECS {

PhysicsSystem::PhysicsSystem() : timeStep{PHYSICS_TIME_STEP}, timeAccumulator{0.0f}, 
                                prevFrameTime{std::chrono::high_resolution_clock::now()} 
                                {}


void PhysicsSystem::SetupSignature() {
    // Set the system signature based on the utilized Components below
    Signature signature;
    signature.set(gOrchestrator.GetComponentTypeId<RigidBody3D>(), true);
    signature.set(gOrchestrator.GetComponentTypeId<Transform3D>(), true);
    gOrchestrator.SetSystemSignature<PhysicsSystem>(signature);    
}

void PhysicsSystem::Init(){

    // Setup the React 3D Physics World
    this->physicsWorld = physicsCommon.createPhysicsWorld();

    // Create rp3d rigid body for the entities that have rigid body components
    // Shape of the collider is obtained from the size of the Transform
    for(auto& entity : m_entities) {
        auto& rigidBody = gOrchestrator.GetComponent<RigidBody3D>(entity); // uninitialized here
        auto& transform = gOrchestrator.GetComponent<Transform3D>(entity); // Initial Transform of entities given outside Physics system 
        // we assign the actual rigidBody here, whose initial position is determined by what was assigned to lazyECS::Transform component outside
        rigidBody.rp3d_rigidBody = std::shared_ptr<rp3d::RigidBody>(this->physicsWorld->createRigidBody(transform.rp3d_transform));
        rigidBody.rp3d_collision_shape = physicsCommon.createBoxShape(rp3d::Vector3(transform.halfExtent[0], transform.halfExtent[1], transform.halfExtent[2]));
        rigidBody.rp3d_collider = std::shared_ptr<rp3d::Collider>(rigidBody.rp3d_rigidBody->addCollider(rigidBody.rp3d_collision_shape, rp3d::Transform::identity()));
        rigidBody.rp3d_rigidBody->updateMassPropertiesFromColliders();
        if(rigidBody.isStatic) // o/w dynamic by default
            rigidBody.rp3d_rigidBody->setType(rp3d::BodyType::STATIC);
    }
}

const float& PhysicsSystem::GetInterpFactor() {
    return interpFactor;
}

void PhysicsSystem::Update() {

    auto currentFrameTime = std::chrono::high_resolution_clock::now();
    auto deltaTime = std::chrono::duration<float, std::chrono::seconds::period>(currentFrameTime-prevFrameTime).count();
    this->prevFrameTime = currentFrameTime; // update previous time
    this->timeAccumulator += deltaTime;

    while(this->timeAccumulator >= this->timeStep) {

        // Update physics world with constant time step
        this->physicsWorld->update(this->timeStep);

        // Decrease the accumulated time
        this->timeAccumulator -= this->timeStep;
    }

    // Compute time interpolation factor
    this->interpFactor = this->timeAccumulator / this->timeStep;

    for (auto const& entity : m_entities) {

        int ent_ctr = 0;
        for(auto& entity : m_entities) {
            auto& rigidBody = gOrchestrator.GetComponent<RigidBody3D>(entity); // updated in the physicsWorld update above
            auto& transform = gOrchestrator.GetComponent<Transform3D>(entity);

            // get the updated transform of the body
            rp3d::Transform currentTrans = rigidBody.rp3d_rigidBody->getTransform(); 

            // compute the interpolated transform of the rigid body based on the leftover time
            rp3d::Transform interpTrans = rp3d::Transform::interpolateTransforms(transform.rp3d_prev_transform, currentTrans, interpFactor);

            // use the interpolated transform as the final transform value for the entity (for rendering, AI, etc)
            transform.rp3d_transform = interpTrans;

            // Update the previous transform
            transform.rp3d_prev_transform = currentTrans;

            // Debug print
            // const reactphysics3d::Vector3& position = transform.rp3d_transform.getPosition();
            // std::cout << "entity: " << ent_ctr << " position: " << position.x << " " << position.y << " " << position.z << std::endl; 
            ent_ctr++;
        }
    }
}

}




