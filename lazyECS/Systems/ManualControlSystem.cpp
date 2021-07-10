#include "ManualControlSystem.h"

#include "ECSCore/Orchestrator.hpp"

#include <functional>
#include <iostream>
#include <reactphysics3d/mathematics/Transform.h>
#include <reactphysics3d/mathematics/Vector3.h>

extern lazyECS::Orchestrator gOrchestrator; // expected to be defined globally in main

namespace lazyECS {

ManualControlSystem::ManualControlSystem() = default;

void ManualControlSystem::SetupSignature() {
    // Set the system signature based on the utilized Components below
    Signature signature;
    signature.set(gOrchestrator.GetComponentTypeId<RigidBody3D>(), true);
    signature.set(gOrchestrator.GetComponentTypeId<Tag>(), true);
    gOrchestrator.SetSystemSignature<ManualControlSystem>(signature);        
}

void ManualControlSystem::Init(const std::string& tag) {
    // Register the interest to the Keyboard events
    gOrchestrator.event_bus_->subscribe<KeyboardEvent>(
        std::bind(&ManualControlSystem::OnKeyboardEvent, this, std::placeholders::_1)
    );

    // Find the ego actor (assumes 1 ego, terminates after finding first)
    for(const auto& entity : m_entities) {
        if(gOrchestrator.GetComponent<Tag>(entity).mTag == tag) {
            manual_entity_ = entity;
            break;            
        }
    }

}

void ManualControlSystem::OnKeyboardEvent(const KeyboardEvent& key_event) {


    auto& rigid_body = gOrchestrator.GetComponent<RigidBody3D>(manual_entity_);

    const auto& cur_trans = rigid_body.rp3d_rigidBody->getTransform();

    switch (key_event.key_button) {
    case KeyboardEvent::KeyButton::KEY_A :
        rigid_body.rp3d_rigidBody->setTransform(
                rp3d::Transform(cur_trans.getPosition() + rp3d::Vector3(-0.2,0,0), cur_trans.getOrientation())
            );
        break;
    case KeyboardEvent::KeyButton::KEY_S :
        rigid_body.rp3d_rigidBody->setTransform(
                rp3d::Transform(cur_trans.getPosition() + rp3d::Vector3(0,0,0.2), cur_trans.getOrientation())
            );    
        break;
    case KeyboardEvent::KeyButton::KEY_D :
        rigid_body.rp3d_rigidBody->setTransform(
                rp3d::Transform(cur_trans.getPosition() + rp3d::Vector3(0.2,0,0), cur_trans.getOrientation())
            );    
        break;
    case KeyboardEvent::KeyButton::KEY_W :
        rigid_body.rp3d_rigidBody->setTransform(
                rp3d::Transform(cur_trans.getPosition() + rp3d::Vector3(0,0,-0.2), cur_trans.getOrientation())
            );    
        break;
    default:
        throw std::runtime_error("This Keyboard Event is not handled");
        break;
    }

}


}