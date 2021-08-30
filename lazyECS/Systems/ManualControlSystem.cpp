#include "ManualControlSystem.h"

#include "ECSCore/Orchestrator.hpp"

#include <functional>
#include <iostream>
#include <reactphysics3d/configuration.h>
#include <reactphysics3d/mathematics/Transform.h>
#include <reactphysics3d/mathematics/Vector3.h>

extern lazyECS::Orchestrator gOrchestrator; // expected to be defined globally in main

namespace lazyECS {

ManualControlSystem::ManualControlSystem() : move_step_(0.2), rotate_step_(1.0) {};

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

    float ego_yaw, ego_pitch, ego_roll;
    std::tie(ego_yaw, ego_pitch, ego_roll) = gOrchestrator.GetComponent<Transform3D>(manual_entity_).GetEulerOrientation(); 

    switch (key_event.key_button) {
    
    // ROTATE COUNTER-CLOCKWISE
    case KeyboardEvent::KeyButton::KEY_A: {
        auto rotation = rp3d::Quaternion::fromEulerAngles(0,rotate_step_/180.0*reactphysics3d::PI, 0); // 1 degrees around y-axis (heading)
        rigid_body.rp3d_rigidBody->setTransform(rp3d::Transform(cur_trans.getPosition(), rotation * cur_trans.getOrientation()));
        break;
    }

    // BACKWARDS
    case KeyboardEvent::KeyButton::KEY_S: {
        rigid_body.rp3d_rigidBody->setTransform(
                rp3d::Transform(cur_trans.getPosition() - rp3d::Vector3(sin(ego_yaw)*move_step_,0,cos(ego_yaw)*move_step_), cur_trans.getOrientation())
            );
        break;
    }

    // ROTATE CLOCKWISE
    case KeyboardEvent::KeyButton::KEY_D: {
        auto rotation = rp3d::Quaternion::fromEulerAngles(0,-rotate_step_/180.0*reactphysics3d::PI, 0); // 1 degrees around y-axis (heading)
        rigid_body.rp3d_rigidBody->setTransform(rp3d::Transform(cur_trans.getPosition(), rotation * cur_trans.getOrientation()));
        break;
    }

    // FORWARD
    case KeyboardEvent::KeyButton::KEY_W: {
        rigid_body.rp3d_rigidBody->setTransform(
                rp3d::Transform(cur_trans.getPosition() + rp3d::Vector3(sin(ego_yaw)*move_step_,0,cos(ego_yaw)*move_step_), cur_trans.getOrientation())
            );
        break;
    }

    default:
        throw std::runtime_error("This Keyboard Event is not handled");
        break;
    }

}


}