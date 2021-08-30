#pragma once

#include "ECSCore/Types.hpp"
#include "Components/RigidBody3D.hpp"
#include "Components/Transform3D.hpp"
#include "Components/Tag.h"
#include "ECSCore/EventManagement.hpp"
#include <reactphysics3d/reactphysics3d.h>

namespace lazyECS {

class ManualControlSystem : public System {

public:
    ManualControlSystem();

    void Init(const std::string& tag);

    void SetupSignature();

    void OnKeyboardEvent(const KeyboardEvent& key_event);

private:
    Entity manual_entity_; // entity corresponding to the actor with tag given in Init
    float move_step_; // meters
    float rotate_step_; // degrees
};

}