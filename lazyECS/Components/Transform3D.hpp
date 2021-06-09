#pragma once

#include <reactphysics3d/reactphysics3d.h>

#include "raylib-cpp.hpp"

namespace lazyECS {

class Transform3D : public reactphysics3d::Transform {

public:
    // This is uninitialized. PhysicsSystem needs to initialize via world->createRigidBody(Transform)  
    std::shared_ptr<reactphysics3d::Transform> rp3d_transform;

    raylib::Vector3 GetRaylibTransform() {
        return raylib::Vector3( rp3d_transform->getPosition().x,
                                rp3d_transform->getPosition().y,
                                rp3d_transform->getPosition().z);
    }

};

}

