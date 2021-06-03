#pragma once

#include <reactphysics3d/reactphysics3d.h>

namespace lazyECS {

class RigidBody3D {

public:
    // This is uninitialized. PhysicsSystem needs to initialize via world->createRigidBody(Transform)  
    std::shared_ptr<reactphysics3d::RigidBody> rp3d_rigidBody; 

};

}